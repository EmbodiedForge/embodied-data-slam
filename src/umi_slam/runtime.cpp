#include "umi_slam/runtime.hpp"

#include "process_util.hpp"
#include "slam_common/sha256.hpp"
#include "slam_common/trajectory_io.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <tlhelp32.h>
#else
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace umi_slam {
namespace {

using slam_common::JsonValue;

std::string random_hex(size_t n_chars) {
    static thread_local std::mt19937_64 rng(std::random_device{}());
    static constexpr char kHex[] = "0123456789abcdef";
    std::string out(n_chars, '0');
    for (size_t i = 0; i < n_chars; ++i) {
        out[i] = kHex[rng() % 16];
    }
    return out;
}

std::string lower_str(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

bool ends_with_ci(const std::string& s, const std::string& suffix) {
    if (s.size() < suffix.size()) return false;
    return lower_str(s.substr(s.size() - suffix.size())) == lower_str(suffix);
}

bool is_subpath(const std::filesystem::path& child, const std::filesystem::path& parent) {
    std::error_code ec;
    auto c = std::filesystem::weakly_canonical(child, ec);
    auto p = std::filesystem::weakly_canonical(parent, ec);
    auto it_c = c.begin(), it_p = p.begin();
    for (; it_p != p.end(); ++it_p, ++it_c) {
        if (it_c == c.end() || *it_c != *it_p) return false;
    }
    return true;
}

std::vector<std::filesystem::path> station_roots() {
    std::filesystem::path exe_dir = current_executable_dir();
    std::vector<std::filesystem::path> roots = {exe_dir, exe_dir.parent_path(), std::filesystem::current_path()};
    if (exe_dir.has_parent_path() && exe_dir.parent_path().has_parent_path()) {
        roots.push_back(exe_dir.parent_path().parent_path());
    }
    return roots;
}

void validate_run_args(
    const std::filesystem::path& source_in,
    const std::filesystem::path& output_in,
    const std::filesystem::path& settings_in,
    const std::filesystem::path& profile_in,
    int stride,
    int max_frames,
    double timeout,
    std::filesystem::path& source,
    std::filesystem::path& output,
    std::filesystem::path& settings,
    std::filesystem::path& profile) {
    source = std::filesystem::weakly_canonical(source_in);
    output = std::filesystem::weakly_canonical(output_in);
    settings = std::filesystem::weakly_canonical(settings_in);
    profile = std::filesystem::weakly_canonical(profile_in);
    if (!std::filesystem::is_directory(source) ||
        !std::filesystem::is_regular_file(settings) ||
        !std::filesystem::is_regular_file(profile)) {
        throw std::invalid_argument("输入、标定或设备配置不存在");
    }
    if (source == output || is_subpath(source, output) || is_subpath(output, source)) {
        throw std::invalid_argument("输入和结果目录不能互相嵌套");
    }
    if (std::filesystem::exists(output)) {
        throw std::invalid_argument("结果目录已存在，不能覆盖");
    }
    if (stride < 1 || max_frames < 0 || !std::isfinite(timeout) || timeout < 0.0) {
        throw std::invalid_argument("运行参数无效");
    }
}

#ifdef _WIN32
std::wstring utf8_to_utf16(const std::string& s) {
    if (s.empty()) return {};
    int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    if (n <= 0) return {};
    std::wstring w(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
    return w;
}

std::vector<DWORD> windows_descendant_pids(DWORD root_pid) {
    std::vector<DWORD> result;
    if (root_pid == 0) return result;
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE) return result;
    std::map<DWORD, std::vector<DWORD>> children;
    PROCESSENTRY32W pe{};
    pe.dwSize = sizeof(pe);
    if (Process32FirstW(snap, &pe)) {
        do {
            children[pe.th32ParentProcessID].push_back(pe.th32ProcessID);
        } while (Process32NextW(snap, &pe));
    }
    CloseHandle(snap);

    std::set<DWORD> seen;
    std::vector<DWORD> queue = {root_pid};
    while (!queue.empty()) {
        DWORD cur = queue.back();
        queue.pop_back();
        auto it = children.find(cur);
        if (it == children.end()) continue;
        for (DWORD ch : it->second) {
            if (ch != 0 && ch != root_pid && !seen.count(ch)) {
                seen.insert(ch);
                result.push_back(ch);
                queue.push_back(ch);
            }
        }
    }
    return result;
}

void windows_kill_pids(const std::vector<DWORD>& pids) {
    for (DWORD p : pids) {
        if (p == 0) continue;
        HANDLE h = OpenProcess(PROCESS_TERMINATE | SYNCHRONIZE, FALSE, p);
        if (!h) h = OpenProcess(PROCESS_TERMINATE, FALSE, p);
        if (h) {
            TerminateProcess(h, 1);
            WaitForSingleObject(h, 2000);
            CloseHandle(h);
        }
    }
}
#endif

}  // namespace

std::optional<NativeBackendInfo> find_native_backend(
    const std::optional<std::filesystem::path>& backend_path) {
    std::vector<std::filesystem::path> candidates;
    if (backend_path.has_value()) {
        candidates.push_back(std::filesystem::weakly_canonical(*backend_path));
    } else {
#ifdef _WIN32
        const char* os_build = "software/umi_slam/.build-windows/backend.json";
        const char* os_dist = "dist/SLAMStation/backend/backend.json";
#elif defined(__APPLE__)
        const char* os_build = "software/umi_slam/.build-macos/backend.json";
        const char* os_dist = "dist-macos/SLAMStation/backend/backend.json";
#else
        const char* os_build = "software/umi_slam/.build-linux/backend.json";
        const char* os_dist = "dist-linux/SLAMStation/backend/backend.json";
#endif
        for (const auto& root : station_roots()) {
            candidates.push_back(root / "backend/backend.json");
            candidates.push_back(root / "software/umi_slam/.build/backend.json");
            candidates.push_back(root / os_build);
            candidates.push_back(root / os_dist);
            candidates.push_back(root / "dist/SLAMStation/backend/backend.json");
        }
    }

    std::filesystem::path exe_dir = current_executable_dir();
#ifdef _WIN32
    std::filesystem::path pipeline_exe = exe_dir / "umi_slam_pipeline.exe";
#else
    std::filesystem::path pipeline_exe = exe_dir / "umi_slam_pipeline";
#endif

    for (const auto& bj : candidates) {
        if (!std::filesystem::is_regular_file(bj)) continue;
        JsonValue cfg = slam_common::read_json_safe(bj);
        if (!cfg.is_object() || !cfg.get("binary").is_string() || !cfg.get("vocabulary").is_string()) continue;
        std::filesystem::path base = bj.parent_path();
        auto res = [&](const std::string& s) {
            std::filesystem::path p = std::filesystem::u8path(s);
            return p.is_absolute() ? p : std::filesystem::weakly_canonical(base / p);
        };
        std::filesystem::path binary = res(cfg.get("binary").as_string());
        std::filesystem::path vocab = res(cfg.get("vocabulary").as_string());
#ifdef _WIN32
        bool os_is_win = true;
#else
        bool os_is_win = false;
#endif
        if (!backend_path.has_value() && os_is_win != ends_with_ci(binary.filename().string(), ".exe")) {
            continue;
        }
        if (!std::filesystem::is_regular_file(binary) || !std::filesystem::is_regular_file(vocab)) {
            continue;
        }
        NativeBackendInfo info;
        info.backend_json = bj;
        info.binary = binary;
        info.vocabulary = vocab;
        info.pipeline_exe = pipeline_exe;
        if (cfg.get("library_paths").is_array()) {
            for (const auto& lp : cfg.get("library_paths").as_array()) {
                if (lp.is_string() && !lp.as_string().empty()) info.library_paths.push_back(res(lp.as_string()));
            }
        }
        return info;
    }
    return std::nullopt;
}

std::vector<std::string> native_command(
    const std::filesystem::path& source_in,
    const std::filesystem::path& output_in,
    const std::filesystem::path& settings_in,
    const std::filesystem::path& profile_in,
    int stride,
    int max_frames,
    double timeout,
    bool prepare_only,
    const std::optional<std::filesystem::path>& backend_path,
    bool viewer) {
    std::filesystem::path source, output, settings, profile;
    validate_run_args(source_in, output_in, settings_in, profile_in, stride, max_frames, timeout, source, output, settings, profile);
    auto info = find_native_backend(backend_path);
    if (!info.has_value()) {
        throw std::runtime_error("未找到本地编译的 SLAM 后端（backend.json / umi_slam_runner）");
    }
    if (output.has_parent_path()) std::filesystem::create_directories(output.parent_path());

    std::vector<std::string> cmd;
    if (std::filesystem::is_regular_file(info->pipeline_exe)) {
        cmd.push_back(info->pipeline_exe.string());
    } else {
#ifdef _WIN32
        cmd.push_back((current_executable_dir() / "slam_station.exe").string());
#else
        cmd.push_back((current_executable_dir() / "slam_station").string());
#endif
        cmd.push_back("--run-pipeline");
    }
    std::ostringstream to_ss;
    to_ss << timeout;
    cmd.insert(cmd.end(), {
        "--input", source.string(),
        "--output", output.string(),
        "--settings", settings.string(),
        "--profile", profile.string(),
        "--backend", info->backend_json.string(),
        "--stride", std::to_string(stride),
        "--max-frames", std::to_string(max_frames),
        "--timeout", to_ss.str()
    });
    if (prepare_only) cmd.push_back("--prepare-only");
    if (viewer) cmd.push_back("--viewer");
    return cmd;
}

std::vector<std::string> command(
    const std::filesystem::path& source,
    const std::filesystem::path& output,
    const std::filesystem::path& settings,
    const std::filesystem::path& profile,
    int stride,
    int max_frames,
    double timeout,
    bool prepare_only,
    const std::optional<std::filesystem::path>& backend_path,
    bool viewer) {
    return native_command(source, output, settings, profile, stride, max_frames, timeout, prepare_only, backend_path, viewer);
}

JsonValue diagnose(const std::optional<std::filesystem::path>& backend_path) {
    auto info = find_native_backend(backend_path);
    if (!info.has_value()) {
        throw std::runtime_error("未找到本地编译的 SLAM 后端（backend.json / umi_slam_runner），请先编译后端");
    }
    JsonValue out = JsonValue::object();
    out["mode"] = "native";
    out["backend"] = info->backend_json.string();
    out["binary"] = info->binary.string();
    out["vocabulary"] = info->vocabulary.string();
    return out;
}

int execute_command(
    const std::vector<std::string>& cmd,
    std::atomic<bool>& cancel,
    const std::filesystem::path& log_path,
    const RuntimeHooks* hooks) {
    if (cancel.load()) return 130;

#ifdef _WIN32
    SECURITY_ATTRIBUTES sa{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
    std::wstring wlog = log_path.wstring();
    HANDLE log_h = CreateFileW(wlog.c_str(), GENERIC_WRITE,
                               FILE_SHARE_READ | FILE_SHARE_WRITE, &sa, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    STARTUPINFOW si{};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdOutput = log_h;
    si.hStdError = log_h;
    PROCESS_INFORMATION pi{};
    std::string cmdline = build_win_cmdline(cmd);
    std::wstring wcmd = utf8_to_utf16(cmdline);
    DWORD cflags = CREATE_NEW_PROCESS_GROUP | CREATE_NO_WINDOW;
    if (!CreateProcessW(nullptr, wcmd.data(), nullptr, nullptr, TRUE, cflags, nullptr, nullptr, &si, &pi)) {
        if (log_h != INVALID_HANDLE_VALUE) CloseHandle(log_h);
        throw std::runtime_error("Failed to launch process: " + cmdline);
    }
    if (log_h != INVALID_HANDLE_VALUE) CloseHandle(log_h);

    while (true) {
        DWORD w = WaitForSingleObject(pi.hProcess, 200);
        if (w == WAIT_OBJECT_0) {
            DWORD ec = 0;
            GetExitCodeProcess(pi.hProcess, &ec);
            CloseHandle(pi.hThread);
            CloseHandle(pi.hProcess);
            return cancel.load() ? 130 : static_cast<int>(ec);
        }
        if (cancel.load()) {
            auto descendants = windows_descendant_pids(pi.dwProcessId);
            GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT, pi.dwProcessId);
            if (WaitForSingleObject(pi.hProcess, static_cast<DWORD>(kWindowsCtrlBreakGraceS * 1000)) == WAIT_OBJECT_0) {
                auto post = windows_descendant_pids(pi.dwProcessId);
                descendants.insert(descendants.end(), post.begin(), post.end());
                windows_kill_pids(descendants);
                CloseHandle(pi.hThread);
                CloseHandle(pi.hProcess);
                return 130;
            }
            auto more = windows_descendant_pids(pi.dwProcessId);
            descendants.insert(descendants.end(), more.begin(), more.end());
            std::vector<std::string> tk = {"taskkill", "/T", "/F", "/PID", std::to_string(pi.dwProcessId)};
            if (hooks && hooks->on_taskkill) hooks->on_taskkill(tk);
            run_subprocess_capture(tk);
            auto last = windows_descendant_pids(pi.dwProcessId);
            descendants.insert(descendants.end(), last.begin(), last.end());
            windows_kill_pids(descendants);
            if (WaitForSingleObject(pi.hProcess, 100) != WAIT_OBJECT_0) {
                TerminateProcess(pi.hProcess, 1);
                WaitForSingleObject(pi.hProcess, static_cast<DWORD>(kKillWaitTimeoutS * 1000));
            }
            CloseHandle(pi.hThread);
            CloseHandle(pi.hProcess);
            return 130;
        }
    }
#else
    (void)hooks;
    pid_t pid = fork();
    if (pid == 0) {
        setsid();
        FILE* lf = fopen(log_path.c_str(), "w");
        if (lf) {
            dup2(fileno(lf), STDOUT_FILENO);
            dup2(fileno(lf), STDERR_FILENO);
            fclose(lf);
        }
        std::vector<char*> argv_exec;
        for (const auto& s : cmd) argv_exec.push_back(const_cast<char*>(s.c_str()));
        argv_exec.push_back(nullptr);
        execvp(cmd[0].c_str(), argv_exec.data());
        _exit(127);
    }
    while (true) {
        int st = 0;
        pid_t wp = waitpid(pid, &st, WNOHANG);
        if (wp == pid) {
            return cancel.load() ? 130 : (WIFEXITED(st) ? WEXITSTATUS(st) : 1);
        }
        if (cancel.load()) {
            killpg(pid, SIGTERM);
            auto term_end = std::chrono::steady_clock::now() + std::chrono::duration<double>(kPosixTermTimeoutS);
            while (std::chrono::steady_clock::now() < term_end) {
                if (waitpid(pid, &st, WNOHANG) == pid) return 130;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            killpg(pid, SIGKILL);
            waitpid(pid, &st, 0);
            return 130;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
#endif
}

Job::Job(
    const std::filesystem::path& source_in,
    const std::filesystem::path& parent_in,
    const std::filesystem::path& settings_in,
    const std::filesystem::path& device_in,
    int stride,
    int max_frames,
    double timeout,
    bool prepare_only,
    const std::optional<std::filesystem::path>& backend_path,
    bool viewer,
    JobHooks hooks)
    : backend_path_(backend_path), hooks_(std::move(hooks)) {
    std::filesystem::path source = std::filesystem::weakly_canonical(source_in);
    std::filesystem::path parent = std::filesystem::weakly_canonical(parent_in);
    std::filesystem::path settings = std::filesystem::weakly_canonical(settings_in);
    std::filesystem::path device = std::filesystem::weakly_canonical(device_in);
    if (source == parent || is_subpath(parent, source) || is_subpath(source, parent)) {
        throw std::invalid_argument("采集目录和结果保存目录不能互相嵌套");
    }
    JsonValue config = slam_common::load_device(device);
    slam_common::check_dataset(source, config);
    if (!std::filesystem::is_regular_file(settings)) {
        throw std::invalid_argument("请选择与当前设备对应的标定 YAML");
    }

    // Validate parameters before allocating job folder
    native_command(source, parent / ("_validate_" + random_hex(16)), settings, device,
                   stride, max_frames, timeout, prepare_only, backend_path, viewer);

    std::time_t now_t = std::time(nullptr);
    std::tm tm_now{};
#ifdef _WIN32
    localtime_s(&tm_now, &now_t);
#else
    localtime_r(&now_t, &tm_now);
#endif
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "run_%Y%m%d_%H%M%S_", &tm_now);
    folder_ = parent / (std::string(stamp) + random_hex(8));
    std::filesystem::create_directories(folder_);
    result_ = folder_ / "result";

    std::filesystem::path cfg_dir = folder_ / "config";
    std::filesystem::create_directories(cfg_dir);
    std::filesystem::copy_file(settings, cfg_dir / "settings.yaml", std::filesystem::copy_options::overwrite_existing);
    slam_common::save_json(cfg_dir / "device.json", config);
    slam_common::save_json(cfg_dir / "profile.json", slam_common::pipeline_profile(config));

    cmd_ = native_command(
        source, result_, cfg_dir / "settings.yaml", cfg_dir / "profile.json",
        stride, max_frames, timeout, prepare_only, backend_path, viewer);

    double unix_now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    state_ = JsonValue::object();
    state_["schema_version"] = static_cast<int64_t>(1);
    state_["status"] = "queued";
    state_["mode"] = "native";
    state_["viewer"] = viewer;
    state_["source"] = source.string();
    state_["device"] = config;
    state_["settings_source"] = settings.string();
    state_["stride"] = static_cast<int64_t>(stride);
    state_["max_frames"] = static_cast<int64_t>(max_frames);
    state_["timeout"] = timeout;
    state_["prepare_only"] = prepare_only;
    state_["created_unix_s"] = unix_now;
    state_["calibration_sha256"] = slam_common::sha256_file(cfg_dir / "settings.yaml");
    JsonValue cmd_arr = JsonValue::array();
    for (const auto& a : cmd_) cmd_arr.push_back(a);
    state_["command"] = cmd_arr;
    save_state_locked();
}

Job::~Job() {
    if (thread_.joinable()) {
        stop();
        thread_.join();
    }
}

void Job::save_state_locked() {
    slam_common::save_json(folder_ / "station_job.json", state_);
}

void Job::reconcile_cancelled_backend_state() {
    std::filesystem::path sp = result_ / "job.json";
    if (!std::filesystem::is_regular_file(sp)) return;
    JsonValue bs = slam_common::read_json_safe(sp);
    if (bs.is_object() && bs.get("status").is_string()) {
        const std::string& st = bs.get("status").as_string();
        if (st == "preparing" || st == "running") {
            bs["status"] = "cancelled";
            bs["finished_unix_s"] = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            slam_common::save_json(sp, bs);
        }
    }
}

void Job::start() {
    if (thread_.joinable()) throw std::runtime_error("任务不能重复启动");
    running_.store(true);
    thread_ = std::thread([this]() { run(); });
}

void Job::run() {
    running_.store(true);
    std::filesystem::path host_log = folder_ / "host.log";
    try {
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            state_["status"] = "checking";
            save_state_locked();
        }
        JsonValue diag = hooks_.diagnose_fn
            ? hooks_.diagnose_fn(backend_path_)
            : diagnose(backend_path_);
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            state_["backend"] = diag;
        }
        if (cancel_.load()) {
            std::lock_guard<std::mutex> lk(state_mtx_);
            state_["status"] = "cancelled";
            state_["finished_unix_s"] = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            save_state_locked();
            running_.store(false);
            return;
        }
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            state_["status"] = "running";
            save_state_locked();
        }
        int code = hooks_.execute_fn
            ? hooks_.execute_fn(cmd_, cancel_, host_log)
            : execute_command(cmd_, cancel_, host_log);
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            state_["exit_code"] = static_cast<int64_t>(code);
        }
        if (code == 130 || cancel_.load()) {
            reconcile_cancelled_backend_state();
            std::lock_guard<std::mutex> lk(state_mtx_);
            state_["status"] = "cancelled";
            state_["finished_unix_s"] = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            save_state_locked();
            running_.store(false);
            return;
        }

        std::filesystem::path sp = result_ / "job.json";
        JsonValue bs = std::filesystem::is_regular_file(sp) ? slam_common::read_json_safe(sp) : JsonValue::object();
        std::string expected = state_.get("prepare_only").as_bool() ? "prepared" : "completed";
        std::string bs_status = (bs.is_object() && bs.get("status").is_string()) ? bs.get("status").as_string() : "";
        if (code != 0 || bs_status != expected) {
            std::string err = (bs.contains("error") && bs.get("error").is_string() && !bs.get("error").as_string().empty())
                ? bs.get("error").as_string()
                : ("后端未完成，退出码 " + std::to_string(code) + "，请查看日志");
            throw std::runtime_error(err);
        }
        if (expected == "completed") {
            if (bs.contains("coverage_fraction")) {
                std::lock_guard<std::mutex> lk(state_mtx_);
                state_["coverage_fraction"] = bs.get("coverage_fraction");
            }
            auto exported = slam_common::export_target(result_, state_.get("device"));
            if (exported.has_value()) {
                std::lock_guard<std::mutex> lk(state_mtx_);
                state_["target_trajectory"] = exported->string();
            }
        }
        std::lock_guard<std::mutex> lk(state_mtx_);
        state_["status"] = expected;
    } catch (const std::exception& exc) {
        {
            std::ofstream lf(host_log, std::ios::app);
            lf << "\nERROR: " << exc.what() << "\n";
        }
        std::lock_guard<std::mutex> lk(state_mtx_);
        state_["status"] = "failed";
        state_["error"] = exc.what();
    }
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        state_["finished_unix_s"] = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        save_state_locked();
    }
    running_.store(false);
}

void Job::stop() { cancel_.store(true); }

void Job::wait() {
    if (thread_.joinable()) thread_.join();
}

bool Job::active() const { return running_.load(); }

JsonValue Job::state() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return state_;
}

}  // namespace umi_slam
