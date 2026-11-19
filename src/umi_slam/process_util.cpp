#include "process_util.hpp"

#include <cerrno>
#include <cstdlib>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace umi_slam {
namespace {

#ifdef _WIN32
// Narrow strings in our processes are UTF-8 (activeCodePage manifest); convert to UTF-16
// so children receive proper Unicode regardless of their own ANSI code page.
std::wstring utf8_to_utf16(const std::string& s) {
    if (s.empty()) return std::wstring();
    int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    if (n <= 0) return std::wstring();
    std::wstring w(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
    return w;
}
#endif

}  // namespace

std::string quote_win_arg(const std::string& arg) {
    if (arg.find_first_of(" \t\n\v\"") == std::string::npos && !arg.empty()) return arg;
    std::string out = "\"";
    size_t backslashes = 0;
    for (char c : arg) {
        if (c == '\\') {
            ++backslashes;
        } else if (c == '"') {
            out.append(backslashes * 2 + 1, '\\');
            out.push_back('"');
            backslashes = 0;
        } else {
            out.append(backslashes, '\\');
            backslashes = 0;
            out.push_back(c);
        }
    }
    out.append(backslashes * 2, '\\');
    out.push_back('"');
    return out;
}

std::string build_win_cmdline(const std::vector<std::string>& args) {
    std::string line;
    for (size_t i = 0; i < args.size(); ++i) {
        if (i > 0) line.push_back(' ');
        line += quote_win_arg(args[i]);
    }
    return line;
}

std::pair<int, std::string> run_subprocess_capture(const std::vector<std::string>& args) {
#ifdef _WIN32
    SECURITY_ATTRIBUTES sa{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
    HANDLE rd = nullptr, wr = nullptr;
    if (!CreatePipe(&rd, &wr, &sa, 0)) return {-1, ""};
    SetHandleInformation(rd, HANDLE_FLAG_INHERIT, 0);

    STARTUPINFOW si{};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdOutput = wr;
    si.hStdError = wr;
    PROCESS_INFORMATION pi{};
    std::wstring cmd = utf8_to_utf16(build_win_cmdline(args));
    BOOL ok = CreateProcessW(nullptr, cmd.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);
    CloseHandle(wr);
    if (!ok) {
        CloseHandle(rd);
        return {-1, ""};
    }
    std::string out;
    char buf[4096];
    DWORD n = 0;
    while (ReadFile(rd, buf, sizeof(buf), &n, nullptr) && n > 0) {
        out.append(buf, n);
    }
    CloseHandle(rd);
    WaitForSingleObject(pi.hProcess, 20000);
    DWORD ec = 1;
    GetExitCodeProcess(pi.hProcess, &ec);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return {static_cast<int>(ec), out};
#else
    int pipefd[2];
    if (pipe(pipefd) != 0) return {-1, ""};
    pid_t pid = fork();
    if (pid == 0) {
        close(pipefd[0]);
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[1], STDERR_FILENO);
        close(pipefd[1]);
        std::vector<char*> argv_exec;
        for (const auto& a : args) argv_exec.push_back(const_cast<char*>(a.c_str()));
        argv_exec.push_back(nullptr);
        execvp(args[0].c_str(), argv_exec.data());
        _exit(127);
    }
    close(pipefd[1]);
    std::string out;
    char buf[4096];
    ssize_t n = 0;
    while ((n = read(pipefd[0], buf, sizeof(buf))) > 0) out.append(buf, static_cast<size_t>(n));
    close(pipefd[0]);
    int status = 0;
    waitpid(pid, &status, 0);
    return {WIFEXITED(status) ? WEXITSTATUS(status) : 1, out};
#endif
}

int run_inherit(const std::vector<std::string>& args, const RunInheritOptions& options) {
    if (args.empty()) return -1;
#ifdef _WIN32
    std::vector<std::pair<std::string, std::optional<std::string>>> saved_env;
    for (const auto& name : options.unset_env) {
        DWORD need = GetEnvironmentVariableA(name.c_str(), nullptr, 0);
        std::optional<std::string> old;
        if (need > 0) {
            std::string v(static_cast<size_t>(need), '\0');
            DWORD got = GetEnvironmentVariableA(name.c_str(), v.data(), need);
            if (got > 0) {
                v.resize(got);
                old = v;
            } else {
                old = std::string();
            }
        }
        saved_env.emplace_back(name, old);
        SetEnvironmentVariableA(name.c_str(), nullptr);
    }

    std::wstring wcmd = utf8_to_utf16(build_win_cmdline(args));
    std::wstring wcwd;
    const wchar_t* cwd_ptr = nullptr;
    if (!options.cwd.empty()) {
        wcwd = utf8_to_utf16(options.cwd.string());
        cwd_ptr = wcwd.c_str();
    }
    STARTUPINFOW si{};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    BOOL ok = CreateProcessW(nullptr, wcmd.data(), nullptr, nullptr, TRUE, 0, nullptr, cwd_ptr, &si, &pi);
    for (const auto& kv : saved_env) {
        if (kv.second.has_value()) SetEnvironmentVariableA(kv.first.c_str(), kv.second->c_str());
        else SetEnvironmentVariableA(kv.first.c_str(), nullptr);
    }
    if (!ok || wcmd.empty()) return -1;
    WaitForSingleObject(pi.hProcess, INFINITE);
    DWORD ec = 1;
    GetExitCodeProcess(pi.hProcess, &ec);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return static_cast<int>(ec);
#else
    pid_t pid = fork();
    if (pid == 0) {
        if (!options.cwd.empty() && chdir(options.cwd.c_str()) != 0) _exit(127);
        for (const auto& name : options.unset_env) unsetenv(name.c_str());
        std::vector<char*> argv_exec;
        for (const auto& a : args) argv_exec.push_back(const_cast<char*>(a.c_str()));
        argv_exec.push_back(nullptr);
        execvp(args[0].c_str(), argv_exec.data());
        _exit(127);
    }
    if (pid < 0) return -1;
    int status = 0;
    while (waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR) return -1;
    }
    return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
#endif
}

std::optional<std::filesystem::path> which(const std::string& program) {
    if (program.empty()) return std::nullopt;
    namespace fs = std::filesystem;
    auto is_exe = [](const fs::path& p) {
#ifdef _WIN32
        return fs::is_regular_file(p);
#else
        return fs::is_regular_file(p) && ::access(p.c_str(), X_OK) == 0;
#endif
    };
    std::vector<std::string> names{program};
#ifdef _WIN32
    names.push_back(program + ".exe");
#endif
    fs::path prog(program);
    bool has_dir = prog.has_parent_path() || program.find('/') != std::string::npos ||
                   program.find('\\') != std::string::npos;
    if (has_dir) {
        for (const auto& n : names) {
            fs::path c(n);
            if (is_exe(c)) return c;
        }
        return std::nullopt;
    }
    const char* path_env = std::getenv("PATH");
    if (path_env == nullptr) return std::nullopt;
#ifdef _WIN32
    const char delim = ';';
#else
    const char delim = ':';
#endif
    std::string path_str(path_env);
    size_t pos = 0;
    while (pos <= path_str.size()) {
        size_t next = path_str.find(delim, pos);
        std::string dir = (next == std::string::npos) ? path_str.substr(pos)
                                                    : path_str.substr(pos, next - pos);
        if (dir.empty()) dir = ".";
        for (const auto& n : names) {
            fs::path c = fs::path(dir) / n;
            if (is_exe(c)) return c;
        }
        if (next == std::string::npos) break;
        pos = next + 1;
    }
    return std::nullopt;
}

std::filesystem::path current_executable_dir() {
#ifdef _WIN32
    wchar_t wbuf[32768];
    DWORD n = GetModuleFileNameW(nullptr, wbuf, static_cast<DWORD>(std::size(wbuf)));
    if (n > 0 && n < std::size(wbuf)) return std::filesystem::path(std::wstring(wbuf, n)).parent_path();
#elif defined(__APPLE__)
    uint32_t sz = 0;
    _NSGetExecutablePath(nullptr, &sz);
    if (sz > 0) {
        std::string buf(sz, '\0');
        if (_NSGetExecutablePath(buf.data(), &sz) == 0) {
            std::error_code ec;
            auto p = std::filesystem::weakly_canonical(std::filesystem::u8path(buf.c_str()), ec);
            if (!ec) return p.parent_path();
        }
    }
#else
    std::error_code ec;
    auto p = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (!ec) return p.parent_path();
#endif
    return std::filesystem::current_path();
}

}  // namespace umi_slam
