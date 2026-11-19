#include "main_window.hpp"

#include "slam_common/json.hpp"
#include "slam_common/trajectory_io.hpp"
#include "umi_slam/runtime.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <thread>

namespace slam_app {
namespace {

std::filesystem::path find_first_dir(const std::vector<std::filesystem::path>& cands) {
    for (const auto& p : cands) {
        if (std::filesystem::is_directory(p)) return p;
    }
    return cands.empty() ? std::filesystem::current_path() : cands.front();
}

std::filesystem::path find_first_file(const std::vector<std::filesystem::path>& cands) {
    for (const auto& p : cands) {
        if (std::filesystem::is_regular_file(p)) return p;
    }
    return cands.empty() ? std::filesystem::current_path() : cands.front();
}

std::string status_label_zh(const std::string& raw) {
    if (raw == "queued") return "等待中";
    if (raw == "checking") return "检查环境…";
    if (raw == "running") return "正在处理";
    if (raw == "preparing") return "准备数据";
    if (raw == "prepared") return "数据准备完成";
    if (raw == "completed") return "处理完成";
    if (raw == "failed") return "处理失败";
    if (raw == "cancelled") return "已停止";
    return raw;
}

std::optional<double> progress_from_logs(const std::string& text, const std::string& status) {
    if (status == "completed" || status == "prepared") return 100.0;
    if (status == "failed" || status == "cancelled") return 0.0;
    std::regex slam_re(R"(UMI_PROGRESS\s+(\d+)/(\d+))");
    double last_pct = -1.0;
    for (auto it = std::sregex_iterator(text.begin(), text.end(), slam_re); it != std::sregex_iterator(); ++it) {
        double done = std::stod((*it)[1].str());
        double tot = std::max(1.0, std::stod((*it)[2].str()));
        last_pct = 100.0 * done / tot;
    }
    if (last_pct >= 0.0) return last_pct;
    std::regex prep_re(R"(PREPARE\s+(\d+)/(\d+))");
    for (auto it2 = std::sregex_iterator(text.begin(), text.end(), prep_re); it2 != std::sregex_iterator(); ++it2) {
        double done = std::stod((*it2)[1].str());
        double tot = std::max(1.0, std::stod((*it2)[2].str()));
        last_pct = std::min(99.0, 100.0 * done / tot);
    }
    if (last_pct >= 0.0) return last_pct;
    return std::nullopt;
}

}  // namespace

MainWindow::MainWindow() {
    std::filesystem::path cwd = std::filesystem::current_path();
    std::filesystem::path app_dir = umi_slam::current_executable_dir();

    devices_dir_ = find_first_dir({
        app_dir / "devices",
        cwd / "software/slam_station/devices",
        cwd.parent_path() / "software/slam_station/devices",
    });
    profiles_dir_ = find_first_dir({
        app_dir / "profiles",
        cwd / "software/umi_slam/profiles",
        cwd.parent_path() / "software/umi_slam/profiles",
    });
    device_api_doc_ = find_first_file({
        app_dir / "DEVICE_API.md",
        cwd / "software/slam_station/DEVICE_API.md",
    });

    parent_dir_ = (user_home_dir() / "SLAM_Results").string();
    std::filesystem::path def_yaml = profiles_dir_ / "umi_20260908.yaml";
    std::filesystem::path def_dev = std::filesystem::is_regular_file(devices_dir_ / "umi_20260908.json")
        ? (devices_dir_ / "umi_20260908.json")
        : (devices_dir_ / "generic_euroc.json");
    if (std::filesystem::is_regular_file(def_yaml)) settings_path_ = def_yaml.string();
    if (std::filesystem::is_regular_file(def_dev)) device_path_ = def_dev.string();

    if (std::filesystem::is_directory(devices_dir_)) {
        for (const auto& e : std::filesystem::directory_iterator(devices_dir_)) {
            if (e.is_regular_file() && e.path().extension() == ".json") {
                device_presets_.push_back(e.path().filename().string());
            }
        }
        std::sort(device_presets_.begin(), device_presets_.end());
    }

    auto pick_dir_cb = []() { return pick_directory(); };
    auto open_path_cb = [](const std::filesystem::path& p) { open_in_os_shell(p); };

    preview_pane_ = std::make_unique<preview_3d::PreviewPane>(
        [this]() {
            if (!popout_preview_pane_) {
                popout_preview_pane_ = std::make_unique<preview_3d::PreviewPane>(
                    nullptr,
                    []() { return pick_directory(); },
                    [](const std::filesystem::path& p) { open_in_os_shell(p); });
            }
            if (!preview_pane_->current_path().empty()) popout_preview_pane_->load_path(preview_pane_->current_path(), true);
            else if (!selected_job_dir_.empty()) popout_preview_pane_->load_path(selected_job_dir_, true);
            show_popout_preview_ = true;
        },
        pick_dir_cb,
        open_path_cb);

    calibrator_pane_ = std::make_unique<CalibrationPane>(
        [this](const std::filesystem::path& y, const std::filesystem::path& j) { apply_calibration(y, j); },
        [this]() {
            if (!popout_calibrator_pane_) {
                popout_calibrator_pane_ = std::make_unique<CalibrationPane>(
                    [this](const std::filesystem::path& y, const std::filesystem::path& j) { apply_calibration(y, j); });
            }
            show_popout_calibrator_ = true;
        });

    show_device_template();
    refresh_history();
    check_backend();
}

MainWindow::~MainWindow() {
    if (env_worker_.joinable()) {
        env_worker_.join();
    }
    if (job_ && job_->active()) {
        job_->stop();
        job_->wait();
    }
}

bool MainWindow::request_close() {
    if (job_ && job_->active()) {
        closing_ = true;
        job_->stop();
        status_text_ = "等待本地原生后端停止后关闭…";
        return false;
    }
    return true;
}

bool MainWindow::should_exit() const {
    return closing_ && (!job_ || !job_->active());
}

void MainWindow::show_message(const std::string& title, const std::string& body) {
    modal_title_ = title;
    modal_body_ = body;
    open_msg_modal_ = true;
}

void MainWindow::render() {
    auto now = std::chrono::steady_clock::now();
    if (now - last_poll_ >= std::chrono::milliseconds(400)) {
        poll_job();
        last_poll_ = now;
    }

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGuiWindowFlags wflags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                              ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("SLAM Station Main", nullptr, wflags);

    ImGui::Text("SLAM Station  ·  UMI / EGO  (Pure Native C++17 / Dear ImGui)");
    ImGui::SameLine();
    std::string env_copy;
    {
        std::lock_guard<std::mutex> lk(env_mtx_);
        env_copy = env_status_;
    }
    ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.80f, 1.0f), "  |  %s", env_copy.c_str());
    ImGui::Separator();

    if (ImGui::BeginTabBar("##main_tabs")) {
        auto tab_flags = [&](int idx) -> ImGuiTabItemFlags {
            return (active_tab_request_ == idx) ? ImGuiTabItemFlags_SetSelected : 0;
        };
        if (ImGui::BeginTabItem("处理任务", nullptr, tab_flags(0))) {
            render_task_tab();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("轨迹与建图预览", nullptr, tab_flags(1))) {
            preview_pane_->render("main_preview");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("多模态标定 / 手眼工作台", nullptr, tab_flags(2))) {
            calibrator_pane_->render("main_calibrator");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("设备与扩展")) {
            render_devices_tab();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("历史结果")) {
            render_history_tab();
            ImGui::EndTabItem();
        }
        active_tab_request_ = -1;
        ImGui::EndTabBar();
    }

    render_popups();
    ImGui::End();

    if (show_popout_preview_ && popout_preview_pane_) {
        ImGui::SetNextWindowSize(ImVec2(1080, 720), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("独立 3D / 2D 建图预览", &show_popout_preview_)) {
            popout_preview_pane_->render("popout_preview");
        }
        ImGui::End();
    }

    if (show_popout_calibrator_ && popout_calibrator_pane_) {
        ImGui::SetNextWindowSize(ImVec2(1100, 780), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("独立免 ROS 标定工作台", &show_popout_calibrator_)) {
            popout_calibrator_pane_->render("popout_calibrator");
        }
        ImGui::End();
    }
}

void MainWindow::render_popups() {
    if (open_msg_modal_) {
        ImGui::OpenPopup("提示##main_msg");
        open_msg_modal_ = false;
    }
    if (ImGui::BeginPopupModal("提示##main_msg", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted(modal_title_.c_str());
        ImGui::Separator();
        ImGui::TextWrapped("%s", modal_body_.c_str());
        if (ImGui::Button("确定", ImVec2(100, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

void MainWindow::render_task_tab() {
    bool running = (job_ && job_->active());
    float input_w = std::max(280.0f, ImGui::GetContentRegionAvail().x - 270.0f);

    ImGui::SetNextItemWidth(input_w);
    input_text_std("视频 / IMU 采集目录", source_dir_);
    ImGui::SameLine();
    if (ImGui::Button("浏览…##tsrc")) {
        if (auto d = pick_directory()) {
            source_dir_ = d->string();
            detect_source_defaults(*d);
        }
    }

    ImGui::SetNextItemWidth(input_w);
    input_text_std("建图结果保存目录", parent_dir_);
    ImGui::SameLine();
    if (ImGui::Button("浏览…##tpar")) {
        if (auto d = pick_directory()) {
            parent_dir_ = d->string();
            refresh_history();
        }
    }

    ImGui::SetNextItemWidth(input_w);
    input_text_std("双目 / IMU 标定 YAML", settings_path_);
    ImGui::SameLine();
    if (ImGui::Button("浏览…##tset")) {
        if (auto f = pick_open_file("YAML", "yaml,yml")) settings_path_ = f->string();
    }

    ImGui::SetNextItemWidth(input_w);
    input_text_std("设备配置 JSON", device_path_);
    ImGui::SameLine();
    if (ImGui::Button("浏览…##tdev")) {
        if (auto f = pick_open_file("JSON", "json")) device_path_ = f->string();
    }

    ImGui::SetNextItemWidth(75); ImGui::InputInt("抽帧间隔", &stride_, 0, 0); ImGui::SameLine();
    ImGui::SetNextItemWidth(85); ImGui::InputInt("最多帧数(0=全部)", &max_frames_, 0, 0); ImGui::SameLine();
    ImGui::SetNextItemWidth(85); ImGui::InputDouble("超时秒数(0=自动)", &timeout_s_, 0, 0, "%.1f");

    ImGui::Checkbox("仅检查并准备数据（不运行 SLAM）", &prepare_only_);
    ImGui::SameLine();
    ImGui::Checkbox("实时 3D 建图窗口 (Pangolin)", &viewer_);

    ImGui::BeginDisabled(running);
    if (ImGui::Button("开始转换并建图")) start_job();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("停止任务")) stop_job();
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("检查本地后端")) check_backend();
    ImGui::SameLine();
    ImGui::BeginDisabled(running);
    if (ImGui::Button("填入现有 UMI 示例")) fill_example();
    ImGui::EndDisabled();

    ImGui::TextColored(ImVec4(0.85f, 0.92f, 0.95f, 1.0f), "%s", status_text_.c_str());
    ImGui::ProgressBar(progress_frac_, ImVec2(-1.0f, 18.0f));

    ImVec2 avail = ImGui::GetContentRegionAvail();
    std::string log_copy = last_log_text_;
    input_text_multiline_std("##task_log", log_copy, ImVec2(avail.x, std::max(140.0f, avail.y - 34.0f)),
                             ImGuiInputTextFlags_ReadOnly);

    if (ImGui::Button("复制日志") && !last_log_text_.empty()) {
        ImGui::SetClipboardText(last_log_text_.c_str());
        status_text_ = "已复制全部日志到剪贴板。";
    }
    ImGui::SameLine();
    if (ImGui::Button("预览当前建图结果")) open_current_preview();
    ImGui::SameLine();
    if (ImGui::Button("加载建图数据目录预览…")) load_preview_directory();
    ImGui::SameLine();
    if (ImGui::Button("查看轨迹报告")) open_selected_report();
    ImGui::SameLine();
    if (ImGui::Button("打开任务目录")) open_selected_folder();
}

void MainWindow::render_devices_tab() {
    ImGui::TextWrapped("设备接口 v1：现有 UMI 原始协议可直接处理；新 UMI / EGO 先转换为同步双目 + IMU（EuRoC）。");
    if (!device_presets_.empty()) {
        ImGui::SetNextItemWidth(320);
        if (ImGui::BeginCombo("设备模板", device_presets_[preset_idx_].c_str())) {
            for (int i = 0; i < static_cast<int>(device_presets_.size()); ++i) {
                bool sel = (i == preset_idx_);
                if (ImGui::Selectable(device_presets_[i].c_str(), sel)) {
                    preset_idx_ = i;
                    show_device_template();
                }
                if (sel) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    }
    ImVec2 avail = ImGui::GetContentRegionAvail();
    std::string copy = device_json_text_;
    input_text_multiline_std("##dev_json", copy, ImVec2(avail.x, std::max(140.0f, avail.y - 36.0f)),
                             ImGuiInputTextFlags_ReadOnly);
    if (ImGui::Button("使用此设备模板")) use_device_template();
    ImGui::SameLine();
    if (ImGui::Button("另存模板…")) save_device_template();
    ImGui::SameLine();
    if (ImGui::Button("查看接口说明")) open_device_api_doc();
}

void MainWindow::render_history_tab() {
    if (ImGui::Button("刷新列表")) refresh_history();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    if (ImGui::BeginTable("##history_tbl", 3,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable,
                          ImVec2(avail.x, std::max(160.0f, avail.y - 36.0f)))) {
        ImGui::TableSetupColumn("任务");
        ImGui::TableSetupColumn("状态", ImGuiTableColumnFlags_WidthFixed, 130.0f);
        ImGui::TableSetupColumn("设备", ImGuiTableColumnFlags_WidthFixed, 220.0f);
        ImGui::TableHeadersRow();
        for (int i = 0; i < static_cast<int>(history_.size()); ++i) {
            const auto& h = history_[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            bool sel = (selected_history_idx_ == i);
            if (ImGui::Selectable(h.name.c_str(), sel, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick)) {
                selected_history_idx_ = i;
                selected_job_dir_ = h.dir;
                if (!job_ || !job_->active()) {
                    std::string logs = read_job_logs(selected_job_dir_);
                    if (!logs.empty()) last_log_text_ = logs;
                }
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    open_current_preview();
                }
            }
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(h.status.c_str());
            ImGui::TableSetColumnIndex(2);
            ImGui::TextUnformatted(h.device.c_str());
        }
        ImGui::EndTable();
    }
    if (ImGui::Button("预览选中任务")) open_current_preview();
    ImGui::SameLine();
    if (ImGui::Button("打开选中任务")) open_selected_folder();
    ImGui::SameLine();
    if (ImGui::Button("查看选中报告")) open_selected_report();
}

void MainWindow::detect_source_defaults(const std::filesystem::path& src) {
    if (std::filesystem::is_directory(src / "mav0") ||
        (std::filesystem::is_directory(src / "cam0") && std::filesystem::is_directory(src / "imu0"))) {
        std::filesystem::path dev = devices_dir_ / "generic_euroc.json";
        if (std::filesystem::is_regular_file(dev)) device_path_ = dev.string();
        status_text_ = "已识别 EuRoC 格式目录；请确认对应的双目 / IMU 标定 YAML。";
    } else if (std::filesystem::is_regular_file(src / "stereo.mkv")) {
        std::filesystem::path dev = devices_dir_ / "umi_20260908.json";
        std::filesystem::path yml = profiles_dir_ / "umi_20260908.yaml";
        if (std::filesystem::is_regular_file(dev)) device_path_ = dev.string();
        if (settings_path_.empty() && std::filesystem::is_regular_file(yml)) settings_path_ = yml.string();
        status_text_ = "已识别 UMI 视频 + IMU 目录；可点击“开始转换并建图”。";
    }
}

void MainWindow::fill_example() {
    if (job_ && job_->active()) return;
    source_dir_ = (std::filesystem::current_path() / "examples/lissajous_20260915/input").string();
    settings_path_ = (profiles_dir_ / "umi_20260908.yaml").string();
    device_path_ = (devices_dir_ / "umi_20260908.json").string();
    stride_ = 5;
    status_text_ = "已填入历史 UMI 示例；此标定仅适用于这套设备。";
}

void MainWindow::check_backend() {
    if (env_checking_.exchange(true)) return;
    if (env_worker_.joinable()) env_worker_.join();
    {
        std::lock_guard<std::mutex> lk(env_mtx_);
        env_status_ = "正在检查本地原生后端…";
    }
    env_worker_ = std::thread([this]() {
        try {
            auto info = umi_slam::diagnose();
            std::string msg = "本地原生后端已就绪 · " +
                              std::filesystem::u8path(info.get("binary").as_string()).filename().string();
            std::lock_guard<std::mutex> lk(env_mtx_);
            env_status_ = msg;
        } catch (const std::exception& exc) {
            std::lock_guard<std::mutex> lk(env_mtx_);
            env_status_ = std::string("后端未就绪：") + exc.what();
        }
        env_checking_.store(false);
    });
}

void MainWindow::start_job() {
    if (job_ && job_->active()) return;
    try {
        if (source_dir_.empty() || parent_dir_.empty() || settings_path_.empty() || device_path_.empty()) {
            throw std::invalid_argument("请选择采集目录、结果目录、标定和设备配置");
        }
        job_ = std::make_unique<umi_slam::Job>(
            std::filesystem::u8path(source_dir_),
            std::filesystem::u8path(parent_dir_),
            std::filesystem::u8path(settings_path_),
            std::filesystem::u8path(device_path_),
            stride_,
            max_frames_,
            timeout_s_,
            prepare_only_,
            std::nullopt,
            viewer_);
        selected_job_dir_ = job_->folder();
        auto_loaded_job_.clear();
        last_log_text_.clear();
        progress_frac_ = 0.0f;
        job_->start();
        refresh_history();
    } catch (const std::exception& exc) {
        show_message("无法开始", exc.what());
    }
}

void MainWindow::stop_job() {
    if (job_ && job_->active()) {
        job_->stop();
        status_text_ = "正在停止任务并保存状态…";
    }
}

std::string MainWindow::read_job_logs(const std::filesystem::path& job_folder) const {
    if (job_folder.empty()) return "";
    std::string combined;
    for (const auto& p : {job_folder / "host.log", job_folder / "result" / "slam.log"}) {
        if (!std::filesystem::is_regular_file(p)) continue;
        std::ifstream f(p, std::ios::binary | std::ios::ate);
        if (!f) continue;
        std::streamoff sz = f.tellg();
        std::streamoff start = std::max<std::streamoff>(0, sz - 16000);
        f.seekg(start);
        std::string buf(static_cast<size_t>(sz - start), '\0');
        f.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        if (!combined.empty()) combined.push_back('\n');
        combined += buf;
    }
    return combined;
}

void MainWindow::poll_job() {
    if (job_ && (job_->active() || auto_loaded_job_ != job_->folder())) {
        auto state = slam_common::read_json_safe(job_->folder() / "station_job.json");
        auto backend = slam_common::read_json_safe(job_->result_dir() / "job.json");
        std::string raw_status = state.get("status").is_string() ? state.get("status").as_string() : "";
        std::string label = status_label_zh(raw_status);
        if (job_->active() && job_->cancel_flag().load()) {
            label = "正在停止任务并保存状态…";
        } else if (job_->active() && raw_status == "running" && backend.get("status").is_string()) {
            label = status_label_zh(backend.get("status").as_string());
        }
        if (state.contains("coverage_fraction") && state.get("coverage_fraction").is_number()) {
            std::ostringstream os;
            os << std::fixed << std::setprecision(1) << (state.get("coverage_fraction").as_double() * 100.0);
            label += " · 轨迹覆盖率 " + os.str() + "%";
        }
        if (state.contains("error") && state.get("error").is_string()) {
            label += " · " + state.get("error").as_string();
        }
        std::string logs = read_job_logs(job_->folder());
        if (auto pct = progress_from_logs(logs, raw_status)) {
            progress_frac_ = static_cast<float>(std::clamp(*pct / 100.0, 0.0, 1.0));
        }
        if (logs != last_log_text_) last_log_text_ = logs;
        status_text_ = label;

        if (!job_->active() && auto_loaded_job_ != job_->folder()) {
            auto_loaded_job_ = job_->folder();
            refresh_history();
            if (raw_status == "completed" && preview_pane_->load_path(job_->folder(), true)) {
                status_text_ = label + " · 已加载 3D / 2D 建图预览";
            }
        }
    }
}

void MainWindow::refresh_history() {
    history_.clear();
    selected_history_idx_ = -1;
    std::filesystem::path parent = std::filesystem::u8path(parent_dir_);
    if (!std::filesystem::is_directory(parent)) return;
    std::vector<std::filesystem::path> runs;
    for (const auto& entry : std::filesystem::directory_iterator(parent)) {
        if (entry.is_directory() && std::filesystem::is_regular_file(entry.path() / "station_job.json")) {
            runs.push_back(entry.path());
        }
    }
    std::sort(runs.rbegin(), runs.rend());
    if (runs.size() > 200) runs.resize(200);
    for (const auto& r : runs) {
        auto d = slam_common::read_json_safe(r / "station_job.json");
        std::string st = d.get("status").is_string() ? status_label_zh(d.get("status").as_string()) : "未知";
        std::string dev_name = d.get("device").get("name").is_string() ? d.get("device").get("name").as_string() : "";
        history_.push_back({r, r.filename().string(), st, dev_name});
    }
}

void MainWindow::open_current_preview() {
    std::filesystem::path target = !selected_job_dir_.empty() ? selected_job_dir_ : preview_pane_->current_path();
    if (target.empty()) {
        load_preview_directory();
        return;
    }
    if (preview_pane_->load_path(target, false)) active_tab_request_ = 1;
}

void MainWindow::load_preview_directory() {
    if (auto chosen = pick_directory()) {
        if (preview_pane_->load_path(*chosen, false)) active_tab_request_ = 1;
    }
}

void MainWindow::apply_calibration(const std::filesystem::path& yaml_path, const std::filesystem::path& json_path) {
    if (job_ && job_->active()) {
        show_message("提示", "当前有 SLAM 任务正在运行，请等待结束后再替换配置。");
        return;
    }
    try {
        auto dev = slam_common::load_device(json_path);
        slam_common::validate_slam_sensor_mode(dev);
    } catch (const std::exception& exc) {
        show_message("无法应用到 SLAM 建图任务", exc.what());
        return;
    }
    settings_path_ = yaml_path.string();
    device_path_ = json_path.string();
    status_text_ = "已应用新生成的标定 YAML 与设备 JSON；请选择采集目录并开始建图。";
    active_tab_request_ = 0;
}

void MainWindow::show_device_template() {
    if (device_presets_.empty()) return;
    std::filesystem::path p = devices_dir_ / std::filesystem::u8path(device_presets_[preset_idx_]);
    if (std::filesystem::is_regular_file(p)) {
        device_json_text_ = slam_common::read_text_utf8(p);
    }
}

void MainWindow::use_device_template() {
    if ((job_ && job_->active()) || device_presets_.empty()) return;
    device_path_ = (devices_dir_ / std::filesystem::u8path(device_presets_[preset_idx_])).string();
    settings_path_.clear();
    status_text_ = "已选择设备模板，请选择该设备对应的标定 YAML。";
    active_tab_request_ = 0;
}

void MainWindow::save_device_template() {
    if (device_presets_.empty()) return;
    if (auto out = pick_save_file("JSON", "json", device_presets_[preset_idx_])) {
        std::filesystem::copy_file(
            devices_dir_ / std::filesystem::u8path(device_presets_[preset_idx_]),
            *out,
            std::filesystem::copy_options::overwrite_existing);
    }
}

void MainWindow::open_device_api_doc() {
    if (std::filesystem::is_regular_file(device_api_doc_)) open_in_os_shell(device_api_doc_);
}

void MainWindow::open_selected_folder() {
    std::filesystem::path target = !selected_job_dir_.empty() ? selected_job_dir_ : preview_pane_->current_path();
    if (!target.empty() && std::filesystem::is_directory(target)) {
        open_in_os_shell(target);
    } else {
        show_message("结果", "请先运行任务、选择历史任务或加载预览目录。");
    }
}

void MainWindow::open_selected_report() {
    std::filesystem::path target = !selected_job_dir_.empty() ? selected_job_dir_ : preview_pane_->current_path();
    for (const auto& cand : {target / "result" / "report.html", target / "report.html"}) {
        if (std::filesystem::is_regular_file(cand)) {
            open_in_os_shell(cand);
            return;
        }
    }
    show_message("报告", "选中的目录还没有轨迹报告。");
}

}  // namespace slam_app
