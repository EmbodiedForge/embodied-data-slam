#pragma once

#include "calibration_workbench.hpp"
#include "platform_ui.hpp"
#include "preview_3d/preview_widget.hpp"
#include "umi_slam/runtime.hpp"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace slam_app {

/// 5-Tab Dear ImGui SLAM Station Main Window (Task, 3D Preview, Calibration Workbench, Device Templates, History).
class MainWindow {
public:
    MainWindow();
    ~MainWindow();

    /// Render the main window (fills viewport) plus optional floating preview/calibrator windows.
    void render();

    /// Called when the OS window close button is pressed; returns true if ready to close immediately.
    bool request_close();

    /// Returns true once graceful shutdown has finished and the app should exit.
    bool should_exit() const;

private:
    void render_task_tab();
    void render_devices_tab();
    void render_history_tab();
    void render_popups();

    void fill_example();
    void check_backend();
    void start_job();
    void stop_job();
    void poll_job();
    void refresh_history();
    void open_current_preview();
    void load_preview_directory();
    void apply_calibration(const std::filesystem::path& yaml_path, const std::filesystem::path& json_path);
    void detect_source_defaults(const std::filesystem::path& src);
    void show_device_template();
    void use_device_template();
    void save_device_template();
    void open_device_api_doc();
    void open_selected_folder();
    void open_selected_report();
    std::string read_job_logs(const std::filesystem::path& job_folder) const;
    void show_message(const std::string& title, const std::string& body);

    std::unique_ptr<umi_slam::Job> job_;
    std::filesystem::path selected_job_dir_;
    std::filesystem::path auto_loaded_job_;
    std::string last_log_text_;
    bool closing_ = false;
    std::chrono::steady_clock::time_point last_poll_{};

    std::filesystem::path devices_dir_;
    std::filesystem::path profiles_dir_;
    std::filesystem::path device_api_doc_;

    std::mutex env_mtx_;
    std::string env_status_ = "本地原生后端尚未检查";
    std::thread env_worker_;
    std::atomic<bool> env_checking_{false};

    int active_tab_request_ = -1;  // 0: task, 1: preview, 2: calibrator

    // Task form state
    std::string source_dir_;
    std::string parent_dir_;
    std::string settings_path_;
    std::string device_path_;
    int stride_ = 5;
    int max_frames_ = 0;
    double timeout_s_ = 0.0;
    bool prepare_only_ = false;
    bool viewer_ = false;
    std::string status_text_ = "请选择视频/IMU 采集目录、标定和设备配置。";
    float progress_frac_ = 0.0f;

    // Devices tab
    std::vector<std::string> device_presets_;
    int preset_idx_ = 0;
    std::string device_json_text_;

    // History tab
    struct HistoryEntry {
        std::filesystem::path dir;
        std::string name;
        std::string status;
        std::string device;
    };
    std::vector<HistoryEntry> history_;
    int selected_history_idx_ = -1;

    // Sub-panes & optional floating windows
    std::unique_ptr<preview_3d::PreviewPane> preview_pane_;
    std::unique_ptr<CalibrationPane> calibrator_pane_;
    bool show_popout_preview_ = false;
    std::unique_ptr<preview_3d::PreviewPane> popout_preview_pane_;
    bool show_popout_calibrator_ = false;
    std::unique_ptr<CalibrationPane> popout_calibrator_pane_;

    // Modal alert
    std::string modal_title_;
    std::string modal_body_;
    bool open_msg_modal_ = false;
};

}  // namespace slam_app
