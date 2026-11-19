#pragma once

#include "slam_common/json.hpp"

#include <atomic>
#include <filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace umi_slam {

constexpr double kCancelPollIntervalS = 0.2;
constexpr double kWindowsCtrlBreakGraceS = 0.6;
constexpr double kWindowsTaskkillTimeoutS = 1.5;
constexpr double kPosixTermTimeoutS = 10.0;
constexpr double kKillWaitTimeoutS = 5.0;

struct NativeBackendInfo {
    std::filesystem::path backend_json;
    std::filesystem::path binary;
    std::filesystem::path vocabulary;
    std::vector<std::filesystem::path> library_paths;
    std::filesystem::path pipeline_exe;
};

struct RuntimeHooks {
    /// Optional callback invoked before taskkill/termination in mock tests.
    std::function<void(const std::vector<std::string>&)> on_taskkill;
};

/// Absolute directory containing the running executable (falls back to the current directory).
std::filesystem::path current_executable_dir();

/// Locate native backend.json, runner binary, ORBvoc.txt, and pipeline executable.
std::optional<NativeBackendInfo> find_native_backend(
    const std::optional<std::filesystem::path>& backend_path = std::nullopt);

/// Build native pipeline CLI command after validating input/output directories and parameters.
std::vector<std::string> native_command(
    const std::filesystem::path& source,
    const std::filesystem::path& output,
    const std::filesystem::path& settings,
    const std::filesystem::path& profile,
    int stride = 5,
    int max_frames = 0,
    double timeout = 0.0,
    bool prepare_only = false,
    const std::optional<std::filesystem::path>& backend_path = std::nullopt,
    bool viewer = false);

/// Alias for native_command (pure native runtime).
std::vector<std::string> command(
    const std::filesystem::path& source,
    const std::filesystem::path& output,
    const std::filesystem::path& settings,
    const std::filesystem::path& profile,
    int stride = 5,
    int max_frames = 0,
    double timeout = 0.0,
    bool prepare_only = false,
    const std::optional<std::filesystem::path>& backend_path = std::nullopt,
    bool viewer = false);

/// Check native backend readiness.
slam_common::JsonValue diagnose(
    const std::optional<std::filesystem::path>& backend_path = std::nullopt);

/// Execute native process tree cleanly, terminating the entire process tree on cancel.
int execute_command(
    const std::vector<std::string>& cmd,
    std::atomic<bool>& cancel,
    const std::filesystem::path& log_path,
    const RuntimeHooks* hooks = nullptr);

struct JobHooks {
    std::function<slam_common::JsonValue(const std::optional<std::filesystem::path>& backend)> diagnose_fn;
    std::function<int(const std::vector<std::string>& cmd, std::atomic<bool>& cancel, const std::filesystem::path& log_path)> execute_fn;
};

class Job {
public:
    Job(const std::filesystem::path& source,
        const std::filesystem::path& parent,
        const std::filesystem::path& settings,
        const std::filesystem::path& device,
        int stride = 5,
        int max_frames = 0,
        double timeout = 0.0,
        bool prepare_only = false,
        const std::optional<std::filesystem::path>& backend_path = std::nullopt,
        bool viewer = false,
        JobHooks hooks = {});
    ~Job();

    void start();
    void run();
    void stop();
    void wait();
    bool active() const;

    const std::filesystem::path& folder() const { return folder_; }
    const std::filesystem::path& result_dir() const { return result_; }
    slam_common::JsonValue state() const;
    std::atomic<bool>& cancel_flag() { return cancel_; }

private:
    void save_state_locked();
    void reconcile_cancelled_backend_state();

    std::filesystem::path folder_;
    std::filesystem::path result_;
    std::optional<std::filesystem::path> backend_path_;
    std::vector<std::string> cmd_;
    slam_common::JsonValue state_;
    mutable std::mutex state_mtx_;
    std::atomic<bool> cancel_{false};
    std::atomic<bool> running_{false};
    std::thread thread_;
    JobHooks hooks_;
};

}  // namespace umi_slam
