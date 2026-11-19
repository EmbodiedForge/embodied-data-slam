#pragma once

#include "slam_common/json.hpp"

#include <opencv2/core.hpp>

#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace umi_slam {

struct SettingsInfo {
    cv::Size shape{0, 0};  // width, height
    double fps = 0.0;
    double imu_hz = 0.0;
    bool fast_init = false;
};

struct BackendConfig {
    std::string binary;
    std::string vocabulary;
    std::vector<std::string> library_paths;
    std::string source_sha256;
};

struct PipelineArgs {
    std::filesystem::path input;
    std::filesystem::path output;
    std::filesystem::path settings;
    std::filesystem::path profile;
    std::filesystem::path backend;
    int stride = 5;
    int max_frames = 0;
    double timeout = 0.0;
    bool prepare_only = false;
    bool prepared_input = false;
    bool viewer = false;
};

/// Read and validate ORB-SLAM3 settings YAML.
SettingsInfo settings_info(const std::filesystem::path& path);

/// Prepare synchronized images, shifted IMU CSV, updated settings.yaml, and input_report.json.
slam_common::JsonValue prepare(
    const std::filesystem::path& source,
    const std::filesystem::path& dest,
    const slam_common::JsonValue& profile,
    const std::filesystem::path& settings,
    int stride,
    int limit);

/// Match exported EuRoC trajectory to exact input timestamps, filter initialized states, and write CSV/TUM/HTML/quality.json.
slam_common::JsonValue export_result(
    const std::filesystem::path& out,
    const slam_common::JsonValue& report);

/// Load backend.json resolving relative paths against backend.json's parent directory.
BackendConfig load_backend_config(const std::filesystem::path& backend_path);

/// Build environment map prepending library_paths when present.
std::map<std::string, std::string> backend_env(
    const BackendConfig& backend,
    const std::map<std::string, std::string>* base_env = nullptr);

/// Apply ORB-SLAM3 offline export patches in an isolated staged copy; rejects mismatched or already-patched trees.
void patch_orb(const std::filesystem::path& orb_root);

/// Execute the full prepare + native runner + export pipeline; returns 0 on success, 130 on cancel, 1 on error.
int run_pipeline_main(const PipelineArgs& args);

/// Parse CLI flags and run the pipeline entrypoint.
int run_pipeline_cli(int argc, char** argv);

}  // namespace umi_slam
