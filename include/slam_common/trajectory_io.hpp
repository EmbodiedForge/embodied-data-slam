#pragma once

#include "slam_common/json.hpp"

#include <Eigen/Core>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace slam_common {

constexpr size_t kDefaultMaxMapPoints = 15000;

struct TrajectoryPoint {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    std::string segment_id = "0";
};

struct PreviewData {
    std::string dir;
    std::string job_dir;
    std::string traj_source;
    std::vector<TrajectoryPoint> points;
    std::vector<TrajectoryPoint> target_points;
    std::string target_frame;
    std::vector<Eigen::Vector3d> keyframes;
    std::vector<Eigen::Vector3d> map_points;
    size_t total_map_points = 0;
    JsonValue quality = JsonValue::object();
    JsonValue station_job = JsonValue::object();
};

/// Validate a 4x4 homogeneous JSON transform matrix and convert to Eigen::Matrix4d.
Eigen::Matrix4d validate_transform(const JsonValue& matrix);

/// Load and validate a version-1 device JSON contract.
JsonValue load_device(const std::filesystem::path& path);

/// Reject sensor modes other than stereo_imu before submitting to the SLAM backend.
void validate_slam_sensor_mode(const JsonValue& device);

/// Extract pipeline-relevant device profile fields.
JsonValue pipeline_profile(const JsonValue& device);

/// Validate dataset layout against device adapter and auxiliary stream contracts.
void check_dataset(const std::filesystem::path& source, const JsonValue& device);

/// Export rigid target-frame trajectory (T_world_target = T_world_imu @ T_imu_target) without altering IMU trajectory.
std::optional<std::filesystem::path> export_target(
    const std::filesystem::path& result_dir,
    const JsonValue& device);

/// Read x_m, y_m, z_m, segment_id points from an imu_trajectory.csv or target_trajectory.csv file.
std::vector<TrajectoryPoint> read_csv_points(const std::filesystem::path& path);

/// Read XYZ coordinates from a TUM/EuRoC whitespace-delimited trajectory file.
std::vector<TrajectoryPoint> read_tum_points(const std::filesystem::path& path);

/// Read and uniformly decimate sparse 3D map points from map_points.csv.
std::pair<std::vector<Eigen::Vector3d>, size_t> read_map_points(
    const std::filesystem::path& path,
    size_t max_points = kDefaultMaxMapPoints);

/// Load trajectory, keyframes, sparse map points, and metadata from a job or result directory.
PreviewData load_preview_data(
    const std::filesystem::path& path,
    size_t max_map_points = kDefaultMaxMapPoints);

}  // namespace slam_common
