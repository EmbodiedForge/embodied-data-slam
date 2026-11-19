#pragma once

#include "slam_common/json.hpp"

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace camera_imu_calibration {

constexpr double kDefaultTimeshiftSearchS = 0.05;
constexpr int kTimeshiftGridSteps = 201;
constexpr double kMinTimeshiftCorr = 0.5;
constexpr double kMinMovingGyroRadS = 0.08;
constexpr int kMinMovingGyroPairs = 4;
constexpr int kMinTranslationRows = 6;
constexpr double kMaxPlausibleLeverArmM = 0.5;

/// Parse standard Kalibr YAML text into a JsonValue tree without requiring external YAML libs.
slam_common::JsonValue parse_simple_yaml(const std::string& text);

struct Observation {
    int64_t ts_ns = 0;
    /// Per-camera (cam_idx -> {obj_pts_3d, img_pts_2d})
    std::map<int, std::vector<cv::Point3f>> obj_pts;
    std::map<int, std::vector<cv::Point2f>> img_pts;
};

struct MonoCalibrationResult {
    int cam_idx = 0;
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    Eigen::Vector4d D = Eigen::Vector4d::Zero();  // k1, k2, p1, p2
    double mono_rms = 0.0;
    int frames = 0;
    std::vector<size_t> kept_indices;
    cv::Size image_size{0, 0};
};

struct StereoCalibrationResult {
    Eigen::Matrix3d K0 = Eigen::Matrix3d::Identity();
    Eigen::Vector4d D0 = Eigen::Vector4d::Zero();
    Eigen::Matrix3d K1 = Eigen::Matrix3d::Identity();
    Eigen::Vector4d D1 = Eigen::Vector4d::Zero();
    Eigen::Matrix4d T_c1_c2 = Eigen::Matrix4d::Identity();
    double baseline = 0.0;
    std::map<int, double> mono_rms;
    double stereo_rms = 0.0;
    int stereo_frames = 0;
};

struct MultiCamEntry {
    int cam_idx = 0;
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    Eigen::Vector4d D = Eigen::Vector4d::Zero();
    cv::Size image_size{0, 0};
    double mono_rms = 0.0;
    int frames = 0;
    Eigen::Matrix4d T_c0_ci = Eigen::Matrix4d::Identity();
    double baseline_to_c0_m = 0.0;
};

struct MultiCamCalibrationResult {
    int num_cams = 0;
    std::vector<MultiCamEntry> cameras;
    slam_common::JsonValue pair_stats = slam_common::JsonValue::object();
};

struct ImuStandaloneStats {
    double measured_hz = 0.0;
    double duration_s = 0.0;
    int sample_count = 0;
    int static_samples = 0;
    Eigen::Vector3d gyro_bias_rad_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_mean_mps2 = Eigen::Vector3d::Zero();
    double gravity_norm_mps2 = 0.0;
    double noise_gyro = 0.0035;
    double noise_acc = 0.02;
    double gyro_walk = 0.0002;
    double acc_walk = 0.002;
};

struct CamImuExtrinsicResult {
    Eigen::Matrix4d T_b_c1 = Eigen::Matrix4d::Identity();
    int64_t timeshift_ns = 0;
    double gyro_rmse_rad_s = 0.0;
    double timeshift_corr = 0.0;
    bool timeshift_at_bound = false;
    bool timeshift_low_corr = false;
    bool translation_valid = false;
    int translation_rows = 0;
    int imu_pairs = 0;
    std::vector<std::string> warnings;
};

/// Calibrate single pinhole-radtan camera intrinsics from detected frames.
MonoCalibrationResult calibrate_monocular_from_observations(
    const std::vector<Observation>& obs_list,
    cv::Size image_size,
    int cam_idx = 0);

/// Calibrate monocular intrinsics and stereo T_c1_c2 from detected frames.
StereoCalibrationResult calibrate_stereo_from_observations(
    const std::vector<Observation>& obs_list,
    cv::Size image_size);

/// Calibrate intrinsics and relative rig extrinsics T_c0_ci for N >= 2 cameras via covisibility BFS.
MultiCamCalibrationResult calibrate_multicam_from_observations(
    const std::vector<Observation>& obs_list,
    const std::vector<cv::Size>& image_sizes);

/// Estimate IMU sample rate, biases, white noise density, and bias random walk.
ImuStandaloneStats calibrate_imu_standalone(
    const std::vector<int64_t>& imu_times_ns,
    const std::vector<Eigen::Matrix<double, 6, 1>>& imu_gyro_acc,
    double imu_hz = 0.0);

/// Estimate IMU.T_b_c1 and camera_to_imu_offset_ns without ROS.
CamImuExtrinsicResult estimate_cam_imu_extrinsic(
    const std::vector<Observation>& obs_list,
    const Eigen::Matrix3d& K0,
    const Eigen::Vector4d& D0,
    const std::vector<int64_t>& imu_times_ns,
    const std::vector<Eigen::Matrix<double, 6, 1>>& imu_gyro_acc,
    int cam_idx = 0,
    double max_timeshift_s = kDefaultTimeshiftSearchS);

struct WriteOrbYamlParams {
    std::filesystem::path output_dir;
    Eigen::Matrix3d cam0_K = Eigen::Matrix3d::Identity();
    Eigen::Vector4d cam0_D = Eigen::Vector4d::Zero();
    bool has_cam1 = false;
    Eigen::Matrix3d cam1_K = Eigen::Matrix3d::Identity();
    Eigen::Vector4d cam1_D = Eigen::Vector4d::Zero();
    int width = 0;
    int height = 0;
    bool has_T_c1_c2 = false;
    Eigen::Matrix4d T_c1_c2 = Eigen::Matrix4d::Identity();
    bool has_T_b_c1 = false;
    Eigen::Matrix4d T_b_c1 = Eigen::Matrix4d::Identity();
    double fps = 125.0;
    double imu_hz = 250.0;
    double noise_gyro = 0.0035;
    double noise_acc = 0.02;
    double gyro_walk = 0.0002;
    double acc_walk = 0.002;
    int fast_init = 0;
    int64_t timeshift_ns = 0;
    std::string adapter = "euroc";
    double accel_range_g = 4.0;
    double gyro_range_dps = 2000.0;
    std::string calibration_source = "SLAMCalibrator";
    std::string profile_id = "custom-calibrated";
    std::string profile_name = "标定生成设备配置";
    std::string sensor_mode = "stereo_imu";
    std::vector<MultiCamEntry> extra_cameras;
    bool has_imu_stats = false;
    ImuStandaloneStats imu_stats;
    std::string imu_extrinsic_status;
    std::string imu_extrinsic_warning;
};

/// Write calibration YAML (mono/stereo/multi/imu) and matching device profile JSON.
std::pair<std::filesystem::path, std::filesystem::path> write_orbslam_calibration(
    const WriteOrbYamlParams& params);

/// Convert Kalibr camchain + optional IMU YAML into ORB-SLAM3 YAML and device JSON without ROS.
std::pair<std::filesystem::path, std::filesystem::path> convert_kalibr_files(
    const std::filesystem::path& camchain_yaml,
    const std::filesystem::path& imu_yaml,
    const std::filesystem::path& output_dir,
    double fps = 125.0,
    const std::string& adapter = "umi_ms901m_uniform",
    double accel_range_g = 4.0,
    double gyro_range_dps = 2000.0);

struct DetectedCorners {
    bool found = false;
    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;
};

using DetectCallback = std::function<DetectedCorners(const cv::Mat& gray, int cam_idx)>;
using ProgressCallback = std::function<void(const std::string& msg)>;
using PreviewCallback = std::function<void(
    const cv::Mat& left_gray,
    const cv::Mat& right_gray,
    const std::vector<cv::Point2f>& img0,
    const std::vector<cv::Point2f>& img1,
    int valid_count)>;

struct SuiteCalibrationParams {
    std::filesystem::path source_dir;
    std::filesystem::path output_dir;
    std::string camera_mode = "stereo";  // none / mono / stereo / multi
    bool enable_imu = true;
    std::string target_type = "aprilgrid";
    int rows = 6;
    int cols = 6;
    double tag_size_m = 0.0345;
    double tag_spacing = 0.3;
    int stride = 5;
    int max_frames = 120;
    double fps = 0.0;  // <= 0 means auto
    double imu_hz = 250.0;
    double accel_range_g = 4.0;
    double gyro_range_dps = 2000.0;
    double noise_gyro = 0.0035;
    double noise_acc = 0.02;
    double gyro_walk = 0.0002;
    double acc_walk = 0.002;
    std::vector<std::filesystem::path> camera_dirs;
    DetectCallback detect_callback;
    ProgressCallback progress_cb;
    PreviewCallback preview_cb;
    const std::atomic<bool>* cancel_flag = nullptr;
};

struct SuiteCalibrationOutput {
    std::filesystem::path yaml_path;
    std::filesystem::path json_path;
    std::filesystem::path report_path;
    slam_common::JsonValue summary = slam_common::JsonValue::object();
};

/// Unified ROS-free calibration supporting None/Mono/Stereo/Multi-Cam x optional IMU.
SuiteCalibrationOutput calibrate_sensor_suite(const SuiteCalibrationParams& params);

}  // namespace camera_imu_calibration
