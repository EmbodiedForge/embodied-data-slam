#pragma once

#include "slam_common/json.hpp"

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace vision_calibration {

struct PlanarCalib2D {
    std::string tool = "planar_board_2d";
    std::string dof = "homography";
    std::string weight_func = "least_squares";
    bool right_handed = true;
    Eigen::Vector2d origin_mm = Eigen::Vector2d::Zero();
    double angle_deg = 0.0;
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d inv_matrix = Eigen::Matrix3d::Identity();
    double rms_mm = 0.0;
    double rms_px = 0.0;
    double pixel_size_mm = 0.0;
    double scale_x_mm_px = 0.0;
    double scale_y_mm_px = 0.0;
    int inlier_count = 0;
    int total_points = 0;

    slam_common::JsonValue to_json() const;
    static PlanarCalib2D from_json(const slam_common::JsonValue& j);
};

struct DistortionModel {
    std::string tool = "image_distortion";
    std::string distortion_type = "radial_perspective";
    std::array<int, 2> image_size{0, 0};
    Eigen::Vector2d center_point = Eigen::Vector2d::Zero();
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    std::array<double, 5> D{0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Matrix3d H_persp = Eigen::Matrix3d::Identity();
    double rms_px = 0.0;

    slam_common::JsonValue to_json() const;
    static DistortionModel from_json(const slam_common::JsonValue& j);
};

struct DualCamMapping2D {
    std::string tool = "dual_camera_mapping_2d";
    std::string dof = "affine";
    std::string weight_func = "least_squares";
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d inv_matrix = Eigen::Matrix3d::Identity();
    double scale_x = 1.0;
    double scale_y = 1.0;
    double rotation_deg = 0.0;
    double rms_px = 0.0;
    double max_err_px = 0.0;
    int inlier_count = 0;
    int total_points = 0;

    slam_common::JsonValue to_json() const;
};

/// Calibrate 2D image-to-board physical coordinate mapping and pixel precision (mm/px).
PlanarCalib2D calibrate_planar_board_2d(
    const std::vector<Eigen::Vector2d>& img_pts,
    const std::vector<Eigen::Vector2d>& obj_pts_mm,
    double origin_x = 0.0,
    double origin_y = 0.0,
    double angle_deg = 0.0,
    bool right_handed = true,
    const std::string& dof = "homography",
    const std::string& weight_func = "least_squares",
    double weight_param = 1.345,
    double ransac_thresh = 1.0);

/// Convert pixel coordinate (u, v) to calibrated physical coordinate (X, Y) mm.
Eigen::Vector2d convert_pixel_to_world_2d(const PlanarCalib2D& cal, double u, double v);
Eigen::Vector2d convert_pixel_to_world_2d(const std::filesystem::path& iwcal_path, double u, double v);
Eigen::Vector2d convert_pixel_to_world_2d(const slam_common::JsonValue& cal_json, double u, double v);

/// Convert pixel distance/length to physical millimeters (optionally evaluated at local pixel u, v).
double convert_pixel_length_to_mm(
    const PlanarCalib2D& cal,
    double pixel_length,
    std::optional<double> u = std::nullopt,
    std::optional<double> v = std::nullopt);
double convert_pixel_length_to_mm(
    const std::filesystem::path& iwcal_path,
    double pixel_length,
    std::optional<double> u = std::nullopt,
    std::optional<double> v = std::nullopt);
double convert_pixel_length_to_mm(
    const slam_common::JsonValue& cal_json,
    double pixel_length,
    std::optional<double> u = std::nullopt,
    std::optional<double> v = std::nullopt);

/// Radial, Perspective (with center input), or Radial+Perspective distortion calibration.
DistortionModel calibrate_image_distortion(
    const std::vector<std::vector<cv::Point3f>>& obj_pts_list,
    const std::vector<std::vector<cv::Point2f>>& img_pts_list,
    cv::Size image_size,
    const std::string& distortion_type = "radial_perspective",
    std::optional<Eigen::Vector2d> center_point = std::nullopt,
    const std::string& weight_func = "least_squares");

/// Apply calibrated radial and/or perspective distortion correction to an image.
cv::Mat undistort_image_with_model(const cv::Mat& image, const DistortionModel& model);

/// Calibrate 2D pixel coordinate mapping between Camera 1 and Camera 2.
DualCamMapping2D calibrate_dual_camera_mapping_2d(
    const std::vector<Eigen::Vector2d>& cam1_pts,
    const std::vector<Eigen::Vector2d>& cam2_pts,
    const std::string& dof = "affine",
    const std::string& weight_func = "least_squares",
    double weight_param = 1.345,
    double ransac_thresh = 2.0);

/// Save 2D planar / distortion / mapping calibration payload as .iwcal JSON.
std::filesystem::path save_iwcal_file(const std::filesystem::path& path, const slam_common::JsonValue& data);

/// Load saved .iwcal JSON calibration payload.
slam_common::JsonValue load_iwcal_file(const std::filesystem::path& path);

}  // namespace vision_calibration
