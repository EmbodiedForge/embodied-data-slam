#pragma once

#include <Eigen/Core>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace handeye_calibration {

struct NPointTransPoint {
    int index = 0;
    double x = 0.0;
    double y = 0.0;
};

struct NPointPlan {
    std::vector<NPointTransPoint> trans_points;
    std::vector<double> rot_angles_deg;
    int origin_index = 4;
    Eigen::Vector2d origin_world = Eigen::Vector2d::Zero();
};

struct RotationCenterResult {
    double cx = 0.0;
    double cy = 0.0;
    double radius = 0.0;
};

struct NPointCalibrationResult {
    std::string tool = "n_point_2d";
    std::string camera_mode = "static_up";
    std::string dof = "affine";
    std::string weight_func = "least_squares";
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d inv_matrix = Eigen::Matrix3d::Identity();
    double rms_mm = 0.0;
    double max_err_mm = 0.0;
    double rms_px = 0.0;
    double pixel_size_mm = 0.0;
    double scale_x_mm_px = 0.0;
    double scale_y_mm_px = 0.0;
    double rotation_deg = 0.0;
    std::optional<Eigen::Vector2d> rotation_center_px;
    std::optional<Eigen::Vector2d> rotation_center_world;
    std::optional<double> rotation_radius_px;
    std::optional<double> rotation_radius_mm;
    std::vector<bool> inlier_mask;
    int inlier_count = 0;
    int total_points = 0;
    std::vector<double> residuals_mm;
};

struct AlignmentOffset2D {
    double dx_mm = 0.0;
    double dy_mm = 0.0;
    double dtheta_deg = 0.0;
    double residual_mm = 0.0;
    Eigen::Vector2d rotation_center_world = Eigen::Vector2d::Zero();
    int point_count = 0;
};

/// Generate an S-shaped translation + rotation path matching VisionMaster N-Point calibration.
NPointPlan generate_n_point_plan(
    int trans_count = 9,
    int rot_count = 3,
    int origin_index = 4,
    double base_x = 0.0,
    double base_y = 0.0,
    double step_x = 5.0,
    double step_y = 5.0,
    const std::string& move_first = "X",
    int turn_count = 3,
    double base_angle_deg = -10.0,
    double step_angle_deg = 10.0);

/// Apply a 3x3 2D calibration matrix to transform pixel (u, v) -> physical (X, Y).
Eigen::Vector2d transform_point_2d(const Eigen::Matrix3d& matrix, double u, double v);

/// Estimate 2D rotation axis center (u_c, v_c) and mean radius from rotated marker observations.
RotationCenterResult solve_rotation_center_2d(
    const std::vector<Eigen::Vector2d>& rot_pixel_points,
    const std::vector<double>& rot_angles_deg = {});

/// Solve VisionMaster 2D N-Point hand-eye transform & optional rotation axis center.
NPointCalibrationResult calibrate_n_point_2d(
    const std::vector<Eigen::Vector2d>& pixel_points,
    const std::vector<Eigen::Vector2d>& world_points,
    const std::vector<Eigen::Vector2d>& rot_pixel_points = {},
    const std::vector<double>& rot_angles_deg = {},
    const std::string& camera_mode = "static_up",
    const std::string& dof = "affine",
    const std::string& weight_func = "least_squares",
    double weight_param = 1.345,
    double ransac_thresh = 2.0,
    double sample_ratio = 0.8);

/// Solve VisionMaster 5.7 Single-Point / Point-Set / Line alignment offset (dX, dY, dTheta).
AlignmentOffset2D compute_alignment_offset_2d(
    const std::vector<Eigen::Vector2d>& target_world_pts,
    const std::vector<Eigen::Vector2d>& current_world_pts,
    std::optional<Eigen::Vector2d> rotation_center_world = std::nullopt,
    const std::string& camera_mode = "static_up");

}  // namespace handeye_calibration
