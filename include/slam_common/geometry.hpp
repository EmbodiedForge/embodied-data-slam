#pragma once

#include <Eigen/Core>
#include <array>
#include <string>
#include <vector>

namespace slam_common {

/// Orthogonal projection of an approximate 3x3 rotation matrix onto SO(3) via SVD.
Eigen::Matrix3d project_so3(const Eigen::Matrix3d& R);

/// Validate and project a 4x4 homogeneous rigid transformation matrix.
Eigen::Matrix4d rigid_matrix(const Eigen::Matrix4d& m);

/// Convert 3x3 rotation matrix to unit quaternion in (qx, qy, qz, qw) order.
Eigen::Vector4d rot_to_quat_xyzw(const Eigen::Matrix3d& R);

/// Convert (qx, qy, qz, qw) quaternion to SO(3) rotation matrix.
Eigen::Matrix3d quat_xyzw_to_rot(const Eigen::Vector4d& q);

/// Convert trajectory (qx, qy, qz, qw) quaternion to rotation matrix, rejecting |norm-1| > 0.01.
Eigen::Matrix3d quaternion_to_rot_strict(const Eigen::Vector4d& q);

/// Compute 4 local 3D corners of an AprilGrid tag given layout and corner order permutation.
std::array<Eigen::Vector3d, 4> aprilgrid_tag_corners_3d(
    int tag_id,
    int rows,
    int cols,
    double tag_size,
    double tag_spacing,
    bool bottom_up = false,
    bool rev = false,
    int roll = 2);

/// Apply a 3x3 planar homography/affine/similarity matrix to 2D points.
std::vector<Eigen::Vector2d> apply_matrix_2d(
    const Eigen::Matrix3d& M,
    const std::vector<Eigen::Vector2d>& pts);

/// Hartley 2D point normalization (centroid at origin, mean distance sqrt(2)).
void normalize_2d(
    const std::vector<Eigen::Vector2d>& pts,
    Eigen::Matrix3d& T_out,
    std::vector<Eigen::Vector2d>& pts_norm_out);

/// Weighted 2D similarity fit; allow_reflection controls whether mirroring is permitted.
Eigen::Matrix3d fit_similarity_2d(
    const std::vector<Eigen::Vector2d>& src,
    const std::vector<Eigen::Vector2d>& dst,
    const std::vector<double>& weights = {},
    bool allow_reflection = true);

/// Weighted 2D affine least-squares fit.
Eigen::Matrix3d fit_affine_2d(
    const std::vector<Eigen::Vector2d>& src,
    const std::vector<Eigen::Vector2d>& dst,
    const std::vector<double>& weights = {});

/// Weighted normalized DLT 2D homography fit.
Eigen::Matrix3d fit_homography_2d(
    const std::vector<Eigen::Vector2d>& src,
    const std::vector<Eigen::Vector2d>& dst,
    const std::vector<double>& weights = {});

/// Normalize 2D DOF alias (similarity / affine / homography).
std::string normalize_dof_2d(const std::string& dof);

/// Normalize 2D robust weight alias (least_squares / huber / tukey / ransac).
std::string normalize_weight_2d(const std::string& weight_func);

struct Robust2DFitResult {
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
    std::vector<bool> inliers;
    std::string dof_key;
    std::string weight_key;
};

/// Unified 2D transform fit supporting least_squares, IRLS (Huber/Tukey), and RANSAC.
Robust2DFitResult fit_2d_robust(
    const std::vector<Eigen::Vector2d>& src,
    const std::vector<Eigen::Vector2d>& dst,
    const std::string& dof = "affine",
    const std::string& weight_func = "least_squares",
    double weight_param = 1.345,
    double ransac_thresh = 1.5,
    double sample_ratio = 0.8,
    bool allow_reflection = true);

}  // namespace slam_common
