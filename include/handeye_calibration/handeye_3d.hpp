#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <string>
#include <vector>

namespace handeye_calibration {

struct HandEye3DResult {
    std::string mode;
    std::string method;
    std::string result_name;
    Eigen::Matrix4d T_result = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_const_other = Eigen::Matrix4d::Identity();
    double rot_rmse_deg = 0.0;
    double trans_rmse_m = 0.0;
    double rot_max_deg = 0.0;
    double trans_max_m = 0.0;
    int pose_count = 0;
    int motion_pair_count = 0;
};

/// Convert [x,y,z,rx,ry,rz], [x,y,z,qx,qy,qz,qw], or 16-element row-major 4x4 into SE(3) matrix.
Eigen::Matrix4d pose_to_matrix(const std::vector<double>& pose_vec);

/// Project an approximate 4x4 matrix onto SE(3).
Eigen::Matrix4d pose_to_matrix(const Eigen::Matrix4d& mat);

/// Load list of 4x4 poses from JSON (array or dict) or CSV (6/7/16 columns).
std::vector<Eigen::Matrix4d> load_poses_file(const std::filesystem::path& path);

/// Solve 3D Robot Hand-Eye Calibration (eye_in_hand AX=XB or eye_to_hand AX=ZB).
/// Supported methods: tsai, park, horaud, andreff, daniilidis.
HandEye3DResult calibrate_handeye_3d(
    const std::vector<Eigen::Matrix4d>& T_base_gripper_list,
    const std::vector<Eigen::Matrix4d>& T_cam_target_list,
    const std::string& mode = "eye_in_hand",
    const std::string& method = "tsai",
    double min_pair_angle_deg = 2.0);

}  // namespace handeye_calibration
