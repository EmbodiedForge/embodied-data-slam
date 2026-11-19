#include "handeye_calibration/handeye_3d.hpp"

#include "slam_common/geometry.hpp"
#include "slam_common/json.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace handeye_calibration {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::string trim_lower(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    std::string out = s.substr(b, e - b + 1);
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return m;
}

Eigen::Vector3d rot_to_axis_angle(const Eigen::Matrix3d& R) {
    cv::Mat R_cv(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) R_cv.at<double>(r, c) = R(r, c);
    }
    cv::Mat rvec;
    cv::Rodrigues(R_cv, rvec);
    cv::Mat rv;
    rvec.convertTo(rv, CV_64F);
    return Eigen::Vector3d(rv.at<double>(0), rv.at<double>(1), rv.at<double>(2));
}

Eigen::Matrix4d quat_left_matrix(const Eigen::Vector4d& q_wxyz) {
    double w = q_wxyz(0), x = q_wxyz(1), y = q_wxyz(2), z = q_wxyz(3);
    Eigen::Matrix4d M;
    M << w, -x, -y, -z,
         x,  w, -z,  y,
         y,  z,  w, -x,
         z, -y,  x,  w;
    return M;
}

Eigen::Matrix4d quat_right_matrix(const Eigen::Vector4d& q_wxyz) {
    double w = q_wxyz(0), x = q_wxyz(1), y = q_wxyz(2), z = q_wxyz(3);
    Eigen::Matrix4d M;
    M << w, -x, -y, -z,
         x,  w,  z, -y,
         y, -z,  w,  x,
         z,  y, -x,  w;
    return M;
}

Eigen::Matrix3d solve_rx_park(
    const std::vector<Eigen::Matrix4d>& A_list,
    const std::vector<Eigen::Matrix4d>& B_list) {
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < A_list.size(); ++i) {
        Eigen::Vector3d alpha = rot_to_axis_angle(A_list[i].topLeftCorner<3, 3>());
        Eigen::Vector3d beta = rot_to_axis_angle(B_list[i].topLeftCorner<3, 3>());
        M += beta * alpha.transpose();
    }
    return slam_common::project_so3(M.transpose());
}

Eigen::Matrix3d solve_rx_tsai(
    const std::vector<Eigen::Matrix4d>& A_list,
    const std::vector<Eigen::Matrix4d>& B_list) {
    std::vector<Eigen::Matrix3d> rows;
    std::vector<Eigen::Vector3d> rhs;
    for (size_t i = 0; i < A_list.size(); ++i) {
        Eigen::Vector3d ra = rot_to_axis_angle(A_list[i].topLeftCorner<3, 3>());
        Eigen::Vector3d rb = rot_to_axis_angle(B_list[i].topLeftCorner<3, 3>());
        double na = ra.norm(), nb = rb.norm();
        if (na < 1e-4 || nb < 1e-4) continue;
        Eigen::Vector3d pa = 2.0 * std::sin(0.5 * na) * (ra / na);
        Eigen::Vector3d pb = 2.0 * std::sin(0.5 * nb) * (rb / nb);
        rows.push_back(skew(pa + pb));
        rhs.push_back(pb - pa);
    }
    if (rows.empty()) return solve_rx_park(A_list, B_list);
    Eigen::MatrixXd A(static_cast<Eigen::Index>(3 * rows.size()), 3);
    Eigen::VectorXd b(static_cast<Eigen::Index>(3 * rows.size()));
    for (size_t i = 0; i < rows.size(); ++i) {
        A.block<3, 3>(static_cast<Eigen::Index>(3 * i), 0) = rows[i];
        b.segment<3>(static_cast<Eigen::Index>(3 * i)) = rhs[i];
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3d p_prime = svd.solve(b);
    Eigen::Vector3d px = 2.0 * p_prime / std::sqrt(1.0 + p_prime.squaredNorm());
    double n2 = px.squaredNorm();
    if (n2 >= 4.0) return solve_rx_park(A_list, B_list);
    Eigen::Matrix3d Rx = (1.0 - 0.5 * n2) * Eigen::Matrix3d::Identity() +
                         0.5 * (px * px.transpose() + std::sqrt(std::max(0.0, 4.0 - n2)) * skew(px));
    return slam_common::project_so3(Rx);
}

Eigen::Matrix3d solve_rx_andreff(
    const std::vector<Eigen::Matrix4d>& A_list,
    const std::vector<Eigen::Matrix4d>& B_list) {
    Eigen::MatrixXd K_mat(static_cast<Eigen::Index>(9 * A_list.size()), 9);
    for (size_t i = 0; i < A_list.size(); ++i) {
        Eigen::Matrix3d Ra = A_list[i].topLeftCorner<3, 3>();
        Eigen::Matrix3d RbT = B_list[i].topLeftCorner<3, 3>().transpose();
        Eigen::Matrix<double, 9, 9> blk = Eigen::Matrix<double, 9, 9>::Zero();
        for (int b = 0; b < 3; ++b) {
            blk.block<3, 3>(b * 3, b * 3) += Ra;
        }
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                blk.block<3, 3>(r * 3, c * 3) -= RbT(r, c) * Eigen::Matrix3d::Identity();
            }
        }
        K_mat.block<9, 9>(static_cast<Eigen::Index>(9 * i), 0) = blk;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(K_mat, Eigen::ComputeFullV);
    Eigen::Matrix<double, 9, 1> v = svd.matrixV().col(8);
    // Column-major reshape matching Python order='F'
    Eigen::Matrix3d cand = Eigen::Map<const Eigen::Matrix3d>(v.data());
    if (cand.determinant() < 0.0) cand = -cand;
    return slam_common::project_so3(cand);
}

Eigen::Matrix3d solve_rx_horaud(
    const std::vector<Eigen::Matrix4d>& A_list,
    const std::vector<Eigen::Matrix4d>& B_list) {
    Eigen::MatrixXd M(static_cast<Eigen::Index>(4 * A_list.size()), 4);
    for (size_t i = 0; i < A_list.size(); ++i) {
        Eigen::Vector4d qa = slam_common::rot_to_quat_xyzw(A_list[i].topLeftCorner<3, 3>());
        Eigen::Vector4d qb = slam_common::rot_to_quat_xyzw(B_list[i].topLeftCorner<3, 3>());
        Eigen::Vector4d qa_w(qa(3), qa(0), qa(1), qa(2));
        Eigen::Vector4d qb_w(qb(3), qb(0), qb(1), qb(2));
        if (qa_w.dot(qb_w) < 0.0) qb_w = -qb_w;
        M.block<4, 4>(static_cast<Eigen::Index>(4 * i), 0) = quat_left_matrix(qa_w) - quat_right_matrix(qb_w);
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
    Eigen::Vector4d qw = svd.matrixV().col(3);
    return slam_common::quat_xyzw_to_rot(Eigen::Vector4d(qw(1), qw(2), qw(3), qw(0)));
}

std::pair<Eigen::Matrix3d, std::optional<Eigen::Vector3d>> solve_daniilidis(
    const std::vector<Eigen::Matrix4d>& A_list,
    const std::vector<Eigen::Matrix4d>& B_list) {
    Eigen::MatrixXd T(static_cast<Eigen::Index>(6 * A_list.size()), 8);
    for (size_t i = 0; i < A_list.size(); ++i) {
        Eigen::Vector4d qa = slam_common::rot_to_quat_xyzw(A_list[i].topLeftCorner<3, 3>());
        Eigen::Vector4d qb = slam_common::rot_to_quat_xyzw(B_list[i].topLeftCorner<3, 3>());
        Eigen::Vector4d ra(qa(3), qa(0), qa(1), qa(2));
        Eigen::Vector4d rb(qb(3), qb(0), qb(1), qb(2));
        if (ra.dot(rb) < 0.0) rb = -rb;
        Eigen::Vector3d ta = A_list[i].topRightCorner<3, 1>();
        Eigen::Vector3d tb = B_list[i].topRightCorner<3, 1>();
        Eigen::Vector4d pa = 0.5 * (quat_left_matrix(Eigen::Vector4d(0.0, ta(0), ta(1), ta(2))) * ra);
        Eigen::Vector4d pb = 0.5 * (quat_left_matrix(Eigen::Vector4d(0.0, tb(0), tb(1), tb(2))) * rb);

        Eigen::Matrix<double, 3, 8> S1 = Eigen::Matrix<double, 3, 8>::Zero();
        S1.block<3, 1>(0, 0) = ra.tail<3>() - rb.tail<3>();
        S1.block<3, 3>(0, 1) = skew(ra.tail<3>() + rb.tail<3>());

        Eigen::Matrix<double, 3, 8> S2 = Eigen::Matrix<double, 3, 8>::Zero();
        S2.block<3, 1>(0, 0) = pa.tail<3>() - pb.tail<3>();
        S2.block<3, 3>(0, 1) = skew(pa.tail<3>() + pb.tail<3>());
        S2.block<3, 1>(0, 4) = ra.tail<3>() - rb.tail<3>();
        S2.block<3, 3>(0, 5) = skew(ra.tail<3>() + rb.tail<3>());

        T.block<3, 8>(static_cast<Eigen::Index>(6 * i), 0) = S1;
        T.block<3, 8>(static_cast<Eigen::Index>(6 * i + 3), 0) = S2;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(T, Eigen::ComputeFullV);
    Eigen::Matrix<double, 8, 1> v7 = svd.matrixV().col(6);
    Eigen::Matrix<double, 8, 1> v8 = svd.matrixV().col(7);
    Eigen::Vector4d u1 = v7.head<4>(), v1 = v7.tail<4>();
    Eigen::Vector4d u2 = v8.head<4>(), v2 = v8.tail<4>();

    double a = u1.dot(v1);
    double b = u1.dot(v2) + u2.dot(v1);
    double c = u2.dot(v2);
    double disc = std::max(0.0, b * b - 4.0 * a * c);

    std::vector<std::pair<double, double>> candidates;
    if (std::abs(a) > 1e-12) {
        for (double sgn : {1.0, -1.0}) {
            candidates.emplace_back((-b + sgn * std::sqrt(disc)) / (2.0 * a), 1.0);
        }
    } else if (std::abs(b) > 1e-12) {
        candidates.emplace_back(-c / b, 1.0);
    } else {
        candidates.emplace_back(1.0, 0.0);
    }

    bool found = false;
    Eigen::Matrix3d best_Rx = Eigen::Matrix3d::Identity();
    Eigen::Vector3d best_tx = Eigen::Vector3d::Zero();
    double best_err = 1e18;
    for (const auto& [l1, l2] : candidates) {
        Eigen::Vector4d u = l1 * u1 + l2 * u2;
        Eigen::Vector4d v = l1 * v1 + l2 * v2;
        double norm_u = u.norm();
        if (norm_u < 1e-9) continue;
        Eigen::Vector4d q_wxyz = u / norm_u;
        Eigen::Vector4d qd_wxyz = v / norm_u;
        Eigen::Matrix3d Rx = slam_common::quat_xyzw_to_rot(Eigen::Vector4d(q_wxyz(1), q_wxyz(2), q_wxyz(3), q_wxyz(0)));
        Eigen::Vector4d q_conj(q_wxyz(0), -q_wxyz(1), -q_wxyz(2), -q_wxyz(3));
        Eigen::Vector4d t_q = 2.0 * (quat_left_matrix(qd_wxyz) * q_conj);
        Eigen::Vector3d tx = t_q.tail<3>();
        double err = 0.0;
        for (size_t i = 0; i < A_list.size(); ++i) {
            err += ((A_list[i].topLeftCorner<3, 3>() - Eigen::Matrix3d::Identity()) * tx -
                    (Rx * B_list[i].topRightCorner<3, 1>() - A_list[i].topRightCorner<3, 1>())).norm();
        }
        if (err < best_err) {
            best_err = err;
            best_Rx = Rx;
            best_tx = tx;
            found = true;
        }
    }
    if (!found) return {solve_rx_horaud(A_list, B_list), std::nullopt};
    return {best_Rx, best_tx};
}

Eigen::Vector3d solve_tx_linear(
    const std::vector<Eigen::Matrix4d>& A_list,
    const std::vector<Eigen::Matrix4d>& B_list,
    const Eigen::Matrix3d& Rx) {
    Eigen::MatrixXd C(static_cast<Eigen::Index>(3 * A_list.size()), 3);
    Eigen::VectorXd d(static_cast<Eigen::Index>(3 * A_list.size()));
    for (size_t i = 0; i < A_list.size(); ++i) {
        C.block<3, 3>(static_cast<Eigen::Index>(3 * i), 0) =
            A_list[i].topLeftCorner<3, 3>() - Eigen::Matrix3d::Identity();
        d.segment<3>(static_cast<Eigen::Index>(3 * i)) =
            (Rx * B_list[i].topRightCorner<3, 1>()) - A_list[i].topRightCorner<3, 1>();
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(C, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return svd.solve(d);
}

Eigen::Matrix4d mean_se3(const std::vector<Eigen::Matrix4d>& transforms) {
    Eigen::Matrix3d R_sum = Eigen::Matrix3d::Zero();
    Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();
    for (const auto& T : transforms) {
        R_sum += T.topLeftCorner<3, 3>();
        t_sum += T.topRightCorner<3, 1>();
    }
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.topLeftCorner<3, 3>() = slam_common::project_so3(R_sum);
    out.topRightCorner<3, 1>() = t_sum / static_cast<double>(transforms.size());
    return out;
}

}  // namespace

Eigen::Matrix4d pose_to_matrix(const Eigen::Matrix4d& mat) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = slam_common::project_so3(mat.topLeftCorner<3, 3>());
    T.topRightCorner<3, 1>() = mat.topRightCorner<3, 1>();
    return T;
}

Eigen::Matrix4d pose_to_matrix(const std::vector<double>& flat) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    if (flat.size() == 7) {
        T.topLeftCorner<3, 3>() = slam_common::quat_xyzw_to_rot(Eigen::Vector4d(flat[3], flat[4], flat[5], flat[6]));
        T.topRightCorner<3, 1>() = Eigen::Vector3d(flat[0], flat[1], flat[2]);
        return T;
    }
    if (flat.size() == 6) {
        cv::Mat R_cv;
        cv::Rodrigues(cv::Vec3d(flat[3], flat[4], flat[5]), R_cv);
        Eigen::Matrix3d R;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) R(r, c) = R_cv.at<double>(r, c);
        }
        T.topLeftCorner<3, 3>() = slam_common::project_so3(R);
        T.topRightCorner<3, 1>() = Eigen::Vector3d(flat[0], flat[1], flat[2]);
        return T;
    }
    if (flat.size() == 16) {
        Eigen::Matrix4d mat;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) mat(r, c) = flat[static_cast<size_t>(r * 4 + c)];
        }
        return pose_to_matrix(mat);
    }
    throw std::invalid_argument("无法识别的位姿维度: len=" + std::to_string(flat.size()));
}

std::vector<Eigen::Matrix4d> load_poses_file(const std::filesystem::path& path) {
    std::filesystem::path p = std::filesystem::weakly_canonical(path);
    std::string ext = trim_lower(p.extension().string());
    if (ext == ".json") {
        slam_common::JsonValue data = slam_common::read_json_file(p);
        if (data.is_object()) {
            for (const char* k : {"poses", "T_base_gripper", "T_cam_target", "transforms"}) {
                if (data.contains(k) && data.get(k).is_array()) {
                    data = data.get(k);
                    break;
                }
            }
        }
        if (!data.is_array()) throw std::invalid_argument("JSON 位姿文件顶层需为数组");
        std::vector<Eigen::Matrix4d> poses;
        for (const auto& item : data.as_array()) {
            if (!item.is_array()) throw std::invalid_argument("无效位姿项");
            const auto& arr = item.as_array();
            if (arr.size() == 4 && arr[0].is_array()) {
                Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
                for (int r = 0; r < 4; ++r) {
                    for (int c = 0; c < 4; ++c) m(r, c) = arr[r].as_array()[c].as_double();
                }
                poses.push_back(pose_to_matrix(m));
            } else {
                std::vector<double> vec;
                vec.reserve(arr.size());
                for (const auto& v : arr) vec.push_back(v.as_double());
                poses.push_back(pose_to_matrix(vec));
            }
        }
        return poses;
    }

    std::ifstream f(p);
    if (!f) throw std::runtime_error("无法打开位姿文件: " + p.string());
    std::vector<Eigen::Matrix4d> poses;
    std::string line;
    while (std::getline(f, line)) {
        size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos || line[first] == '#') continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::replace(line.begin(), line.end(), ';', ' ');
        std::stringstream ss(line);
        std::vector<double> vals;
        double v = 0.0;
        while (ss >> v) vals.push_back(v);
        if (vals.size() == 6 || vals.size() == 7 || vals.size() == 16) {
            poses.push_back(pose_to_matrix(vals));
        }
    }
    if (poses.empty()) {
        throw std::invalid_argument("未从 " + p.filename().string() + " 解析到有效位姿行（支持 6/7/16 列数值）");
    }
    return poses;
}

HandEye3DResult calibrate_handeye_3d(
    const std::vector<Eigen::Matrix4d>& T_base_gripper_list,
    const std::vector<Eigen::Matrix4d>& T_cam_target_list,
    const std::string& mode,
    const std::string& method,
    double min_pair_angle_deg) {
    if (T_base_gripper_list.size() != T_cam_target_list.size() || T_base_gripper_list.size() < 3) {
        throw std::invalid_argument("3D 手眼标定至少需要 3 组一一对应的机械臂末端与视觉标定板位姿");
    }
    std::string mode_key = trim_lower(mode);
    if (mode_key != "eye_in_hand" && mode_key != "eye_to_hand") {
        throw std::invalid_argument("mode 必须为 eye_in_hand 或 eye_to_hand");
    }

    size_t n = T_base_gripper_list.size();
    std::vector<Eigen::Matrix4d> T_bg(n), T_ct(n), T_ref(n);
    for (size_t i = 0; i < n; ++i) {
        T_bg[i] = pose_to_matrix(T_base_gripper_list[i]);
        T_ct[i] = pose_to_matrix(T_cam_target_list[i]);
        T_ref[i] = (mode_key == "eye_to_hand") ? T_bg[i].inverse() : T_bg[i];
    }

    std::vector<Eigen::Matrix4d> A_list, B_list;
    double min_rad = min_pair_angle_deg * kPi / 180.0;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            Eigen::Matrix4d A = T_ref[j].inverse() * T_ref[i];
            Eigen::Matrix4d B = T_ct[j] * T_ct[i].inverse();
            if (rot_to_axis_angle(A.topLeftCorner<3, 3>()).norm() >= min_rad &&
                rot_to_axis_angle(B.topLeftCorner<3, 3>()).norm() >= min_rad) {
                A_list.push_back(A);
                B_list.push_back(B);
            }
        }
    }
    if (A_list.size() < 2) {
        throw std::invalid_argument("位姿间旋转变化太小，请采集包含多角度旋转（绕不同轴）的机械臂运动数据");
    }

    std::string m_key = trim_lower(method);
    Eigen::Matrix3d Rx = Eigen::Matrix3d::Identity();
    std::optional<Eigen::Vector3d> tx_init;
    if (m_key == "tsai") Rx = solve_rx_tsai(A_list, B_list);
    else if (m_key == "park") Rx = solve_rx_park(A_list, B_list);
    else if (m_key == "horaud") Rx = solve_rx_horaud(A_list, B_list);
    else if (m_key == "andreff") Rx = solve_rx_andreff(A_list, B_list);
    else if (m_key == "daniilidis") {
        auto sol = solve_daniilidis(A_list, B_list);
        Rx = sol.first;
        tx_init = sol.second;
    } else {
        throw std::invalid_argument("未知 3D 手眼标定算法: " + method);
    }

    Eigen::Vector3d tx = tx_init.has_value() ? *tx_init : solve_tx_linear(A_list, B_list, Rx);
    Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
    X.topLeftCorner<3, 3>() = slam_common::project_so3(Rx);
    X.topRightCorner<3, 1>() = tx;

    std::vector<Eigen::Matrix4d> consts(n);
    for (size_t i = 0; i < n; ++i) consts[i] = T_ref[i] * X * T_ct[i];
    Eigen::Matrix4d T_const_mean = mean_se3(consts);

    double r_sq = 0.0, t_sq = 0.0, r_max = 0.0, t_max = 0.0;
    Eigen::Matrix4d T_const_inv = T_const_mean.inverse();
    for (const auto& C : consts) {
        Eigen::Matrix4d dC = T_const_inv * C;
        double r_deg = rot_to_axis_angle(dC.topLeftCorner<3, 3>()).norm() * 180.0 / kPi;
        double t_m = dC.topRightCorner<3, 1>().norm();
        r_sq += r_deg * r_deg;
        t_sq += t_m * t_m;
        r_max = std::max(r_max, r_deg);
        t_max = std::max(t_max, t_m);
    }

    HandEye3DResult out;
    out.mode = mode_key;
    out.method = m_key;
    out.result_name = (mode_key == "eye_in_hand") ? "T_gripper_cam" : "T_base_cam";
    out.T_result = X;
    out.T_const_other = T_const_mean;
    out.rot_rmse_deg = std::sqrt(r_sq / static_cast<double>(n));
    out.trans_rmse_m = std::sqrt(t_sq / static_cast<double>(n));
    out.rot_max_deg = r_max;
    out.trans_max_m = t_max;
    out.pose_count = static_cast<int>(n);
    out.motion_pair_count = static_cast<int>(A_list.size());
    return out;
}

}  // namespace handeye_calibration
