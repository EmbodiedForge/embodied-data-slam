#include "handeye_calibration/n_point_2d.hpp"

#include "slam_common/geometry.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace handeye_calibration {
namespace {

constexpr double kPi = 3.14159265358979323846;

double deg2rad(double deg) { return deg * kPi / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / kPi; }

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

}  // namespace

NPointPlan generate_n_point_plan(
    int trans_count,
    int rot_count,
    int origin_index,
    double base_x,
    double base_y,
    double step_x,
    double step_y,
    const std::string& move_first,
    int turn_count,
    double base_angle_deg,
    double step_angle_deg) {
    turn_count = std::max(1, turn_count);
    if (trans_count < 4) {
        throw std::invalid_argument("平移标定点数至少需要 4 个（建议 9 点）");
    }
    if (origin_index < 0 || origin_index >= trans_count) {
        throw std::invalid_argument("标定原点索引必须在 [0, " + std::to_string(trans_count - 1) + "] 之间");
    }

    std::string mf = trim_lower(move_first);
    bool first_x = mf.empty() || mf.front() == 'x';

    std::vector<std::pair<int, int>> raw_coords;
    raw_coords.reserve(static_cast<size_t>(trans_count));
    for (int k = 0; k < trans_count; ++k) {
        int stripe = k / turn_count;
        int pos = k % turn_count;
        if (stripe % 2 == 1) pos = turn_count - 1 - pos;
        if (first_x) raw_coords.emplace_back(pos, stripe);
        else raw_coords.emplace_back(stripe, pos);
    }

    int ox = raw_coords[origin_index].first;
    int oy = raw_coords[origin_index].second;
    NPointPlan plan;
    plan.origin_index = origin_index;
    plan.origin_world = Eigen::Vector2d(base_x, base_y);
    for (int k = 0; k < trans_count; ++k) {
        plan.trans_points.push_back({
            k,
            base_x + static_cast<double>(raw_coords[k].first - ox) * step_x,
            base_y + static_cast<double>(raw_coords[k].second - oy) * step_y,
        });
    }
    for (int k = 0; k < std::max(0, rot_count); ++k) {
        plan.rot_angles_deg.push_back(base_angle_deg + static_cast<double>(k) * step_angle_deg);
    }
    return plan;
}

Eigen::Vector2d transform_point_2d(const Eigen::Matrix3d& matrix, double u, double v) {
    auto res = slam_common::apply_matrix_2d(matrix, {Eigen::Vector2d(u, v)});
    return res[0];
}

RotationCenterResult solve_rotation_center_2d(
    const std::vector<Eigen::Vector2d>& pts,
    const std::vector<double>& rot_angles_deg) {
    if (pts.size() < 2) {
        throw std::invalid_argument("旋转中心求解至少需要 2 个旋转观测点");
    }

    if (!rot_angles_deg.empty() && rot_angles_deg.size() == pts.size()) {
        std::vector<Eigen::Matrix2d> A_blocks;
        std::vector<Eigen::Vector2d> b_blocks;
        for (size_t i = 0; i < pts.size(); ++i) {
            for (size_t j = i + 1; j < pts.size(); ++j) {
                double dth = deg2rad(rot_angles_deg[j] - rot_angles_deg[i]);
                if (std::abs(std::sin(dth)) < 1e-3 && std::abs(1.0 - std::cos(dth)) < 1e-3) continue;
                Eigen::Matrix2d R;
                R << std::cos(dth), -std::sin(dth),
                     std::sin(dth),  std::cos(dth);
                A_blocks.push_back(Eigen::Matrix2d::Identity() - R);
                b_blocks.push_back(pts[j] - R * pts[i]);
            }
        }
        if (!A_blocks.empty()) {
            Eigen::MatrixXd A(static_cast<Eigen::Index>(2 * A_blocks.size()), 2);
            Eigen::VectorXd b(static_cast<Eigen::Index>(2 * A_blocks.size()));
            for (size_t k = 0; k < A_blocks.size(); ++k) {
                A.block<2, 2>(static_cast<Eigen::Index>(2 * k), 0) = A_blocks[k];
                b.segment<2>(static_cast<Eigen::Index>(2 * k)) = b_blocks[k];
            }
            Eigen::JacobiSVD<Eigen::MatrixXd> svd_c(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::Vector2d center = svd_c.solve(b);
            double rad_sum = 0.0;
            for (const auto& p : pts) rad_sum += (p - center).norm();
            return {center.x(), center.y(), rad_sum / static_cast<double>(pts.size())};
        }
    }

    if (pts.size() < 3) {
        throw std::invalid_argument("未提供旋转角度时，求解旋转中心至少需要 3 个不共线点");
    }

    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& p : pts) mean += p;
    mean /= static_cast<double>(pts.size());

    Eigen::MatrixXd A(static_cast<Eigen::Index>(pts.size()), 3);
    Eigen::VectorXd rhs(static_cast<Eigen::Index>(pts.size()));
    for (size_t i = 0; i < pts.size(); ++i) {
        double u = pts[i].x() - mean.x();
        double v = pts[i].y() - mean.y();
        A(static_cast<Eigen::Index>(i), 0) = u;
        A(static_cast<Eigen::Index>(i), 1) = v;
        A(static_cast<Eigen::Index>(i), 2) = 1.0;
        rhs(static_cast<Eigen::Index>(i)) = u * u + v * v;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.rank() < 3) {
        throw std::invalid_argument("旋转观测点近似共线或重合，无法确定旋转中心");
    }
    Eigen::Vector3d sol = svd.solve(rhs);
    double cx = mean.x() + 0.5 * sol(0);
    double cy = mean.y() + 0.5 * sol(1);
    double r = std::sqrt(std::max(0.0, std::pow(0.5 * sol(0), 2) + std::pow(0.5 * sol(1), 2) + sol(2)));
    return {cx, cy, r};
}

NPointCalibrationResult calibrate_n_point_2d(
    const std::vector<Eigen::Vector2d>& pixel_points,
    const std::vector<Eigen::Vector2d>& world_points,
    const std::vector<Eigen::Vector2d>& rot_pixel_points,
    const std::vector<double>& rot_angles_deg,
    const std::string& camera_mode,
    const std::string& dof,
    const std::string& weight_func,
    double weight_param,
    double ransac_thresh,
    double sample_ratio) {
    if (pixel_points.size() != world_points.size()) {
        throw std::invalid_argument("像素点数量与物理坐标点数量必须一致");
    }

    auto fit = slam_common::fit_2d_robust(
        pixel_points, world_points, dof, weight_func, weight_param, ransac_thresh, sample_ratio, true);
    Eigen::Matrix3d inv_M = fit.matrix.inverse();

    auto pred_world = slam_common::apply_matrix_2d(fit.matrix, pixel_points);
    auto pred_px = slam_common::apply_matrix_2d(inv_M, world_points);
    size_t n = pixel_points.size();
    std::vector<double> err_world(n), err_px(n);
    for (size_t i = 0; i < n; ++i) {
        err_world[i] = (pred_world[i] - world_points[i]).norm();
        err_px[i] = (pred_px[i] - pixel_points[i]).norm();
    }

    bool any_inl = std::any_of(fit.inliers.begin(), fit.inliers.end(), [](bool b) { return b; });
    double sum_w2 = 0.0, sum_p2 = 0.0, max_w = 0.0;
    int eval_cnt = 0;
    int inlier_cnt = 0;
    for (size_t i = 0; i < n; ++i) {
        if (fit.inliers[i]) ++inlier_cnt;
        if (!any_inl || fit.inliers[i]) {
            sum_w2 += err_world[i] * err_world[i];
            sum_p2 += err_px[i] * err_px[i];
            max_w = std::max(max_w, err_world[i]);
            ++eval_cnt;
        }
    }

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto& p : pixel_points) centroid += p;
    centroid /= static_cast<double>(n);
    Eigen::Vector2d p0 = transform_point_2d(fit.matrix, centroid.x(), centroid.y());
    Eigen::Vector2d du = transform_point_2d(fit.matrix, centroid.x() + 1.0, centroid.y()) - p0;
    Eigen::Vector2d dv = transform_point_2d(fit.matrix, centroid.x(), centroid.y() + 1.0) - p0;
    double scale_x = du.norm();
    double scale_y = dv.norm();
    double pixel_size_mm = std::sqrt(std::max(scale_x * scale_y, 1e-12));
    double rotation_deg = rad2deg(std::atan2(du.y(), du.x()));

    NPointCalibrationResult res;
    res.camera_mode = camera_mode;
    res.dof = fit.dof_key;
    res.weight_func = fit.weight_key;
    res.matrix = fit.matrix;
    res.inv_matrix = inv_M;
    res.rms_mm = std::sqrt(sum_w2 / static_cast<double>(std::max(1, eval_cnt)));
    res.max_err_mm = max_w;
    res.rms_px = std::sqrt(sum_p2 / static_cast<double>(std::max(1, eval_cnt)));
    res.pixel_size_mm = pixel_size_mm;
    res.scale_x_mm_px = scale_x;
    res.scale_y_mm_px = scale_y;
    res.rotation_deg = rotation_deg;
    res.inlier_mask = fit.inliers;
    res.inlier_count = inlier_cnt;
    res.total_points = static_cast<int>(n);
    res.residuals_mm = std::move(err_world);

    if (rot_pixel_points.size() >= 2) {
        auto rc = solve_rotation_center_2d(rot_pixel_points, rot_angles_deg);
        Eigen::Vector2d rw = transform_point_2d(fit.matrix, rc.cx, rc.cy);
        res.rotation_center_px = Eigen::Vector2d(rc.cx, rc.cy);
        res.rotation_center_world = rw;
        res.rotation_radius_px = rc.radius;
        res.rotation_radius_mm = rc.radius * pixel_size_mm;
    }
    return res;
}

AlignmentOffset2D compute_alignment_offset_2d(
    const std::vector<Eigen::Vector2d>& tgt,
    const std::vector<Eigen::Vector2d>& cur,
    std::optional<Eigen::Vector2d> rotation_center_world,
    const std::string& camera_mode) {
    if (tgt.size() != cur.size() || tgt.empty()) {
        throw std::invalid_argument("对位计算需要相同数量的目标点与当前特征点（至少 1 对）");
    }
    Eigen::Vector2d rc = Eigen::Vector2d::Zero();
    if (rotation_center_world.has_value()) {
        rc = *rotation_center_world;
    } else {
        for (const auto& p : cur) rc += p;
        rc /= static_cast<double>(cur.size());
    }

    double dtheta_rad = 0.0;
    Eigen::Vector2d delta = Eigen::Vector2d::Zero();
    double residual = 0.0;

    if (tgt.size() == 1) {
        delta = tgt[0] - cur[0];
    } else {
        Eigen::Vector2d tgt_mean = Eigen::Vector2d::Zero();
        Eigen::Vector2d cur_mean = Eigen::Vector2d::Zero();
        for (size_t i = 0; i < tgt.size(); ++i) {
            tgt_mean += tgt[i];
            cur_mean += cur[i];
        }
        tgt_mean /= static_cast<double>(tgt.size());
        cur_mean /= static_cast<double>(cur.size());

        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < tgt.size(); ++i) {
            H += (cur[i] - cur_mean) * (tgt[i] - tgt_mean).transpose();
        }
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d Vt = svd.matrixV().transpose();
        Eigen::Matrix2d R = Vt.transpose() * U.transpose();
        if (R.determinant() < 0.0) {
            Vt.row(1) *= -1.0;
            R = Vt.transpose() * U.transpose();
        }
        dtheta_rad = std::atan2(R(1, 0), R(0, 0));

        std::vector<Eigen::Vector2d> rotated_cur(cur.size());
        for (size_t i = 0; i < cur.size(); ++i) {
            rotated_cur[i] = rc + R * (cur[i] - rc);
            delta += (tgt[i] - rotated_cur[i]);
        }
        delta /= static_cast<double>(cur.size());
        double sq = 0.0;
        for (size_t i = 0; i < cur.size(); ++i) {
            sq += (tgt[i] - (rotated_cur[i] + delta)).squaredNorm();
        }
        residual = std::sqrt(sq / static_cast<double>(cur.size()));
    }

    double sign = (trim_lower(camera_mode) == "moving") ? -1.0 : 1.0;
    return {
        sign * delta.x(),
        sign * delta.y(),
        sign * rad2deg(dtheta_rad),
        residual,
        rc,
        static_cast<int>(tgt.size()),
    };
}

}  // namespace handeye_calibration
