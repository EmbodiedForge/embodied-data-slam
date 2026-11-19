#include "slam_common/geometry.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <numeric>
#include <random>
#include <stdexcept>

namespace slam_common {
namespace {

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

std::vector<double> sanitized_weights(size_t n, const std::vector<double>& weights) {
    if (weights.empty()) return std::vector<double>(n, 1.0);
    if (weights.size() != n) throw std::invalid_argument("Weight count mismatch");
    std::vector<double> out(n);
    for (size_t i = 0; i < n; ++i) {
        out[i] = std::max(weights[i], 1e-12);
    }
    return out;
}

double median_of(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t mid = v.size() / 2;
    if (v.size() % 2 == 1) return v[mid];
    return 0.5 * (v[mid - 1] + v[mid]);
}

std::vector<double> compute_irls_weights_2d(
    const std::vector<double>& residuals,
    const std::string& weight_key,
    double weight_param) {
    double mad = median_of(residuals);
    double scale = std::max(1e-5, 1.4826 * mad);
    std::vector<double> w(residuals.size(), 1.0);
    if (weight_key == "huber") {
        double c = std::max(0.2, weight_param) * scale;
        for (size_t i = 0; i < residuals.size(); ++i) {
            w[i] = (residuals[i] <= c) ? 1.0 : (c / std::max(residuals[i], 1e-12));
        }
        return w;
    }
    double c = std::max(0.5, weight_param >= 2.0 ? weight_param : 4.685) * scale;
    for (size_t i = 0; i < residuals.size(); ++i) {
        double u = residuals[i] / c;
        if (u < 1.0) {
            double t = 1.0 - u * u;
            w[i] = t * t;
        } else {
            w[i] = 0.0;
        }
    }
    return w;
}

}  // namespace

Eigen::Matrix3d project_so3(const Eigen::Matrix3d& R) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d Vt = svd.matrixV().transpose();
    double det = (U * Vt).determinant();
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    D(2, 2) = (det >= 0.0) ? 1.0 : -1.0;
    return U * D * Vt;
}

Eigen::Matrix4d rigid_matrix(const Eigen::Matrix4d& m) {
    if (!m.allFinite()) {
        throw std::invalid_argument("Invalid 4x4 transform matrix");
    }
    Eigen::RowVector4d bottom_expected(0.0, 0.0, 0.0, 1.0);
    if ((m.row(3) - bottom_expected).cwiseAbs().maxCoeff() > 1e-4) {
        throw std::invalid_argument("Invalid 4x4 transform matrix");
    }
    Eigen::Matrix3d R = m.topLeftCorner<3, 3>();
    if ((R.transpose() * R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1e-3 ||
        std::abs(R.determinant() - 1.0) > 1e-3) {
        throw std::invalid_argument("Invalid rotation block in transform matrix");
    }
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.topLeftCorner<3, 3>() = project_so3(R);
    out.topRightCorner<3, 1>() = m.topRightCorner<3, 1>();
    return out;
}

Eigen::Vector4d rot_to_quat_xyzw(const Eigen::Matrix3d& R_in) {
    Eigen::Matrix3d R = project_so3(R_in);
    double tr = R.trace();
    if (tr > 0.0) {
        double s = 2.0 * std::sqrt(std::max(1.0 + tr, 1e-12));
        return Eigen::Vector4d(
            (R(2, 1) - R(1, 2)) / s,
            (R(0, 2) - R(2, 0)) / s,
            (R(1, 0) - R(0, 1)) / s,
            0.25 * s);
    }
    int idx = 0;
    if (R(1, 1) > R(0, 0)) idx = 1;
    if (R(2, 2) > R(idx, idx)) idx = 2;
    if (idx == 0) {
        double s = 2.0 * std::sqrt(std::max(1.0 + R(0, 0) - R(1, 1) - R(2, 2), 1e-12));
        return Eigen::Vector4d(
            0.25 * s,
            (R(0, 1) + R(1, 0)) / s,
            (R(0, 2) + R(2, 0)) / s,
            (R(2, 1) - R(1, 2)) / s);
    }
    if (idx == 1) {
        double s = 2.0 * std::sqrt(std::max(1.0 + R(1, 1) - R(0, 0) - R(2, 2), 1e-12));
        return Eigen::Vector4d(
            (R(0, 1) + R(1, 0)) / s,
            0.25 * s,
            (R(1, 2) + R(2, 1)) / s,
            (R(0, 2) - R(2, 0)) / s);
    }
    double s = 2.0 * std::sqrt(std::max(1.0 + R(2, 2) - R(0, 0) - R(1, 1), 1e-12));
    return Eigen::Vector4d(
        (R(0, 2) + R(2, 0)) / s,
        (R(1, 2) + R(2, 1)) / s,
        0.25 * s,
        (R(1, 0) - R(0, 1)) / s);
}

Eigen::Matrix3d quat_xyzw_to_rot(const Eigen::Vector4d& q) {
    double n = std::max(q.norm(), 1e-12);
    double x = q.x() / n;
    double y = q.y() / n;
    double z = q.z() / n;
    double w = q.w() / n;
    Eigen::Matrix3d R;
    R << 1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
         2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
         2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y);
    return project_so3(R);
}

Eigen::Matrix3d quaternion_to_rot_strict(const Eigen::Vector4d& q) {
    double n = q.norm();
    if (!std::isfinite(n) || std::abs(n - 1.0) > 0.01) {
        throw std::invalid_argument("无效轨迹四元数");
    }
    return quat_xyzw_to_rot(q);
}

std::array<Eigen::Vector3d, 4> aprilgrid_tag_corners_3d(
    int tag_id,
    int rows,
    int cols,
    double tag_size,
    double tag_spacing,
    bool bottom_up,
    bool rev,
    int roll) {
    static const std::array<Eigen::Vector3d, 4> kUnit = {{
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
    }};
    double pitch = tag_size * (1.0 + tag_spacing);
    int raw_row = tag_id / cols;
    int col = tag_id % cols;
    int row = bottom_up ? raw_row : (rows - 1 - raw_row);

    std::array<Eigen::Vector3d, 4> sq = kUnit;
    if (rev) {
        std::reverse(sq.begin(), sq.end());
    }
    int r = ((roll % 4) + 4) % 4;
    if (r != 0) {
        std::array<Eigen::Vector3d, 4> rolled;
        for (int i = 0; i < 4; ++i) {
            rolled[(i + r) % 4] = sq[i];
        }
        sq = rolled;
    }
    Eigen::Vector3d origin(col * pitch, row * pitch, 0.0);
    std::array<Eigen::Vector3d, 4> out;
    for (int i = 0; i < 4; ++i) {
        out[i] = sq[i] * tag_size + origin;
    }
    return out;
}

std::vector<Eigen::Vector2d> apply_matrix_2d(
    const Eigen::Matrix3d& M,
    const std::vector<Eigen::Vector2d>& pts) {
    std::vector<Eigen::Vector2d> out;
    out.reserve(pts.size());
    for (const auto& p : pts) {
        Eigen::Vector3d hom = M * Eigen::Vector3d(p.x(), p.y(), 1.0);
        double w = (std::abs(hom.z()) < 1e-12) ? 1e-12 : hom.z();
        out.emplace_back(hom.x() / w, hom.y() / w);
    }
    return out;
}

void normalize_2d(
    const std::vector<Eigen::Vector2d>& pts,
    Eigen::Matrix3d& T_out,
    std::vector<Eigen::Vector2d>& pts_norm_out) {
    if (pts.empty()) throw std::invalid_argument("Cannot normalize empty 2D point set");
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& p : pts) mean += p;
    mean /= static_cast<double>(pts.size());
    double sum_sq = 0.0;
    for (const auto& p : pts) sum_sq += (p - mean).squaredNorm();
    double dist = std::sqrt(sum_sq / static_cast<double>(pts.size()));
    double s = std::sqrt(2.0) / std::max(dist, 1e-9);
    T_out << s, 0.0, -s * mean.x(),
             0.0, s, -s * mean.y(),
             0.0, 0.0, 1.0;
    pts_norm_out.resize(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        pts_norm_out[i] = T_out.topLeftCorner<2, 2>() * pts[i] + T_out.topRightCorner<2, 1>();
    }
}

Eigen::Matrix3d fit_similarity_2d(
    const std::vector<Eigen::Vector2d>& src,
    const std::vector<Eigen::Vector2d>& dst,
    const std::vector<double>& weights,
    bool allow_reflection) {
    if (src.size() != dst.size() || src.size() < 2) {
        throw std::invalid_argument("相似变换拟合至少需要 2 对点");
    }
    size_t n = src.size();
    std::vector<double> w = sanitized_weights(n, weights);
    double ws = std::accumulate(w.begin(), w.end(), 0.0);
    Eigen::Vector2d mu_s = Eigen::Vector2d::Zero();
    Eigen::Vector2d mu_d = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < n; ++i) {
        mu_s += w[i] * src[i];
        mu_d += w[i] * dst[i];
    }
    mu_s /= ws;
    mu_d /= ws;

    std::vector<Eigen::Vector2d> sc(n), dc(n);
    double denom = 0.0;
    for (size_t i = 0; i < n; ++i) {
        sc[i] = src[i] - mu_s;
        dc[i] = dst[i] - mu_d;
        denom += w[i] * sc[i].squaredNorm();
    }
    if (denom < 1e-12) {
        throw std::invalid_argument("点集退化共点，无法拟合相似变换");
    }

    auto fit_case = [&](bool flip_y) -> std::pair<Eigen::Matrix3d, double> {
        double num_a = 0.0, num_b = 0.0;
        double sy = flip_y ? -1.0 : 1.0;
        for (size_t i = 0; i < n; ++i) {
            double sx_i = sc[i].x();
            double sy_i = flip_y ? -sc[i].y() : sc[i].y();
            num_a += w[i] * (dc[i].x() * sx_i + dc[i].y() * sy_i);
            num_b += w[i] * (dc[i].y() * sx_i - dc[i].x() * sy_i);
        }
        double a = num_a / denom;
        double b = num_b / denom;
        Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
        M(0, 0) = a;  M(0, 1) = -b * sy;
        M(1, 0) = b;  M(1, 1) = a * sy;
        M.topRightCorner<2, 1>() = mu_d - M.topLeftCorner<2, 2>() * mu_s;
        auto pred = apply_matrix_2d(M, src);
        double err = 0.0;
        for (size_t i = 0; i < n; ++i) err += w[i] * (pred[i] - dst[i]).squaredNorm();
        return {M, err};
    };

    auto [M_pos, e_pos] = fit_case(false);
    if (!allow_reflection) return M_pos;
    auto [M_neg, e_neg] = fit_case(true);
    return (e_pos <= e_neg) ? M_pos : M_neg;
}

Eigen::Matrix3d fit_affine_2d(
    const std::vector<Eigen::Vector2d>& src,
    const std::vector<Eigen::Vector2d>& dst,
    const std::vector<double>& weights) {
    if (src.size() != dst.size() || src.size() < 3) {
        throw std::invalid_argument("仿射变换拟合至少需要 3 对点");
    }
    size_t n = src.size();
    std::vector<double> w = sanitized_weights(n, weights);
    Eigen::MatrixXd Aw(static_cast<Eigen::Index>(n), 3);
    Eigen::VectorXd bx(static_cast<Eigen::Index>(n));
    Eigen::VectorXd by(static_cast<Eigen::Index>(n));
    for (size_t i = 0; i < n; ++i) {
        double sw = std::sqrt(w[i]);
        auto r = static_cast<Eigen::Index>(i);
        Aw(r, 0) = src[i].x() * sw;
        Aw(r, 1) = src[i].y() * sw;
        Aw(r, 2) = sw;
        bx(r) = dst[i].x() * sw;
        by(r) = dst[i].y() * sw;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Aw, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3d cx = svd.solve(bx);
    Eigen::Vector3d cy = svd.solve(by);
    Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
    M.row(0) = cx.transpose();
    M.row(1) = cy.transpose();
    return M;
}

Eigen::Matrix3d fit_homography_2d(
    const std::vector<Eigen::Vector2d>& src,
    const std::vector<Eigen::Vector2d>& dst,
    const std::vector<double>& weights) {
    if (src.size() != dst.size() || src.size() < 4) {
        throw std::invalid_argument("透视变换拟合至少需要 4 对点");
    }
    size_t n = src.size();
    std::vector<double> w = sanitized_weights(n, weights);
    Eigen::Matrix3d Ts, Td;
    std::vector<Eigen::Vector2d> sn, dn;
    normalize_2d(src, Ts, sn);
    normalize_2d(dst, Td, dn);

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(2 * n), 9);
    for (size_t i = 0; i < n; ++i) {
        double wi = std::sqrt(w[i]);
        double u = sn[i].x(), v = sn[i].y();
        double x = dn[i].x(), y = dn[i].y();
        auto r0 = static_cast<Eigen::Index>(2 * i);
        auto r1 = r0 + 1;
        A(r0, 3) = -u * wi; A(r0, 4) = -v * wi; A(r0, 5) = -wi;
        A(r0, 6) = y * u * wi; A(r0, 7) = y * v * wi; A(r0, 8) = y * wi;
        A(r1, 0) = u * wi; A(r1, 1) = v * wi; A(r1, 2) = wi;
        A(r1, 6) = -x * u * wi; A(r1, 7) = -x * v * wi; A(r1, 8) = -x * wi;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::VectorXd h = svd.matrixV().col(8);
    Eigen::Matrix3d Hn;
    Hn << h(0), h(1), h(2),
          h(3), h(4), h(5),
          h(6), h(7), h(8);
    Eigen::Matrix3d H = Td.inverse() * Hn * Ts;
    if (std::abs(H(2, 2)) > 1e-12) {
        H /= H(2, 2);
    }
    return H;
}

std::string normalize_dof_2d(const std::string& dof) {
    std::string d = trim_lower(dof);
    if (d == "similarity" || d == "4" || d == "4dof" || d == "相似" || d == "相似性变换") {
        return "similarity";
    }
    if (d == "affine" || d == "6" || d == "6dof" || d == "仿射" || d == "仿射变换") {
        return "affine";
    }
    if (d == "homography" || d == "perspective" || d == "8" || d == "8dof" || d == "透视" || d == "透视变换") {
        return "homography";
    }
    throw std::invalid_argument("不支持的自由度模型: " + dof);
}

std::string normalize_weight_2d(const std::string& weight_func) {
    std::string w = trim_lower(weight_func);
    if (w == "least_squares" || w == "ls" || w == "ols" || w == "最小二乘" || w == "最小二乘法") {
        return "least_squares";
    }
    if (w == "huber") return "huber";
    if (w == "tukey") return "tukey";
    if (w == "ransac") return "ransac";
    throw std::invalid_argument("不支持的权重函数: " + weight_func);
}

Robust2DFitResult fit_2d_robust(
    const std::vector<Eigen::Vector2d>& src,
    const std::vector<Eigen::Vector2d>& dst,
    const std::string& dof,
    const std::string& weight_func,
    double weight_param,
    double ransac_thresh,
    double sample_ratio,
    bool allow_reflection) {
    if (src.size() != dst.size()) {
        throw std::invalid_argument("源点集与目标点集数量必须一致");
    }
    size_t n = src.size();
    std::string dof_key = normalize_dof_2d(dof);
    std::string w_key = normalize_weight_2d(weight_func);
    size_t min_pts = (dof_key == "similarity") ? 2 : (dof_key == "affine" ? 3 : 4);
    if (n < min_pts) {
        throw std::invalid_argument(dof_key + " 模型至少需要 " + std::to_string(min_pts) +
                                    " 对点（当前仅 " + std::to_string(n) + " 对）");
    }

    auto fit_raw = [&](const std::vector<Eigen::Vector2d>& s_arr,
                       const std::vector<Eigen::Vector2d>& d_arr,
                       const std::vector<double>& w_arr = {}) -> Eigen::Matrix3d {
        if (dof_key == "similarity") return fit_similarity_2d(s_arr, d_arr, w_arr, allow_reflection);
        if (dof_key == "affine") return fit_affine_2d(s_arr, d_arr, w_arr);
        return fit_homography_2d(s_arr, d_arr, w_arr);
    };

    if (w_key == "ransac") {
        std::mt19937_64 rng(2026);
        double thresh = std::max(1e-4, ransac_thresh);
        double ratio = std::max(0.2, std::min(1.0, sample_ratio));
        int n_trials = std::min(400, std::max(50, static_cast<int>(std::llround(n * 15.0 * ratio))));
        std::vector<size_t> all_idx(n);
        std::iota(all_idx.begin(), all_idx.end(), 0);

        std::vector<bool> best_inliers;
        double best_score = 1e18;
        bool found_any = false;
        for (int trial = 0; trial < n_trials; ++trial) {
            for (size_t k = 0; k < min_pts; ++k) {
                std::uniform_int_distribution<size_t> dist(k, n - 1);
                std::swap(all_idx[k], all_idx[dist(rng)]);
            }
            std::vector<Eigen::Vector2d> s_sub(min_pts), d_sub(min_pts);
            for (size_t k = 0; k < min_pts; ++k) {
                s_sub[k] = src[all_idx[k]];
                d_sub[k] = dst[all_idx[k]];
            }
            Eigen::Matrix3d M_cand;
            try {
                M_cand = fit_raw(s_sub, d_sub);
            } catch (...) {
                continue;
            }
            auto pred = apply_matrix_2d(M_cand, src);
            std::vector<bool> inl(n, false);
            int n_inl = 0;
            double loss = 0.0;
            for (size_t i = 0; i < n; ++i) {
                double err = (pred[i] - dst[i]).norm();
                if (err <= thresh) {
                    inl[i] = true;
                    ++n_inl;
                }
                double c_err = std::min(err, thresh);
                loss += c_err * c_err;
            }
            double score = -static_cast<double>(n_inl) * 1e6 + loss;
            if (score < best_score) {
                best_score = score;
                best_inliers = std::move(inl);
                found_any = true;
            }
        }
        int inl_cnt = 0;
        if (found_any) {
            for (bool b : best_inliers) {
                if (b) ++inl_cnt;
            }
        }
        std::vector<bool> inliers = (found_any && inl_cnt >= static_cast<int>(min_pts))
            ? best_inliers
            : std::vector<bool>(n, true);
        std::vector<Eigen::Vector2d> s_inl, d_inl;
        for (size_t i = 0; i < n; ++i) {
            if (inliers[i]) {
                s_inl.push_back(src[i]);
                d_inl.push_back(dst[i]);
            }
        }
        return {fit_raw(s_inl, d_inl), inliers, dof_key, w_key};
    }

    Eigen::Matrix3d M = fit_raw(src, dst);
    if (w_key == "least_squares") {
        return {M, std::vector<bool>(n, true), dof_key, w_key};
    }
    std::vector<double> weights(n, 1.0);
    for (int iter = 0; iter < 20; ++iter) {
        auto pred = apply_matrix_2d(M, src);
        std::vector<double> res(n);
        for (size_t i = 0; i < n; ++i) res[i] = (pred[i] - dst[i]).norm();
        weights = compute_irls_weights_2d(res, w_key, weight_param);
        int active = 0;
        for (double wi : weights) {
            if (wi > 1e-6) ++active;
        }
        if (active < static_cast<int>(min_pts)) break;
        Eigen::Matrix3d M_new = fit_raw(src, dst, weights);
        double diff = (M_new - M).cwiseAbs().maxCoeff();
        M = M_new;
        if (diff < 1e-9) break;
    }
    std::vector<bool> inliers(n);
    for (size_t i = 0; i < n; ++i) inliers[i] = (weights[i] > 1e-3);
    return {M, inliers, dof_key, w_key};
}

}  // namespace slam_common
