#include "vision_calibration/planar_and_distortion.hpp"

#include "slam_common/geometry.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace vision_calibration {
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

slam_common::JsonValue mat3_to_json(const Eigen::Matrix3d& M) {
    slam_common::JsonValue arr = slam_common::JsonValue::array();
    for (int r = 0; r < 3; ++r) {
        arr.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{M(r, 0), M(r, 1), M(r, 2)}));
    }
    return arr;
}

Eigen::Matrix3d json_to_mat3(const slam_common::JsonValue& j) {
    Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
    const auto& rows = j.as_array();
    if (rows.size() != 3) throw std::invalid_argument("Expected 3x3 matrix in .iwcal");
    for (int r = 0; r < 3; ++r) {
        const auto& cols = rows[static_cast<size_t>(r)].as_array();
        if (cols.size() != 3) throw std::invalid_argument("Expected 3 columns in 3x3 matrix");
        for (int c = 0; c < 3; ++c) M(r, c) = cols[static_cast<size_t>(c)].as_double();
    }
    return M;
}

std::string normalize_distortion_type(const std::string& dtype) {
    std::string t = trim_lower(dtype);
    if (t == "radial" || t == "径向" || t == "径向畸变") return "radial";
    if (t == "perspective" || t == "透视" || t == "透视畸变") return "perspective";
    if (t == "radial_perspective" || t == "both" || t == "径向透视" || t == "径向透视畸变") {
        return "radial_perspective";
    }
    throw std::invalid_argument("不支持的畸变类型: " + dtype);
}

std::pair<Eigen::Matrix3d, double> fit_perspective_rectification_at_center(
    const std::vector<Eigen::Vector2d>& img_pts_2d,
    const std::vector<Eigen::Vector2d>& obj_pts_2d,
    const Eigen::Vector2d& center_point,
    const std::string& weight_func) {
    auto fit = slam_common::fit_2d_robust(
        img_pts_2d, obj_pts_2d, "homography", weight_func, 1.345, 1.5, 0.8, true);
    const Eigen::Matrix3d& H_iw = fit.matrix;
    double cx = center_point.x(), cy = center_point.y();

    auto pts = slam_common::apply_matrix_2d(
        H_iw, {Eigen::Vector2d(cx, cy), Eigen::Vector2d(cx + 1.0, cy), Eigen::Vector2d(cx, cy + 1.0)});
    Eigen::Vector2d wc = pts[0];
    Eigen::Vector2d w_du = pts[1] - wc;
    Eigen::Vector2d w_dv = pts[2] - wc;

    Eigen::Matrix2d J_iw;
    J_iw.col(0) = w_du;
    J_iw.col(1) = w_dv;
    if (std::abs(J_iw.determinant()) < 1e-12) {
        throw std::invalid_argument("透视单应矩阵在中心点附近奇异，无法构造去畸变变换");
    }
    Eigen::Matrix2d J_wi = J_iw.inverse();
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(J_wi, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d Vt = svd.matrixV().transpose();
    Eigen::Matrix2d R2 = U * Vt;
    if (R2.determinant() < 0.0) {
        U.col(1) *= -1.0;
        R2 = U * Vt;
    }
    Eigen::Vector2d sigmas = svd.singularValues();
    double scale_px_per_unit = std::sqrt(std::max(sigmas(0) * sigmas(1), 1e-12));

    Eigen::Matrix3d T_c_out;
    T_c_out << 1.0, 0.0, cx,
               0.0, 1.0, cy,
               0.0, 0.0, 1.0;
    Eigen::Matrix3d S_R = Eigen::Matrix3d::Identity();
    S_R.topLeftCorner<2, 2>() = scale_px_per_unit * R2;
    Eigen::Matrix3d T_w_shift;
    T_w_shift << 1.0, 0.0, -wc.x(),
                 0.0, 1.0, -wc.y(),
                 0.0, 0.0, 1.0;
    Eigen::Matrix3d H_persp = T_c_out * S_R * T_w_shift * H_iw;
    if (std::abs(H_persp(2, 2)) > 1e-12) {
        H_persp /= H_persp(2, 2);
    }

    auto rect_pts = slam_common::apply_matrix_2d(H_persp, img_pts_2d);
    Eigen::Matrix3d M_ideal = slam_common::fit_similarity_2d(obj_pts_2d, rect_pts, {}, true);
    auto ideal_pred = slam_common::apply_matrix_2d(M_ideal, obj_pts_2d);
    double sq = 0.0;
    for (size_t i = 0; i < rect_pts.size(); ++i) {
        sq += (ideal_pred[i] - rect_pts[i]).squaredNorm();
    }
    double rms = std::sqrt(sq / static_cast<double>(std::max<size_t>(1, rect_pts.size())));
    return {H_persp, rms};
}

}  // namespace

slam_common::JsonValue PlanarCalib2D::to_json() const {
    slam_common::JsonValue j = slam_common::JsonValue::object();
    j["tool"] = tool;
    j["dof"] = dof;
    j["weight_func"] = weight_func;
    j["right_handed"] = right_handed;
    j["origin_mm"] = slam_common::JsonValue(slam_common::JsonValue::Array{origin_mm.x(), origin_mm.y()});
    j["angle_deg"] = angle_deg;
    j["matrix"] = mat3_to_json(matrix);
    j["inv_matrix"] = mat3_to_json(inv_matrix);
    j["rms_mm"] = rms_mm;
    j["rms_px"] = rms_px;
    j["pixel_size_mm"] = pixel_size_mm;
    j["scale_x_mm_px"] = scale_x_mm_px;
    j["scale_y_mm_px"] = scale_y_mm_px;
    j["inlier_count"] = static_cast<int64_t>(inlier_count);
    j["total_points"] = static_cast<int64_t>(total_points);
    return j;
}

PlanarCalib2D PlanarCalib2D::from_json(const slam_common::JsonValue& j) {
    PlanarCalib2D cal;
    if (j.contains("tool") && j.get("tool").is_string()) cal.tool = j.get("tool").as_string();
    if (j.contains("dof") && j.get("dof").is_string()) cal.dof = j.get("dof").as_string();
    if (j.contains("weight_func") && j.get("weight_func").is_string()) cal.weight_func = j.get("weight_func").as_string();
    if (j.contains("matrix")) cal.matrix = json_to_mat3(j.get("matrix"));
    if (j.contains("inv_matrix")) cal.inv_matrix = json_to_mat3(j.get("inv_matrix"));
    else cal.inv_matrix = cal.matrix.inverse();
    if (j.contains("pixel_size_mm")) cal.pixel_size_mm = j.get("pixel_size_mm").as_double();
    if (j.contains("rms_mm")) cal.rms_mm = j.get("rms_mm").as_double();
    if (j.contains("rms_px")) cal.rms_px = j.get("rms_px").as_double();
    return cal;
}

slam_common::JsonValue DistortionModel::to_json() const {
    slam_common::JsonValue j = slam_common::JsonValue::object();
    j["tool"] = tool;
    j["distortion_type"] = distortion_type;
    j["image_size"] = slam_common::JsonValue(slam_common::JsonValue::Array{
        static_cast<int64_t>(image_size[0]), static_cast<int64_t>(image_size[1])});
    j["center_point"] = slam_common::JsonValue(slam_common::JsonValue::Array{center_point.x(), center_point.y()});
    j["K"] = mat3_to_json(K);
    slam_common::JsonValue d_arr = slam_common::JsonValue::array();
    for (double v : D) d_arr.push_back(v);
    j["D"] = d_arr;
    j["H_persp"] = mat3_to_json(H_persp);
    j["rms_px"] = rms_px;
    return j;
}

DistortionModel DistortionModel::from_json(const slam_common::JsonValue& j) {
    DistortionModel m;
    if (j.contains("distortion_type")) m.distortion_type = j.get("distortion_type").as_string();
    if (j.contains("image_size")) {
        m.image_size = {static_cast<int>(j.get("image_size").as_array()[0].as_int()),
                        static_cast<int>(j.get("image_size").as_array()[1].as_int())};
    }
    if (j.contains("center_point")) {
        m.center_point = Eigen::Vector2d(j.get("center_point").as_array()[0].as_double(),
                                         j.get("center_point").as_array()[1].as_double());
    }
    if (j.contains("K")) m.K = json_to_mat3(j.get("K"));
    if (j.contains("D")) {
        const auto& da = j.get("D").as_array();
        for (size_t i = 0; i < std::min<size_t>(5, da.size()); ++i) m.D[i] = da[i].as_double();
    }
    if (j.contains("H_persp")) m.H_persp = json_to_mat3(j.get("H_persp"));
    if (j.contains("rms_px")) m.rms_px = j.get("rms_px").as_double();
    return m;
}

slam_common::JsonValue DualCamMapping2D::to_json() const {
    slam_common::JsonValue j = slam_common::JsonValue::object();
    j["tool"] = tool;
    j["dof"] = dof;
    j["weight_func"] = weight_func;
    j["matrix"] = mat3_to_json(matrix);
    j["inv_matrix"] = mat3_to_json(inv_matrix);
    j["scale_x"] = scale_x;
    j["scale_y"] = scale_y;
    j["rotation_deg"] = rotation_deg;
    j["rms_px"] = rms_px;
    j["max_err_px"] = max_err_px;
    j["inlier_count"] = static_cast<int64_t>(inlier_count);
    j["total_points"] = static_cast<int64_t>(total_points);
    return j;
}

PlanarCalib2D calibrate_planar_board_2d(
    const std::vector<Eigen::Vector2d>& img_pts,
    const std::vector<Eigen::Vector2d>& obj_pts_mm,
    double origin_x,
    double origin_y,
    double angle_deg,
    bool right_handed,
    const std::string& dof,
    const std::string& weight_func,
    double weight_param,
    double ransac_thresh) {
    if (img_pts.size() != obj_pts_mm.size()) {
        throw std::invalid_argument("图像点与物理平面点数量必须一致");
    }
    double rad = angle_deg * kPi / 180.0;
    Eigen::Matrix2d R2;
    R2 << std::cos(rad), -std::sin(rad),
          std::sin(rad),  std::cos(rad);
    Eigen::Vector2d origin(origin_x, origin_y);
    std::vector<Eigen::Vector2d> dst(obj_pts_mm.size());
    for (size_t i = 0; i < obj_pts_mm.size(); ++i) {
        Eigen::Vector2d p = obj_pts_mm[i];
        if (!right_handed) p.y() = -p.y();
        dst[i] = R2 * p + origin;
    }

    auto fit = slam_common::fit_2d_robust(
        img_pts, dst, dof, weight_func, weight_param, ransac_thresh, 0.8, true);
    Eigen::Matrix3d inv_M = fit.matrix.inverse();

    auto pred_mm = slam_common::apply_matrix_2d(fit.matrix, img_pts);
    auto pred_px = slam_common::apply_matrix_2d(inv_M, dst);
    bool any_inl = std::any_of(fit.inliers.begin(), fit.inliers.end(), [](bool b) { return b; });
    double sum_mm2 = 0.0, sum_px2 = 0.0;
    int eval_cnt = 0;
    for (size_t i = 0; i < img_pts.size(); ++i) {
        if (!any_inl || fit.inliers[i]) {
            sum_mm2 += (pred_mm[i] - dst[i]).squaredNorm();
            sum_px2 += (pred_px[i] - img_pts[i]).squaredNorm();
            ++eval_cnt;
        }
    }

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto& p : img_pts) centroid += p;
    centroid /= static_cast<double>(std::max<size_t>(1, img_pts.size()));
    auto jac = slam_common::apply_matrix_2d(
        fit.matrix,
        {centroid, Eigen::Vector2d(centroid.x() + 1.0, centroid.y()), Eigen::Vector2d(centroid.x(), centroid.y() + 1.0)});
    double sx = (jac[1] - jac[0]).norm();
    double sy = (jac[2] - jac[0]).norm();

    PlanarCalib2D out;
    out.dof = fit.dof_key;
    out.weight_func = fit.weight_key;
    out.right_handed = right_handed;
    out.origin_mm = origin;
    out.angle_deg = angle_deg;
    out.matrix = fit.matrix;
    out.inv_matrix = inv_M;
    out.rms_mm = std::sqrt(sum_mm2 / static_cast<double>(std::max(1, eval_cnt)));
    out.rms_px = std::sqrt(sum_px2 / static_cast<double>(std::max(1, eval_cnt)));
    out.pixel_size_mm = std::sqrt(std::max(sx * sy, 1e-12));
    out.scale_x_mm_px = sx;
    out.scale_y_mm_px = sy;
    out.inlier_count = eval_cnt;
    out.total_points = static_cast<int>(img_pts.size());
    return out;
}

Eigen::Vector2d convert_pixel_to_world_2d(const PlanarCalib2D& cal, double u, double v) {
    return slam_common::apply_matrix_2d(cal.matrix, {Eigen::Vector2d(u, v)})[0];
}

Eigen::Vector2d convert_pixel_to_world_2d(const std::filesystem::path& iwcal_path, double u, double v) {
    return convert_pixel_to_world_2d(PlanarCalib2D::from_json(load_iwcal_file(iwcal_path)), u, v);
}

Eigen::Vector2d convert_pixel_to_world_2d(const slam_common::JsonValue& cal_json, double u, double v) {
    return convert_pixel_to_world_2d(PlanarCalib2D::from_json(cal_json), u, v);
}

double convert_pixel_length_to_mm(
    const PlanarCalib2D& cal,
    double pixel_length,
    std::optional<double> u,
    std::optional<double> v) {
    if (u.has_value() && v.has_value()) {
        auto pts = slam_common::apply_matrix_2d(
            cal.matrix,
            {Eigen::Vector2d(*u, *v), Eigen::Vector2d(*u + 1.0, *v), Eigen::Vector2d(*u, *v + 1.0)});
        double local_scale = std::sqrt(std::max((pts[1] - pts[0]).norm() * (pts[2] - pts[0]).norm(), 1e-12));
        return pixel_length * local_scale;
    }
    return pixel_length * cal.pixel_size_mm;
}

double convert_pixel_length_to_mm(
    const std::filesystem::path& iwcal_path,
    double pixel_length,
    std::optional<double> u,
    std::optional<double> v) {
    return convert_pixel_length_to_mm(PlanarCalib2D::from_json(load_iwcal_file(iwcal_path)), pixel_length, u, v);
}

double convert_pixel_length_to_mm(
    const slam_common::JsonValue& cal_json,
    double pixel_length,
    std::optional<double> u,
    std::optional<double> v) {
    return convert_pixel_length_to_mm(PlanarCalib2D::from_json(cal_json), pixel_length, u, v);
}

DistortionModel calibrate_image_distortion(
    const std::vector<std::vector<cv::Point3f>>& obj_pts_list,
    const std::vector<std::vector<cv::Point2f>>& img_pts_list,
    cv::Size image_size,
    const std::string& distortion_type,
    std::optional<Eigen::Vector2d> center_point,
    const std::string& weight_func) {
    if (obj_pts_list.empty() || obj_pts_list.size() != img_pts_list.size()) {
        throw std::invalid_argument("畸变标定至少需要 1 帧有效的靶标角点数据");
    }
    int w = image_size.width, h = image_size.height;
    Eigen::Vector2d cp = center_point.has_value() ? *center_point : Eigen::Vector2d(w * 0.5, h * 0.5);
    std::string d_type = normalize_distortion_type(distortion_type);

    Eigen::Matrix3d K_mat;
    K_mat << std::max(w, h) * 1.1, 0.0, cp.x(),
             0.0, std::max(w, h) * 1.1, cp.y(),
             0.0, 0.0, 1.0;
    std::array<double, 5> D_vec{0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Matrix3d H_persp = Eigen::Matrix3d::Identity();
    double rms_px = 0.0;

    cv::Mat K_cv(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) K_cv.at<double>(r, c) = K_mat(r, c);
    }
    cv::Mat D_cv = cv::Mat::zeros(5, 1, CV_64F);

    if (d_type == "radial" || d_type == "radial_perspective") {
        int flags = cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K3;
        if (obj_pts_list.size() < 3) {
            flags |= cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_PRINCIPAL_POINT;
        } else if (center_point.has_value()) {
            flags |= cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_PRINCIPAL_POINT;
        } else {
            K_cv = cv::Mat::eye(3, 3, CV_64F);
        }
        std::vector<cv::Mat> rvecs, tvecs;
        rms_px = cv::calibrateCamera(obj_pts_list, img_pts_list, image_size, K_cv, D_cv, rvecs, tvecs, flags);
        K_cv.convertTo(K_cv, CV_64F);
        D_cv.convertTo(D_cv, CV_64F);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) K_mat(r, c) = K_cv.at<double>(r, c);
        }
        cv::Mat flat = D_cv.reshape(1, 1);
        for (int i = 0; i < std::min(5, flat.cols); ++i) D_vec[static_cast<size_t>(i)] = flat.at<double>(0, i);
    }

    if (d_type == "perspective" || d_type == "radial_perspective") {
        std::vector<cv::Point2f> und_cv;
        if (d_type == "radial_perspective") {
            cv::undistortPoints(img_pts_list[0], und_cv, K_cv, D_cv, cv::noArray(), K_cv);
        } else {
            und_cv = img_pts_list[0];
        }
        std::vector<Eigen::Vector2d> und_eig, obj_eig;
        und_eig.reserve(und_cv.size());
        obj_eig.reserve(obj_pts_list[0].size());
        for (const auto& p : und_cv) und_eig.emplace_back(p.x, p.y);
        for (const auto& p : obj_pts_list[0]) obj_eig.emplace_back(p.x, p.y);
        auto [Hp, p_rms] = fit_perspective_rectification_at_center(und_eig, obj_eig, cp, weight_func);
        H_persp = Hp;
        rms_px = (d_type == "perspective") ? p_rms : std::sqrt(0.5 * (rms_px * rms_px + p_rms * p_rms));
    }

    DistortionModel out;
    out.distortion_type = d_type;
    out.image_size = {w, h};
    out.center_point = cp;
    out.K = K_mat;
    out.D = D_vec;
    out.H_persp = H_persp;
    out.rms_px = rms_px;
    return out;
}

cv::Mat undistort_image_with_model(const cv::Mat& image, const DistortionModel& cal) {
    if (image.empty()) throw std::invalid_argument("待校正图像为空");
    cv::Mat out = image.clone();
    if (cal.distortion_type == "radial" || cal.distortion_type == "radial_perspective") {
        cv::Mat K(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) K.at<double>(r, c) = cal.K(r, c);
        }
        cv::Mat D(5, 1, CV_64F);
        for (int i = 0; i < 5; ++i) D.at<double>(i, 0) = cal.D[static_cast<size_t>(i)];
        cv::Mat tmp;
        cv::undistort(out, tmp, K, D, K);
        out = tmp;
    }
    if (cal.distortion_type == "perspective" || cal.distortion_type == "radial_perspective") {
        cv::Mat H(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) H.at<double>(r, c) = cal.H_persp(r, c);
        }
        cv::Mat tmp;
        cv::warpPerspective(out, tmp, H, out.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        out = tmp;
    }
    return out;
}

DualCamMapping2D calibrate_dual_camera_mapping_2d(
    const std::vector<Eigen::Vector2d>& cam1_pts,
    const std::vector<Eigen::Vector2d>& cam2_pts,
    const std::string& dof,
    const std::string& weight_func,
    double weight_param,
    double ransac_thresh) {
    if (cam1_pts.size() != cam2_pts.size()) {
        throw std::invalid_argument("两台相机的特征点数量必须一致");
    }
    auto fit = slam_common::fit_2d_robust(
        cam1_pts, cam2_pts, dof, weight_func, weight_param, ransac_thresh, 0.8, true);
    Eigen::Matrix3d inv_M = fit.matrix.inverse();
    auto pred = slam_common::apply_matrix_2d(fit.matrix, cam1_pts);
    bool any_inl = std::any_of(fit.inliers.begin(), fit.inliers.end(), [](bool b) { return b; });
    double sum2 = 0.0, max_e = 0.0;
    int cnt = 0;
    for (size_t i = 0; i < cam1_pts.size(); ++i) {
        if (!any_inl || fit.inliers[i]) {
            double e = (pred[i] - cam2_pts[i]).norm();
            sum2 += e * e;
            max_e = std::max(max_e, e);
            ++cnt;
        }
    }
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto& p : cam1_pts) centroid += p;
    centroid /= static_cast<double>(std::max<size_t>(1, cam1_pts.size()));
    auto jac = slam_common::apply_matrix_2d(
        fit.matrix,
        {centroid, Eigen::Vector2d(centroid.x() + 1.0, centroid.y()), Eigen::Vector2d(centroid.x(), centroid.y() + 1.0)});
    Eigen::Vector2d du = jac[1] - jac[0];
    Eigen::Vector2d dv = jac[2] - jac[0];

    DualCamMapping2D out;
    out.dof = fit.dof_key;
    out.weight_func = fit.weight_key;
    out.matrix = fit.matrix;
    out.inv_matrix = inv_M;
    out.scale_x = du.norm();
    out.scale_y = dv.norm();
    out.rotation_deg = std::atan2(du.y(), du.x()) * 180.0 / kPi;
    out.rms_px = std::sqrt(sum2 / static_cast<double>(std::max(1, cnt)));
    out.max_err_px = max_e;
    out.inlier_count = cnt;
    out.total_points = static_cast<int>(cam1_pts.size());
    return out;
}

std::filesystem::path save_iwcal_file(const std::filesystem::path& path, const slam_common::JsonValue& data) {
    std::filesystem::path p = std::filesystem::weakly_canonical(path);
    slam_common::save_json(p, data, true);
    return p;
}

slam_common::JsonValue load_iwcal_file(const std::filesystem::path& path) {
    return slam_common::read_json_file(std::filesystem::weakly_canonical(path));
}

}  // namespace vision_calibration
