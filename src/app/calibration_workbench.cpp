#include "calibration_workbench.hpp"

#include "calibration_targets/detector.hpp"
#include "calibration_targets/generator.hpp"
#include "camera_imu_calibration/engine.hpp"
#include "handeye_calibration/handeye_3d.hpp"
#include "handeye_calibration/n_point_2d.hpp"
#include "slam_common/geometry.hpp"
#include "umi_slam/runtime.hpp"
#include "vision_calibration/planar_and_distortion.hpp"

#include <imgui.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace slam_app {
namespace {

constexpr const char* kCamModeLabels[] = {
    "双目相机 (Stereo)", "单目相机 (Mono)", "多目相机 (Multi-Cam)", "不选相机 (仅 IMU)"
};
constexpr const char* kCamModeKeys[] = {"stereo", "mono", "multi", "none"};

constexpr const char* kTargetLabels[] = {
    "AprilGrid (36h11)", "棋盘格 (Checkerboard)", "对称圆点 (Circles)", "非对称圆点 (Acircles)"
};
constexpr const char* kTargetKeys[] = {"aprilgrid", "chessboard", "circles", "acircles"};

constexpr const char* kPaperLabels[] = {"A3", "A4", "A2", "Letter", "Custom"};
constexpr const char* kOrientLabels[] = {"横向 (Landscape)", "纵向 (Portrait)"};
constexpr const char* kOrientKeys[] = {"landscape", "portrait"};

constexpr const char* kInstallLabels[] = {"固定上相机 (static_up)", "固定下相机 (static_down)", "移动相机 (Eye-in-Hand)"};
constexpr const char* kInstallKeys[] = {"static_up", "static_down", "moving"};

constexpr const char* kDofLabels[] = {"4 自由度 (相似)", "6 自由度 (仿射)", "8 自由度 (透视)"};
constexpr const char* kDofKeys[] = {"similarity", "affine", "homography"};

constexpr const char* kWeightLabels[] = {"最小二乘", "Huber", "Tukey", "RANSAC"};
constexpr const char* kWeightKeys[] = {"least_squares", "huber", "tukey", "ransac"};

constexpr const char* kHeModeLabels[] = {"眼在手上 (Eye-in-Hand · AX=XB)", "眼在手外 (Eye-to-Hand · AX=ZB)"};
constexpr const char* kHeModeKeys[] = {"eye_in_hand", "eye_to_hand"};

constexpr const char* kHeMethodLabels[] = {"Tsai-Lenz", "Park-Martin", "Horaud", "Andreff", "Daniilidis"};
constexpr const char* kHeMethodKeys[] = {"tsai", "park", "horaud", "andreff", "daniilidis"};

constexpr const char* kDistLabels[] = {"径向 + 透视 (两者)", "仅镜头径向", "仅透视畸变"};
constexpr const char* kDistKeys[] = {"radial_perspective", "radial", "perspective"};

cv::Mat make_stereo_preview_mat(
    const cv::Mat& left_gray,
    const cv::Mat& right_gray,
    const std::vector<cv::Point2f>& img0,
    const std::vector<cv::Point2f>& img1) {
    cv::Mat vis0, vis1;
    if (left_gray.channels() == 1) cv::cvtColor(left_gray, vis0, cv::COLOR_GRAY2BGR);
    else vis0 = left_gray.clone();
    if (right_gray.channels() == 1) cv::cvtColor(right_gray, vis1, cv::COLOR_GRAY2BGR);
    else vis1 = right_gray.clone();

    if (!img0.empty()) {
        size_t step = std::max<size_t>(1, img0.size() / 160);
        for (size_t i = 0; i < img0.size(); i += step) {
            cv::circle(vis0, cv::Point(static_cast<int>(std::llround(img0[i].x)), static_cast<int>(std::llround(img0[i].y))), 4, cv::Scalar(94, 197, 34), -1);
        }
    }
    if (!img1.empty()) {
        size_t step = std::max<size_t>(1, img1.size() / 160);
        for (size_t i = 0; i < img1.size(); i += step) {
            cv::circle(vis1, cv::Point(static_cast<int>(std::llround(img1[i].x)), static_cast<int>(std::llround(img1[i].y))), 4, cv::Scalar(248, 189, 56), -1);
        }
    }
    if (img1.empty() && right_gray.data == left_gray.data) return vis0;
    cv::Mat combined;
    cv::hconcat(vis0, vis1, combined);
    return combined;
}

std::vector<std::vector<double>> parse_float_rows(const std::string& text, size_t expected_cols) {
    std::vector<std::vector<double>> rows;
    std::istringstream ss(text);
    std::string line;
    while (std::getline(ss, line)) {
        size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos || line[first] == '#') continue;
        for (char& ch : line) {
            if (ch == ',' || ch == ';' || ch == '\t') ch = ' ';
        }
        std::istringstream rs(line);
        std::vector<double> vals;
        double v = 0.0;
        while (rs >> v) vals.push_back(v);
        if (vals.size() < expected_cols) {
            throw std::invalid_argument("数据行列数不足（需要至少 " + std::to_string(expected_cols) + " 列）: " + line);
        }
        rows.push_back(std::move(vals));
    }
    return rows;
}

std::vector<Eigen::Vector2d> parse_semicolon_xy(const std::string& text) {
    std::string norm = text;
    for (char& c : norm) {
        if (c == ';') c = '\n';
    }
    auto rows = parse_float_rows(norm, 2);
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(rows.size());
    for (const auto& r : rows) pts.emplace_back(r[0], r[1]);
    return pts;
}

std::filesystem::path find_repo_asset(const std::filesystem::path& rel) {
    std::filesystem::path app_dir = umi_slam::current_executable_dir();
    std::filesystem::path cwd = std::filesystem::current_path();
    for (const auto& base : {app_dir, app_dir.parent_path(), cwd, cwd.parent_path(), cwd.parent_path().parent_path()}) {
        if (!base.empty() && std::filesystem::exists(base / rel)) return base / rel;
    }
    return cwd / rel;
}

void render_scaled_texture(const GlTexture& tex, float max_w, float max_h) {
    if (!tex.valid()) return;
    float sx = max_w / static_cast<float>(tex.width());
    float sy = max_h / static_cast<float>(tex.height());
    float s = std::min(sx, sy);
    if (s <= 0.0f) return;
    ImGui::Image(tex.imgui_id(), ImVec2(static_cast<float>(tex.width()) * s, static_cast<float>(tex.height()) * s));
}

}  // namespace

CalibrationPane::CalibrationPane(ApplyCallback on_apply, PopoutCallback on_popout)
    : on_apply_(std::move(on_apply)), on_popout_(std::move(on_popout)) {
    std::time_t now_t = std::time(nullptr);
    std::tm tm_now{};
#ifdef _WIN32
    localtime_s(&tm_now, &now_t);
#else
    localtime_r(&now_t, &tm_now);
#endif
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "calib_%Y%m%d_%H%M%S", &tm_now);
    output_dir_ = (user_home_dir() / "SLAM_Calibrations" / stamp).string();
    fill_npoint_demo();
}

CalibrationPane::~CalibrationPane() {
    slam_cancel_.store(true);
    if (slam_worker_.joinable()) slam_worker_.join();
    if (vision_worker_.joinable()) vision_worker_.join();
}

void CalibrationPane::show_message(const std::string& title, const std::string& body) {
    modal_title_ = title;
    modal_body_ = body;
    open_msg_modal_ = true;
}

void CalibrationPane::render(const char* id_prefix) {
    ImGui::PushID(id_prefix);

    ImGui::TextUnformatted("全功能免 ROS 标定工作台 · 单/双/多目 + IMU · 标定板生成打印 · N点/3D手眼 · 2D平面/畸变/映射");
    if (on_popout_) {
        ImGui::SameLine();
        if (ImGui::Button("弹出独立标定窗口")) on_popout_();
    }
    ImGui::Separator();

    if (ImGui::BeginTabBar("##calib_tabs")) {
        if (ImGui::BeginTabItem("多模态相机 / IMU 标定")) {
            render_slam_tab();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("标定板生成与打印")) {
            render_board_tab();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("N 点标定 & 3D 手眼")) {
            render_handeye_tab();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("2D 平面 / 畸变 / 相机映射")) {
            render_vision_tab();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    render_popups();
    ImGui::PopID();
}

void CalibrationPane::render_popups() {
    if (open_msg_modal_) {
        ImGui::OpenPopup("提示##calib_msg");
        open_msg_modal_ = false;
    }
    if (ImGui::BeginPopupModal("提示##calib_msg", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted(modal_title_.c_str());
        ImGui::Separator();
        ImGui::TextWrapped("%s", modal_body_.c_str());
        if (ImGui::Button("确定", ImVec2(100, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (open_extrinsic_confirm_) {
        ImGui::OpenPopup("外参异常警告##calib_ext_confirm");
        open_extrinsic_confirm_ = false;
    }
    if (ImGui::BeginPopupModal("外参异常警告##calib_ext_confirm", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.25f, 1.0f), "相机-IMU 外参标定存在退化或警告：");
        ImGui::TextWrapped("%s", extrinsic_confirm_msg_.c_str());
        ImGui::Spacing();
        if (ImGui::Button("取消（推荐）", ImVec2(140, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("仍强制应用", ImVec2(140, 0))) {
            std::optional<std::pair<std::filesystem::path, std::filesystem::path>> outs;
            {
                std::lock_guard<std::mutex> lk(slam_mtx_);
                outs = last_outputs_;
            }
            if (on_apply_ && outs.has_value()) {
                on_apply_(outs->first, outs->second);
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (open_kalibr_modal_) {
        ImGui::OpenPopup("导入 Kalibr YAML 转换##kalibr_dlg");
        open_kalibr_modal_ = false;
    }
    if (ImGui::BeginPopupModal("导入 Kalibr YAML 转换##kalibr_dlg", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::SetNextItemWidth(380);
        input_text_std("camchain-imucam.yaml", kalibr_camchain_);
        ImGui::SameLine();
        if (ImGui::Button("浏览…##kcam")) {
            if (auto p = pick_open_file("YAML", "yaml,yml")) kalibr_camchain_ = p->string();
        }
        ImGui::SetNextItemWidth(380);
        input_text_std("imu.yaml (可选)", kalibr_imu_);
        ImGui::SameLine();
        if (ImGui::Button("浏览…##kimu")) {
            if (auto p = pick_open_file("YAML", "yaml,yml")) kalibr_imu_ = p->string();
        }
        if (ImGui::Button("执行转换", ImVec2(120, 0))) {
            try {
                auto [yp, jp] = camera_imu_calibration::convert_kalibr_files(
                    std::filesystem::u8path(kalibr_camchain_),
                    kalibr_imu_.empty() ? std::filesystem::path() : std::filesystem::u8path(kalibr_imu_),
                    std::filesystem::u8path(output_dir_),
                    fps_ > 0.0 ? fps_ : 125.0,
                    "umi_ms901m_uniform",
                    accel_g_,
                    gyro_dps_);
                {
                    std::lock_guard<std::mutex> lk(slam_mtx_);
                    last_outputs_ = {yp, jp};
                    last_summary_ = slam_common::JsonValue::object();
                    slam_status_ = "Kalibr 转换成功：" + yp.filename().string();
                    slam_log_ += "\n[Kalibr 转换成功]\nYAML: " + yp.string() + "\nJSON: " + jp.string() + "\n";
                }
                ImGui::CloseCurrentPopup();
            } catch (const std::exception& exc) {
                show_message("Kalibr 转换失败", exc.what());
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("取消", ImVec2(100, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

void CalibrationPane::render_slam_tab() {
    bool running = slam_running_.load();

    // Upload pending preview texture on UI thread
    {
        std::lock_guard<std::mutex> lk(slam_mtx_);
        if (has_pending_slam_preview_) {
            slam_preview_tex_.upload(pending_slam_preview_);
            has_pending_slam_preview_ = false;
        }
    }

    float input_w = std::max(260.0f, ImGui::GetContentRegionAvail().x - 280.0f);
    ImGui::SetNextItemWidth(input_w);
    input_text_std("标定采集目录 / 视频", source_dir_);
    ImGui::SameLine();
    if (ImGui::Button("浏览…##src")) {
        if (auto d = pick_directory()) source_dir_ = d->string();
    }

    ImGui::SetNextItemWidth(input_w);
    input_text_std("标定结果保存目录", output_dir_);
    ImGui::SameLine();
    if (ImGui::Button("浏览…##out")) {
        if (auto d = pick_directory()) output_dir_ = d->string();
    }

    ImGui::SetNextItemWidth(input_w);
    input_text_std("多目目录 (可选;分隔)", multi_dirs_);
    ImGui::SameLine();
    if (ImGui::Button("添加…##mdir")) {
        if (auto d = pick_directory()) {
            if (!multi_dirs_.empty()) multi_dirs_ += "; ";
            multi_dirs_ += d->string();
        }
    }

    ImGui::SetNextItemWidth(190);
    ImGui::Combo("传感器组合", &cam_mode_idx_, kCamModeLabels, 4);
    ImGui::SameLine();
    ImGui::Checkbox("启用 IMU 联合标定", &enable_imu_);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(190);
    ImGui::Combo("标定板", &target_idx_, kTargetLabels, 4);

    ImGui::SetNextItemWidth(70); ImGui::InputInt("行##sr", &rows_, 0, 0); ImGui::SameLine();
    ImGui::SetNextItemWidth(70); ImGui::InputInt("列##sc", &cols_, 0, 0); ImGui::SameLine();
    ImGui::SetNextItemWidth(80); ImGui::InputDouble("边长(mm)##ssz", &tag_size_mm_, 0, 0, "%.2f"); ImGui::SameLine();
    ImGui::SetNextItemWidth(70); ImGui::InputDouble("间距比##ssp", &tag_spacing_, 0, 0, "%.2f"); ImGui::SameLine();
    ImGui::SetNextItemWidth(70); ImGui::InputInt("抽帧##sst", &stride_, 0, 0); ImGui::SameLine();
    ImGui::SetNextItemWidth(75); ImGui::InputInt("最大帧##smf", &max_frames_, 0, 0);

    if (ImGui::CollapsingHeader("IMU / 相机高级参数")) {
        ImGui::SetNextItemWidth(90); ImGui::InputDouble("FPS(0=自动)", &fps_, 0, 0, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(90); ImGui::InputDouble("IMU Hz", &imu_hz_, 0, 0, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(80); ImGui::InputDouble("Acc量程(g)", &accel_g_, 0, 0, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(90); ImGui::InputDouble("Gyro量程(dps)", &gyro_dps_, 0, 0, "%.1f");
        ImGui::SetNextItemWidth(95); ImGui::InputDouble("NoiseGyro", &noise_gyro_, 0, 0, "%.5f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(95); ImGui::InputDouble("NoiseAcc", &noise_acc_, 0, 0, "%.5f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(95); ImGui::InputDouble("GyroWalk", &gyro_walk_, 0, 0, "%.5f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(95); ImGui::InputDouble("AccWalk", &acc_walk_, 0, 0, "%.5f");
    }

    ImGui::BeginDisabled(running);
    if (ImGui::Button("开始免 ROS 标定")) start_calibration();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("停止标定")) stop_calibration();
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("导入 Kalibr YAML…")) open_kalibr_modal_ = true;
    ImGui::SameLine();
    if (ImGui::Button("打开参考 AprilGrid PDF")) {
        auto pdf = find_repo_asset("calibration/targets/aprilgrid_6x6_nominal40mm_A3.pdf");
        if (std::filesystem::is_regular_file(pdf)) open_in_os_shell(pdf);
        else show_message("未找到文件", "未找到参考标定板 PDF：" + pdf.string());
    }
    ImGui::SameLine();
    if (ImGui::Button("打开输出目录")) {
        auto p = std::filesystem::u8path(output_dir_);
        if (std::filesystem::is_directory(p)) open_in_os_shell(p);
        else show_message("提示", "输出目录尚未创建：" + output_dir_);
    }
    std::string st_copy, log_copy;
    bool has_outputs = false;
    {
        std::lock_guard<std::mutex> lk(slam_mtx_);
        st_copy = slam_status_;
        log_copy = slam_log_;
        has_outputs = last_outputs_.has_value();
    }

    ImGui::SameLine();
    ImGui::BeginDisabled(!has_outputs || !on_apply_);
    if (ImGui::Button("应用到当前 SLAM 任务")) apply_to_slam();
    ImGui::EndDisabled();

    ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.80f, 1.0f), "%s", st_copy.c_str());

    if (slam_preview_tex_.valid()) {
        render_scaled_texture(slam_preview_tex_, ImGui::GetContentRegionAvail().x, 180.0f);
    }

    ImVec2 avail = ImGui::GetContentRegionAvail();
    input_text_multiline_std("##slam_log", log_copy, ImVec2(avail.x, std::max(120.0f, avail.y - 30.0f)),
                             ImGuiInputTextFlags_ReadOnly);
    if (ImGui::Button("复制标定报告")) {
        ImGui::SetClipboardText(log_copy.c_str());
    }
}

void CalibrationPane::start_calibration() {
    if (slam_running_.load()) return;
    if (slam_worker_.joinable()) slam_worker_.join();

    camera_imu_calibration::SuiteCalibrationParams p;
    p.source_dir = std::filesystem::u8path(source_dir_);
    p.output_dir = std::filesystem::u8path(output_dir_);
    p.camera_mode = kCamModeKeys[cam_mode_idx_];
    p.enable_imu = enable_imu_;
    p.target_type = kTargetKeys[target_idx_];
    p.rows = rows_;
    p.cols = cols_;
    p.tag_size_m = tag_size_mm_ / 1000.0;
    p.tag_spacing = tag_spacing_;
    p.stride = std::max(1, stride_);
    p.max_frames = std::max(1, max_frames_);
    p.fps = fps_;
    p.imu_hz = imu_hz_;
    p.accel_range_g = accel_g_;
    p.gyro_range_dps = gyro_dps_;
    p.noise_gyro = noise_gyro_;
    p.noise_acc = noise_acc_;
    p.gyro_walk = gyro_walk_;
    p.acc_walk = acc_walk_;

    if (!multi_dirs_.empty()) {
        std::istringstream ss(multi_dirs_);
        std::string seg;
        while (std::getline(ss, seg, ';')) {
            size_t b = seg.find_first_not_of(" \t\r\n");
            if (b == std::string::npos) continue;
            size_t e = seg.find_last_not_of(" \t\r\n");
            p.camera_dirs.push_back(std::filesystem::u8path(seg.substr(b, e - b + 1)));
        }
    }

    slam_cancel_.store(false);
    slam_running_.store(true);
    {
        std::lock_guard<std::mutex> lk(slam_mtx_);
        slam_status_ = "正在检测标定板并求解参数…";
        slam_log_.clear();
    }

    p.cancel_flag = &slam_cancel_;
    p.detect_callback = [tt = p.target_type, r = p.rows, c = p.cols, sz = p.tag_size_m, sp = p.tag_spacing](
                            const cv::Mat& gray, int) -> camera_imu_calibration::DetectedCorners {
        auto det = calibration_targets::detect_target(gray, tt, r, c, sz, sp);
        return {det.found, std::move(det.obj_pts), std::move(det.img_pts)};
    };
    p.progress_cb = [this](const std::string& msg) {
        std::lock_guard<std::mutex> lk(slam_mtx_);
        slam_status_ = msg;
        slam_log_ += msg + "\n";
    };
    p.preview_cb = [this](const cv::Mat& l, const cv::Mat& r,
                          const std::vector<cv::Point2f>& i0, const std::vector<cv::Point2f>& i1, int) {
        cv::Mat combined = make_stereo_preview_mat(l, r, i0, i1);
        std::lock_guard<std::mutex> lk(slam_mtx_);
        pending_slam_preview_ = combined;
        has_pending_slam_preview_ = true;
    };

    slam_worker_ = std::thread([this, p]() mutable {
        try {
            auto out = camera_imu_calibration::calibrate_sensor_suite(p);
            std::lock_guard<std::mutex> lk(slam_mtx_);
            last_outputs_ = {out.yaml_path, out.json_path};
            last_summary_ = out.summary;
            slam_status_ = "标定完成：" + out.yaml_path.filename().string();
            slam_log_ += "\n=== 标定摘要 ===\n" + out.summary.dump(2) + "\n";
        } catch (const std::exception& exc) {
            std::lock_guard<std::mutex> lk(slam_mtx_);
            slam_status_ = std::string("标定失败：") + exc.what();
            slam_log_ += "\nERROR: " + std::string(exc.what()) + "\n";
        }
        slam_running_.store(false);
    });
}

void CalibrationPane::stop_calibration() {
    slam_cancel_.store(true);
    std::lock_guard<std::mutex> lk(slam_mtx_);
    slam_status_ = "正在取消标定…";
}

void CalibrationPane::apply_to_slam() {
    if (!on_apply_) return;
    std::optional<std::pair<std::filesystem::path, std::filesystem::path>> outs;
    slam_common::JsonValue summary;
    {
        std::lock_guard<std::mutex> lk(slam_mtx_);
        outs = last_outputs_;
        summary = last_summary_;
    }
    if (!outs.has_value()) return;
    std::string ext_status;
    std::string ext_warn;
    if (summary.is_object()) {
        if (summary.get("imu_extrinsic_status").is_string()) {
            ext_status = summary.get("imu_extrinsic_status").as_string();
        }
        if (summary.get("imu_extrinsic_warning").is_string()) {
            ext_warn = summary.get("imu_extrinsic_warning").as_string();
        }
    }
    if (!ext_warn.empty() || (!ext_status.empty() && ext_status != "ok" && ext_status != "calibrated")) {
        extrinsic_confirm_msg_ = ext_warn.empty() ? ("外参状态异常：" + ext_status) : ext_warn;
        open_extrinsic_confirm_ = true;
        return;
    }
    on_apply_(outs->first, outs->second);
}

void CalibrationPane::render_board_tab() {
    ImGui::SetNextItemWidth(180); ImGui::Combo("板类型##bt", &b_target_idx_, kTargetLabels, 4); ImGui::SameLine();
    ImGui::SetNextItemWidth(70); ImGui::InputInt("行##br", &b_rows_, 0, 0); ImGui::SameLine();
    ImGui::SetNextItemWidth(70); ImGui::InputInt("列##bc", &b_cols_, 0, 0); ImGui::SameLine();
    ImGui::SetNextItemWidth(80); ImGui::InputDouble("边长(mm)##bsz", &b_size_mm_, 0, 0, "%.2f"); ImGui::SameLine();
    ImGui::SetNextItemWidth(70); ImGui::InputDouble("间距比##bsp", &b_spacing_, 0, 0, "%.2f");

    ImGui::SetNextItemWidth(110); ImGui::Combo("纸张##bp", &b_paper_idx_, kPaperLabels, 5); ImGui::SameLine();
    ImGui::SetNextItemWidth(150); ImGui::Combo("方向##bo", &b_orient_idx_, kOrientLabels, 2); ImGui::SameLine();
    ImGui::SetNextItemWidth(75); ImGui::InputInt("DPI##bdpi", &b_dpi_, 0, 0); ImGui::SameLine();
    ImGui::SetNextItemWidth(75); ImGui::InputDouble("边距(mm)##bm", &b_margin_mm_, 0, 0, "%.1f");
    if (b_paper_idx_ == 4) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(75); ImGui::InputDouble("宽(mm)##bcw", &b_custom_w_, 0, 0, "%.1f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(75); ImGui::InputDouble("高(mm)##bch", &b_custom_h_, 0, 0, "%.1f");
    }

    if (ImGui::Button("生成预览")) preview_board();
    ImGui::SameLine();
    if (ImGui::Button("保存 1:1 PDF…")) save_board("pdf");
    ImGui::SameLine();
    if (ImGui::Button("保存矢量 SVG…")) save_board("svg");
    ImGui::SameLine();
    if (ImGui::Button("保存高清 PNG…")) save_board("png");

    ImGui::TextWrapped("%s", board_info_.c_str());
    ImVec2 avail = ImGui::GetContentRegionAvail();
    if (board_preview_tex_.valid()) {
        render_scaled_texture(board_preview_tex_, avail.x, avail.y);
    }
}

void CalibrationPane::preview_board() {
    try {
        std::array<double, 2> custom{b_custom_w_, b_custom_h_};
        auto rb = calibration_targets::render_calibration_board(
            kTargetKeys[b_target_idx_], b_rows_, b_cols_, b_size_mm_, b_spacing_,
            kPaperLabels[b_paper_idx_], std::min(150, std::max(72, b_dpi_)),
            kOrientKeys[b_orient_idx_], b_margin_mm_,
            b_paper_idx_ == 4 ? &custom : nullptr);
        board_info_ = rb.meta.summary;
        board_preview_tex_.upload(rb.image);
    } catch (const std::exception& exc) {
        show_message("生成标定板失败", exc.what());
    }
}

void CalibrationPane::save_board(const std::string& ext) {
    auto out = pick_save_file("Board", ext.c_str(), std::string("calibration_board.") + ext);
    if (!out.has_value()) return;
    try {
        std::array<double, 2> custom{b_custom_w_, b_custom_h_};
        auto [p, meta] = calibration_targets::save_calibration_board(
            *out, kTargetKeys[b_target_idx_], b_rows_, b_cols_, b_size_mm_, b_spacing_,
            kPaperLabels[b_paper_idx_], b_dpi_, kOrientKeys[b_orient_idx_], b_margin_mm_,
            b_paper_idx_ == 4 ? &custom : nullptr);
        board_info_ = "已保存：" + p.string() + " · " + meta.summary;
    } catch (const std::exception& exc) {
        show_message("保存标定板失败", exc.what());
    }
}

void CalibrationPane::render_handeye_tab() {
    if (ImGui::CollapsingHeader("2D N 点手眼标定与对位 (VisionMaster 兼容)", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SetNextItemWidth(65); ImGui::InputInt("平移点数", &np_trans_n_, 0, 0); ImGui::SameLine();
        ImGui::SetNextItemWidth(65); ImGui::InputInt("旋转点数", &np_rot_n_, 0, 0); ImGui::SameLine();
        ImGui::SetNextItemWidth(65); ImGui::InputInt("原点索引", &np_origin_idx_, 0, 0); ImGui::SameLine();
        ImGui::SetNextItemWidth(65); ImGui::InputDouble("步距X", &np_step_x_, 0, 0, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(65); ImGui::InputDouble("步距Y", &np_step_y_, 0, 0, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(65); ImGui::InputDouble("旋转步角", &np_rot_step_, 0, 0, "%.1f");

        ImGui::SetNextItemWidth(190); ImGui::Combo("安装方式", &np_install_idx_, kInstallLabels, 3); ImGui::SameLine();
        ImGui::SetNextItemWidth(160); ImGui::Combo("自由度##npd", &np_dof_idx_, kDofLabels, 3); ImGui::SameLine();
        ImGui::SetNextItemWidth(130); ImGui::Combo("权重##npw", &np_weight_idx_, kWeightLabels, 4);

        if (ImGui::Button("生成 S 形路径")) gen_npoint_plan();
        ImGui::SameLine();
        if (ImGui::Button("填入示例点")) fill_npoint_demo();
        ImGui::SameLine();
        if (ImGui::Button("执行 N 点标定")) run_npoint();
        ImGui::SameLine();
        if (ImGui::Button("计算对位偏差")) run_alignment();
        ImGui::SameLine();
        if (ImGui::Button("保存 .iwcal…##nps")) save_npt_iwcal();
        ImGui::SameLine();
        if (ImGui::Button("加载 .iwcal…##npl")) load_npt_iwcal();

        float half_w = std::max(200.0f, ImGui::GetContentRegionAvail().x * 0.5f - 8.0f);
        ImGui::BeginGroup();
        ImGui::TextUnformatted("平移对应点 (u, v, X_mm, Y_mm):");
        input_text_multiline_std("##np_trans", np_trans_text_, ImVec2(half_w, 110));
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGui::TextUnformatted("旋转中心点 (u, v [, angle_deg]):");
        input_text_multiline_std("##np_rot", np_rot_text_, ImVec2(half_w, 110));
        ImGui::EndGroup();

        ImGui::SetNextItemWidth(half_w); input_text_std("目标点集(x,y;...)", align_tgt_text_);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(half_w); input_text_std("当前点集(x,y;...)", align_cur_text_);

        input_text_multiline_std("##np_res", np_result_text_, ImVec2(ImGui::GetContentRegionAvail().x, 95),
                                 ImGuiInputTextFlags_ReadOnly);
    }

    if (ImGui::CollapsingHeader("3D 机械臂手眼标定 (AX=XB / AX=ZB)", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SetNextItemWidth(240); ImGui::Combo("标定模式", &he_mode_idx_, kHeModeLabels, 2); ImGui::SameLine();
        ImGui::SetNextItemWidth(160); ImGui::Combo("求解算法", &he_method_idx_, kHeMethodLabels, 5);

        float fw = std::max(240.0f, ImGui::GetContentRegionAvail().x - 240.0f);
        ImGui::SetNextItemWidth(fw);
        input_text_std("机械臂位姿文件 (JSON/CSV)", he_robot_file_);
        ImGui::SameLine();
        if (ImGui::Button("浏览…##her")) {
            if (auto p = pick_open_file("Poses", "json,csv,txt")) he_robot_file_ = p->string();
        }
        ImGui::SetNextItemWidth(fw);
        input_text_std("相机靶标位姿文件 (JSON/CSV)", he_cam_file_);
        ImGui::SameLine();
        if (ImGui::Button("浏览…##hec")) {
            if (auto p = pick_open_file("Poses", "json,csv,txt")) he_cam_file_ = p->string();
        }

        if (ImGui::Button("求解 3D 手眼")) run_handeye_3d();
        ImGui::SameLine();
        if (ImGui::Button("导出结果 JSON…")) save_handeye_3d();

        input_text_multiline_std("##he_res", he_result_text_, ImVec2(ImGui::GetContentRegionAvail().x, 110),
                                 ImGuiInputTextFlags_ReadOnly);
    }
}

void CalibrationPane::fill_npoint_demo() {
    auto plan = handeye_calibration::generate_n_point_plan(9, 3, 4, 0.0, 0.0, 5.0, 5.0, "X", 3, -10.0, 10.0);
    std::ostringstream ts;
    ts << std::fixed << std::setprecision(3);
    for (const auto& pt : plan.trans_points) {
        double u = 320.0 + pt.x * 12.0;
        double v = 240.0 + pt.y * 12.0;
        ts << u << ", " << v << ", " << pt.x << ", " << pt.y << "\n";
    }
    np_trans_text_ = ts.str();
    np_rot_text_ = "350.0, 260.0, -10.0\n362.0, 255.0, 0.0\n370.0, 245.0, 10.0\n";
}

void CalibrationPane::gen_npoint_plan() {
    try {
        auto plan = handeye_calibration::generate_n_point_plan(
            np_trans_n_, np_rot_n_, np_origin_idx_, 0.0, 0.0, np_step_x_, np_step_y_, "X", 3, -np_rot_step_, np_rot_step_);
        std::ostringstream os;
        os << "已生成 " << plan.trans_points.size() << " 平移点 + " << plan.rot_angles_deg.size() << " 旋转角：\n";
        for (const auto& p : plan.trans_points) {
            os << "  #" << p.index << ": (" << p.x << ", " << p.y << ") mm\n";
        }
        np_result_text_ = os.str();
    } catch (const std::exception& exc) {
        show_message("生成路径失败", exc.what());
    }
}

void CalibrationPane::run_npoint() {
    try {
        auto rows = parse_float_rows(np_trans_text_, 4);
        std::vector<Eigen::Vector2d> px, wd;
        for (const auto& r : rows) {
            px.emplace_back(r[0], r[1]);
            wd.emplace_back(r[2], r[3]);
        }
        std::vector<Eigen::Vector2d> rpx;
        std::vector<double> rang;
        if (!np_rot_text_.empty()) {
            auto rrows = parse_float_rows(np_rot_text_, 2);
            for (const auto& r : rrows) {
                rpx.emplace_back(r[0], r[1]);
                if (r.size() >= 3) rang.push_back(r[2]);
            }
        }
        auto res = handeye_calibration::calibrate_n_point_2d(
            px, wd, rpx, rang, kInstallKeys[np_install_idx_], kDofKeys[np_dof_idx_], kWeightKeys[np_weight_idx_]);

        slam_common::JsonValue j = slam_common::JsonValue::object();
        j["tool"] = res.tool;
        j["camera_mode"] = res.camera_mode;
        j["dof"] = res.dof;
        j["rms_mm"] = res.rms_mm;
        j["pixel_size_mm"] = res.pixel_size_mm;
        j["rotation_deg"] = res.rotation_deg;
        slam_common::JsonValue mat = slam_common::JsonValue::array();
        for (int r = 0; r < 3; ++r) {
            slam_common::JsonValue row = slam_common::JsonValue::array();
            for (int c = 0; c < 3; ++c) row.push_back(res.matrix(r, c));
            mat.push_back(row);
        }
        j["matrix"] = mat;
        if (res.rotation_center_world.has_value()) {
            j["rotation_center_world"] = slam_common::JsonValue::Array{
                res.rotation_center_world->x(), res.rotation_center_world->y()};
        }
        last_iwcal_npt_ = j;
        np_result_text_ = j.dump(2);
    } catch (const std::exception& exc) {
        show_message("N 点标定失败", exc.what());
    }
}

void CalibrationPane::run_alignment() {
    try {
        auto tgt = parse_semicolon_xy(align_tgt_text_);
        auto cur = parse_semicolon_xy(align_cur_text_);
        std::optional<Eigen::Vector2d> rc;
        if (last_iwcal_npt_.is_object() && last_iwcal_npt_.get("rotation_center_world").is_array()) {
            const auto& a = last_iwcal_npt_.get("rotation_center_world").as_array();
            if (a.size() >= 2) rc = Eigen::Vector2d(a[0].as_double(), a[1].as_double());
        }
        auto al = handeye_calibration::compute_alignment_offset_2d(tgt, cur, rc, kInstallKeys[np_install_idx_]);
        std::ostringstream os;
        os << std::fixed << std::setprecision(4)
           << "对位偏差结果:\n"
           << "  dX = " << al.dx_mm << " mm\n"
           << "  dY = " << al.dy_mm << " mm\n"
           << "  dTheta = " << al.dtheta_deg << " deg\n"
           << "  Residual = " << al.residual_mm << " mm\n";
        np_result_text_ = os.str();
    } catch (const std::exception& exc) {
        show_message("对位计算失败", exc.what());
    }
}

void CalibrationPane::save_npt_iwcal() {
    if (!last_iwcal_npt_.is_object()) {
        show_message("提示", "请先执行 N 点标定。");
        return;
    }
    if (auto p = pick_save_file("iwcal", "iwcal,json", "npoint.iwcal")) {
        vision_calibration::save_iwcal_file(*p, last_iwcal_npt_);
    }
}

void CalibrationPane::load_npt_iwcal() {
    if (auto p = pick_open_file("iwcal", "iwcal,json")) {
        try {
            last_iwcal_npt_ = vision_calibration::load_iwcal_file(*p);
            np_result_text_ = last_iwcal_npt_.dump(2);
        } catch (const std::exception& exc) {
            show_message("加载失败", exc.what());
        }
    }
}

void CalibrationPane::run_handeye_3d() {
    try {
        auto rposes = handeye_calibration::load_poses_file(std::filesystem::u8path(he_robot_file_));
        auto cposes = handeye_calibration::load_poses_file(std::filesystem::u8path(he_cam_file_));
        auto res = handeye_calibration::calibrate_handeye_3d(
            rposes, cposes, kHeModeKeys[he_mode_idx_], kHeMethodKeys[he_method_idx_]);
        slam_common::JsonValue j = slam_common::JsonValue::object();
        j["mode"] = res.mode;
        j["method"] = res.method;
        j["result_name"] = res.result_name;
        j["rot_rmse_deg"] = res.rot_rmse_deg;
        j["trans_rmse_m"] = res.trans_rmse_m;
        j["pose_count"] = static_cast<int64_t>(res.pose_count);
        slam_common::JsonValue T = slam_common::JsonValue::array();
        for (int r = 0; r < 4; ++r) {
            slam_common::JsonValue row = slam_common::JsonValue::array();
            for (int c = 0; c < 4; ++c) row.push_back(res.T_result(r, c));
            T.push_back(row);
        }
        j["T_result"] = T;
        last_handeye_3d_ = j;
        he_result_text_ = j.dump(2);
    } catch (const std::exception& exc) {
        show_message("3D 手眼标定失败", exc.what());
    }
}

void CalibrationPane::save_handeye_3d() {
    if (!last_handeye_3d_.is_object()) {
        show_message("提示", "请先求解 3D 手眼标定。");
        return;
    }
    if (auto p = pick_save_file("JSON", "json", "handeye_3d.json")) {
        slam_common::save_json(*p, last_handeye_3d_, true);
    }
}

void CalibrationPane::render_vision_tab() {
    bool has_undist = false;
    std::string vlog;
    {
        std::lock_guard<std::mutex> lk(vision_mtx_);
        if (has_pending_vision_preview_) {
            vision_preview_tex_.upload(pending_vision_preview_);
            has_pending_vision_preview_ = false;
        }
        has_undist = !last_undistorted_img_.empty();
        vlog = vision_log_;
    }

    float fw = std::max(240.0f, ImGui::GetContentRegionAvail().x - 240.0f);
    ImGui::SetNextItemWidth(fw);
    input_text_std("相机1标定图像", v_img1_path_);
    ImGui::SameLine();
    if (ImGui::Button("浏览…##v1")) {
        if (auto p = pick_open_file("Images", "png,jpg,jpeg,bmp,tif")) v_img1_path_ = p->string();
    }
    ImGui::SetNextItemWidth(fw);
    input_text_std("相机2标定图像 (双目映射)", v_img2_path_);
    ImGui::SameLine();
    if (ImGui::Button("浏览…##v2")) {
        if (auto p = pick_open_file("Images", "png,jpg,jpeg,bmp,tif")) v_img2_path_ = p->string();
    }

    ImGui::SetNextItemWidth(180); ImGui::Combo("板类型##vt", &v_target_idx_, kTargetLabels, 4); ImGui::SameLine();
    ImGui::SetNextItemWidth(65); ImGui::InputInt("行##vr", &v_rows_, 0, 0); ImGui::SameLine();
    ImGui::SetNextItemWidth(65); ImGui::InputInt("列##vc", &v_cols_, 0, 0); ImGui::SameLine();
    ImGui::SetNextItemWidth(75); ImGui::InputDouble("边长(mm)##vsz", &v_size_mm_, 0, 0, "%.2f"); ImGui::SameLine();
    ImGui::SetNextItemWidth(70); ImGui::InputDouble("间距比##vsp", &v_spacing_, 0, 0, "%.2f");

    ImGui::SetNextItemWidth(160); ImGui::Combo("畸变类型", &v_dist_idx_, kDistLabels, 3); ImGui::SameLine();
    ImGui::SetNextItemWidth(150); ImGui::Combo("自由度##vd", &v_dof_idx_, kDofLabels, 3); ImGui::SameLine();
    ImGui::SetNextItemWidth(120); ImGui::Combo("权重##vw", &v_weight_idx_, kWeightLabels, 4);

    bool busy = vision_running_.load();
    ImGui::BeginDisabled(busy);
    if (ImGui::Button("2D 平面标定")) run_planar_2d();
    ImGui::SameLine();
    if (ImGui::Button("畸变标定并去畸变")) run_distortion_2d();
    ImGui::SameLine();
    if (ImGui::Button("双相机映射")) run_dual_cam_map();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!has_undist);
    if (ImGui::Button("保存去畸变图…")) save_undistorted_img();
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("保存 .iwcal…##vs")) save_planar_iwcal();
    ImGui::SameLine();
    if (ImGui::Button("加载 .iwcal…##vl")) load_planar_iwcal();

    ImGui::SetNextItemWidth(85); ImGui::InputDouble("u(px)", &v_test_u_, 0, 0, "%.1f"); ImGui::SameLine();
    ImGui::SetNextItemWidth(85); ImGui::InputDouble("v(px)", &v_test_v_, 0, 0, "%.1f"); ImGui::SameLine();
    ImGui::SetNextItemWidth(85); ImGui::InputDouble("长度(px)", &v_test_pxlen_, 0, 0, "%.1f"); ImGui::SameLine();
    if (ImGui::Button("像素 -> 物理毫米换算")) test_unit_convert();

    if (vision_preview_tex_.valid()) {
        render_scaled_texture(vision_preview_tex_, ImGui::GetContentRegionAvail().x, 200.0f);
    }

    ImVec2 avail = ImGui::GetContentRegionAvail();
    input_text_multiline_std("##v_log", vlog, ImVec2(avail.x, std::max(100.0f, avail.y)),
                             ImGuiInputTextFlags_ReadOnly);
}

void CalibrationPane::run_planar_2d() {
    if (vision_running_.load()) return;
    if (vision_worker_.joinable()) vision_worker_.join();
    vision_running_.store(true);
    std::string path = v_img1_path_;
    std::string tt = kTargetKeys[v_target_idx_];
    int r = v_rows_, c = v_cols_;
    double sz = v_size_mm_, sp = v_spacing_;
    std::string dof = kDofKeys[v_dof_idx_], wf = kWeightKeys[v_weight_idx_];

    vision_worker_ = std::thread([this, path, tt, r, c, sz, sp, dof, wf]() {
        try {
            cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
            if (img.empty()) throw std::runtime_error("无法读取图像：" + path);
            auto det = calibration_targets::detect_target(img, tt, r, c, sz, sp);
            if (!det.found) throw std::runtime_error("未在图像中检测到标定板角点");
            std::vector<Eigen::Vector2d> ip, op;
            for (size_t i = 0; i < det.img_pts.size(); ++i) {
                ip.emplace_back(det.img_pts[i].x, det.img_pts[i].y);
                op.emplace_back(det.obj_pts[i].x, det.obj_pts[i].y);
            }
            auto cal = vision_calibration::calibrate_planar_board_2d(ip, op, 0.0, 0.0, 0.0, true, dof, wf);
            cv::Mat vis = img.clone();
            for (const auto& p : det.img_pts) cv::circle(vis, cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), 4, cv::Scalar(0, 255, 0), -1);
            std::lock_guard<std::mutex> lk(vision_mtx_);
            last_iwcal_2d_ = cal.to_json();
            pending_vision_preview_ = vis;
            has_pending_vision_preview_ = true;
            vision_log_ = cal.to_json().dump(2);
        } catch (const std::exception& exc) {
            std::lock_guard<std::mutex> lk(vision_mtx_);
            vision_log_ = std::string("ERROR: ") + exc.what();
        }
        vision_running_.store(false);
    });
}

void CalibrationPane::run_distortion_2d() {
    if (vision_running_.load()) return;
    if (vision_worker_.joinable()) vision_worker_.join();
    vision_running_.store(true);
    std::string path = v_img1_path_;
    std::string tt = kTargetKeys[v_target_idx_];
    int r = v_rows_, c = v_cols_;
    double sz = v_size_mm_, sp = v_spacing_;
    std::string dt = kDistKeys[v_dist_idx_], wf = kWeightKeys[v_weight_idx_];

    vision_worker_ = std::thread([this, path, tt, r, c, sz, sp, dt, wf]() {
        try {
            cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
            if (img.empty()) throw std::runtime_error("无法读取图像：" + path);
            auto det = calibration_targets::detect_target(img, tt, r, c, sz, sp);
            if (!det.found) throw std::runtime_error("未在图像中检测到标定板角点");
            auto model = vision_calibration::calibrate_image_distortion(
                {det.obj_pts}, {det.img_pts}, img.size(), dt,
                Eigen::Vector2d(img.cols * 0.5, img.rows * 0.5), wf);
            cv::Mat undist = vision_calibration::undistort_image_with_model(img, model);
            std::lock_guard<std::mutex> lk(vision_mtx_);
            last_dist_model_ = model;
            last_undistorted_img_ = undist;
            pending_vision_preview_ = undist;
            has_pending_vision_preview_ = true;
            vision_log_ = model.to_json().dump(2);
        } catch (const std::exception& exc) {
            std::lock_guard<std::mutex> lk(vision_mtx_);
            vision_log_ = std::string("ERROR: ") + exc.what();
        }
        vision_running_.store(false);
    });
}

void CalibrationPane::save_undistorted_img() {
    cv::Mat img_copy;
    {
        std::lock_guard<std::mutex> lk(vision_mtx_);
        if (!last_undistorted_img_.empty()) img_copy = last_undistorted_img_.clone();
    }
    if (img_copy.empty()) return;
    if (auto p = pick_save_file("PNG", "png", "undistorted.png")) {
        cv::imwrite(p->string(), img_copy);
    }
}

void CalibrationPane::run_dual_cam_map() {
    if (vision_running_.load()) return;
    if (vision_worker_.joinable()) vision_worker_.join();
    vision_running_.store(true);
    std::string p1 = v_img1_path_, p2 = v_img2_path_;
    std::string tt = kTargetKeys[v_target_idx_];
    int r = v_rows_, c = v_cols_;
    double sz = v_size_mm_, sp = v_spacing_;
    std::string dof = kDofKeys[v_dof_idx_], wf = kWeightKeys[v_weight_idx_];

    vision_worker_ = std::thread([this, p1, p2, tt, r, c, sz, sp, dof, wf]() {
        try {
            cv::Mat i1 = cv::imread(p1, cv::IMREAD_COLOR);
            cv::Mat i2 = cv::imread(p2, cv::IMREAD_COLOR);
            if (i1.empty() || i2.empty()) throw std::runtime_error("请先选择两张有效的标定图像");
            auto d1 = calibration_targets::detect_target(i1, tt, r, c, sz, sp);
            auto d2 = calibration_targets::detect_target(i2, tt, r, c, sz, sp);
            if (!d1.found || !d2.found || d1.img_pts.size() != d2.img_pts.size()) {
                throw std::runtime_error("两幅图像未能匹配相同数量的标定板角点");
            }
            std::vector<Eigen::Vector2d> c1, c2;
            for (size_t k = 0; k < d1.img_pts.size(); ++k) {
                c1.emplace_back(d1.img_pts[k].x, d1.img_pts[k].y);
                c2.emplace_back(d2.img_pts[k].x, d2.img_pts[k].y);
            }
            auto dm = vision_calibration::calibrate_dual_camera_mapping_2d(c1, c2, dof, wf);
            std::lock_guard<std::mutex> lk(vision_mtx_);
            vision_log_ = dm.to_json().dump(2);
        } catch (const std::exception& exc) {
            std::lock_guard<std::mutex> lk(vision_mtx_);
            vision_log_ = std::string("ERROR: ") + exc.what();
        }
        vision_running_.store(false);
    });
}

void CalibrationPane::test_unit_convert() {
    slam_common::JsonValue iwcal_copy;
    {
        std::lock_guard<std::mutex> lk(vision_mtx_);
        iwcal_copy = last_iwcal_2d_;
    }
    if (!iwcal_copy.is_object()) {
        show_message("提示", "请先执行 2D 平面标定或加载 .iwcal 文件。");
        return;
    }
    try {
        auto xy = vision_calibration::convert_pixel_to_world_2d(iwcal_copy, v_test_u_, v_test_v_);
        double len_mm = vision_calibration::convert_pixel_length_to_mm(iwcal_copy, v_test_pxlen_, v_test_u_, v_test_v_);
        std::ostringstream os;
        os << std::fixed << std::setprecision(4)
           << "像素 (" << v_test_u_ << ", " << v_test_v_ << ") -> 物理 (" << xy.x() << ", " << xy.y() << ") mm\n"
           << "像素长度 " << v_test_pxlen_ << " px -> " << len_mm << " mm\n";
        std::lock_guard<std::mutex> lk(vision_mtx_);
        vision_log_ = os.str() + "\n" + vision_log_;
    } catch (const std::exception& exc) {
        show_message("换算失败", exc.what());
    }
}

void CalibrationPane::save_planar_iwcal() {
    slam_common::JsonValue iwcal_copy;
    {
        std::lock_guard<std::mutex> lk(vision_mtx_);
        iwcal_copy = last_iwcal_2d_;
    }
    if (!iwcal_copy.is_object()) {
        show_message("提示", "请先执行 2D 平面标定。");
        return;
    }
    if (auto p = pick_save_file("iwcal", "iwcal,json", "planar_2d.iwcal")) {
        vision_calibration::save_iwcal_file(*p, iwcal_copy);
    }
}

void CalibrationPane::load_planar_iwcal() {
    if (auto p = pick_open_file("iwcal", "iwcal,json")) {
        try {
            auto loaded = vision_calibration::load_iwcal_file(*p);
            std::lock_guard<std::mutex> lk(vision_mtx_);
            last_iwcal_2d_ = std::move(loaded);
            vision_log_ = last_iwcal_2d_.dump(2);
        } catch (const std::exception& exc) {
            show_message("加载失败", exc.what());
        }
    }
}

}  // namespace slam_app
