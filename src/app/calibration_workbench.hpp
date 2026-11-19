#pragma once

#include "platform_ui.hpp"
#include "slam_common/json.hpp"
#include "vision_calibration/planar_and_distortion.hpp"

#include <opencv2/core.hpp>

#include <atomic>
#include <filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace slam_app {

/// 4-Tab Dear ImGui ROS-free multi-modal calibration, board generator, N-point / 3D hand-eye, and 2D vision workbench.
class CalibrationPane {
public:
    using ApplyCallback = std::function<void(const std::filesystem::path& yaml, const std::filesystem::path& json)>;
    using PopoutCallback = std::function<void()>;

    explicit CalibrationPane(
        ApplyCallback on_apply = nullptr,
        PopoutCallback on_popout = nullptr);
    ~CalibrationPane();

    void render(const char* id_prefix = "calibrator");

private:
    void render_slam_tab();
    void render_board_tab();
    void render_handeye_tab();
    void render_vision_tab();
    void render_popups();

    void start_calibration();
    void stop_calibration();
    void apply_to_slam();
    void preview_board();
    void save_board(const std::string& ext);

    void fill_npoint_demo();
    void gen_npoint_plan();
    void run_npoint();
    void run_alignment();
    void save_npt_iwcal();
    void load_npt_iwcal();
    void run_handeye_3d();
    void save_handeye_3d();

    void run_planar_2d();
    void run_distortion_2d();
    void save_undistorted_img();
    void run_dual_cam_map();
    void test_unit_convert();
    void save_planar_iwcal();
    void load_planar_iwcal();

    void show_message(const std::string& title, const std::string& body);

    ApplyCallback on_apply_;
    PopoutCallback on_popout_;

    // Modal state
    std::string modal_title_;
    std::string modal_body_;
    bool open_msg_modal_ = false;
    bool open_extrinsic_confirm_ = false;
    std::string extrinsic_confirm_msg_;
    bool open_kalibr_modal_ = false;
    std::string kalibr_camchain_;
    std::string kalibr_imu_;

    // Tab 1: Multi-modal SLAM calibration
    std::string source_dir_;
    std::string output_dir_;
    std::string multi_dirs_;
    int cam_mode_idx_ = 0;  // 0: stereo, 1: mono, 2: multi, 3: none
    bool enable_imu_ = true;
    int target_idx_ = 0;    // 0: aprilgrid, 1: chessboard, 2: circles, 3: acircles
    int rows_ = 6;
    int cols_ = 6;
    double tag_size_mm_ = 34.5;
    double tag_spacing_ = 0.3;
    int stride_ = 5;
    int max_frames_ = 120;
    double fps_ = 0.0;
    double imu_hz_ = 250.0;
    double accel_g_ = 4.0;
    double gyro_dps_ = 2000.0;
    double noise_gyro_ = 0.0035;
    double noise_acc_ = 0.02;
    double gyro_walk_ = 0.0002;
    double acc_walk_ = 0.002;

    std::mutex slam_mtx_;
    std::string slam_status_ = "请选择标定目录并点击“开始免 ROS 标定”。";
    std::string slam_log_;
    cv::Mat pending_slam_preview_;
    bool has_pending_slam_preview_ = false;
    GlTexture slam_preview_tex_;
    std::thread slam_worker_;
    std::atomic<bool> slam_cancel_{false};
    std::atomic<bool> slam_running_{false};
    std::optional<std::pair<std::filesystem::path, std::filesystem::path>> last_outputs_;
    slam_common::JsonValue last_summary_;

    // Tab 2: Board generator
    int b_target_idx_ = 0;
    int b_rows_ = 6;
    int b_cols_ = 6;
    double b_size_mm_ = 34.5;
    double b_spacing_ = 0.3;
    int b_paper_idx_ = 0;   // A3, A4, A2, Letter, Custom
    int b_orient_idx_ = 0;  // landscape, portrait
    int b_dpi_ = 300;
    double b_margin_mm_ = 12.0;
    double b_custom_w_ = 420.0;
    double b_custom_h_ = 297.0;
    std::string board_info_ = "点击“生成预览”查看 1:1 物理尺寸标定板";
    GlTexture board_preview_tex_;

    // Tab 3: N-point 2D & 3D Hand-Eye
    int np_trans_n_ = 9;
    int np_rot_n_ = 3;
    int np_origin_idx_ = 4;
    double np_step_x_ = 5.0;
    double np_step_y_ = 5.0;
    double np_rot_step_ = 10.0;
    int np_install_idx_ = 0;  // static_up, static_down, moving
    int np_dof_idx_ = 1;      // similarity, affine, homography
    int np_weight_idx_ = 0;   // least_squares, huber, tukey, ransac
    std::string np_trans_text_;
    std::string np_rot_text_;
    std::string align_tgt_text_ = "10, 2; 20, 2; 15, 8";
    std::string align_cur_text_ = "10, 2; 20, 2; 15, 8";
    std::string np_result_text_;

    int he_mode_idx_ = 0;     // eye_in_hand, eye_to_hand
    int he_method_idx_ = 0;   // tsai, park, horaud, andreff, daniilidis
    std::string he_robot_file_;
    std::string he_cam_file_;
    std::string he_result_text_;
    slam_common::JsonValue last_iwcal_npt_;
    slam_common::JsonValue last_handeye_3d_;

    // Tab 4: 2D Vision
    std::string v_img1_path_;
    std::string v_img2_path_;
    int v_target_idx_ = 1;  // chessboard default
    int v_rows_ = 6;
    int v_cols_ = 9;
    double v_size_mm_ = 20.0;
    double v_spacing_ = 0.3;
    double v_cx_ = 0.0;
    double v_cy_ = 0.0;
    int v_dist_idx_ = 0;    // radial_perspective, radial, perspective
    int v_dof_idx_ = 2;     // similarity, affine, homography
    int v_weight_idx_ = 0;
    double v_test_u_ = 320.0;
    double v_test_v_ = 240.0;
    double v_test_pxlen_ = 100.0;

    std::mutex vision_mtx_;
    std::string vision_log_;
    cv::Mat pending_vision_preview_;
    bool has_pending_vision_preview_ = false;
    GlTexture vision_preview_tex_;
    std::thread vision_worker_;
    std::atomic<bool> vision_running_{false};
    slam_common::JsonValue last_iwcal_2d_;
    std::optional<vision_calibration::DistortionModel> last_dist_model_;
    cv::Mat last_undistorted_img_;
};

}  // namespace slam_app
