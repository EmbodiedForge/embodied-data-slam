#pragma once

#include <opencv2/core.hpp>
#include <functional>
#include <string>
#include <vector>

namespace calibration_targets {

struct AprilGridPerm {
    bool bottom_up = false;
    bool rev = false;
    int roll = 2;
    double expand = 7.0 / 6.0;
    bool valid = false;
};

struct TargetDetectionResult {
    bool found = false;
    std::vector<cv::Point3f> obj_pts;
    std::vector<cv::Point2f> img_pts;
};

using AprilTagDetectorFn = std::function<void(
    const cv::Mat& gray,
    std::vector<std::vector<cv::Point2f>>& corners,
    std::vector<int>& ids)>;

/// Create a callable AprilTag 36h11 marker detector with subpixel refinement disabled.
AprilTagDetectorFn create_apriltag_detector();

/// Expand raw inner AprilTag quads outward when needed and refine corners with cornerSubPix.
std::vector<std::vector<cv::Point2f>> refine_apriltag_corners(
    const cv::Mat& gray,
    const std::vector<std::vector<cv::Point2f>>& raw_corners,
    double expand = 7.0 / 6.0);

/// Infer Kalibr/OpenCV AprilGrid corner permutation and quad expansion factor via homography fit.
AprilGridPerm infer_aprilgrid_perm(
    const cv::Mat& image,
    const AprilTagDetectorFn& detect_fn,
    int rows,
    int cols,
    double tag_size,
    double tag_spacing);

/// Detect AprilGrid 36h11 markers and return aligned 3D object corners and 2D image corners.
TargetDetectionResult detect_aprilgrid_corners(
    const cv::Mat& image,
    const AprilTagDetectorFn& detect_fn,
    int rows,
    int cols,
    double tag_size,
    double tag_spacing,
    bool bottom_up = false,
    bool rev = false,
    int roll = 2,
    double expand = 7.0 / 6.0);

/// Detect inner checkerboard corners with optional median blur and contrast verification.
TargetDetectionResult detect_chessboard_corners(
    const cv::Mat& image,
    int inner_cols,
    int inner_rows,
    double square_size,
    bool median_blur = false,
    int subpix_win = 5,
    double min_contrast = 0.0);

/// Detect symmetric or asymmetric circle grid centers with auto/dark/light polarity.
TargetDetectionResult detect_circle_grid_corners(
    const cv::Mat& image,
    int cols,
    int rows,
    double spacing_m,
    bool asymmetric = false,
    double min_circularity = 0.7,
    const std::string& blob_polarity = "auto");

struct TargetDetectOptions {
    AprilTagDetectorFn detect_fn;
    AprilGridPerm april_perm;
    bool auto_perm = true;
    bool median_blur = false;
    int subpix_win = 5;
    double min_contrast = 0.0;
    double min_circularity = 0.7;
    std::string blob_polarity = "auto";
};

/// Unified calibration target detector supporting aprilgrid, chessboard, circles, and acircles.
TargetDetectionResult detect_target(
    const cv::Mat& image,
    const std::string& target_type,
    int rows,
    int cols,
    double size_m,
    double spacing_ratio = 0.3,
    const TargetDetectOptions& opts = TargetDetectOptions{});

}  // namespace calibration_targets
