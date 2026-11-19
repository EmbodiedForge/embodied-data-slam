#include "calibration_targets/detector.hpp"

#include "calibration_targets/generator.hpp"
#include "slam_common/geometry.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace calibration_targets {
namespace {

cv::Mat to_gray(const cv::Mat& image) {
    if (image.empty()) throw std::invalid_argument("输入图像为空");
    if (image.channels() == 1) return image;
    cv::Mat gray;
    if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    return gray;
}

cv::Ptr<cv::SimpleBlobDetector> build_blob_detector(
    int blob_color,
    double min_circularity,
    double image_area) {
    cv::SimpleBlobDetector::Params params;
    params.filterByColor = true;
    params.blobColor = static_cast<uchar>(blob_color);
    params.filterByCircularity = true;
    params.minCircularity = static_cast<float>(std::max(0.05, std::min(0.99, min_circularity)));
    params.filterByArea = true;
    params.minArea = 12.0f;
    params.maxArea = static_cast<float>(std::max(25000.0, image_area * 0.2));
    params.filterByInertia = true;
    params.minInertiaRatio = 0.2f;
    params.filterByConvexity = true;
    params.minConvexity = 0.55f;
    return cv::SimpleBlobDetector::create(params);
}

}  // namespace

AprilTagDetectorFn create_apriltag_detector() {
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::aruco::DetectorParameters params;
    params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    cv::aruco::ArucoDetector detector(dict, params);
    return [detector](const cv::Mat& gray,
                      std::vector<std::vector<cv::Point2f>>& corners,
                      std::vector<int>& ids) mutable {
        std::vector<std::vector<cv::Point2f>> rejected;
        detector.detectMarkers(gray, corners, ids, rejected);
    };
}

std::vector<std::vector<cv::Point2f>> refine_apriltag_corners(
    const cv::Mat& gray,
    const std::vector<std::vector<cv::Point2f>>& raw_corners,
    double expand) {
    if (raw_corners.empty()) return {};
    float max_x = std::max(1.0f, static_cast<float>(gray.cols - 2));
    float max_y = std::max(1.0f, static_cast<float>(gray.rows - 2));
    std::vector<cv::Point2f> flat;
    flat.reserve(raw_corners.size() * 4);
    for (const auto& quad : raw_corners) {
        if (quad.size() < 4) continue;
        cv::Point2f center(0.0f, 0.0f);
        for (int k = 0; k < 4; ++k) center += quad[k];
        center *= 0.25f;
        for (int k = 0; k < 4; ++k) {
            cv::Point2f pt = quad[k];
            if (std::abs(expand - 1.0) > 1e-6) {
                pt = center + (pt - center) * static_cast<float>(expand);
            }
            pt.x = std::max(1.0f, std::min(max_x, pt.x));
            pt.y = std::max(1.0f, std::min(max_y, pt.y));
            flat.push_back(pt);
        }
    }
    if (flat.empty()) return {};
    cv::TermCriteria term(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 1e-4);
    cv::cornerSubPix(gray, flat, cv::Size(5, 5), cv::Size(-1, -1), term);
    std::vector<std::vector<cv::Point2f>> out;
    out.reserve(flat.size() / 4);
    for (size_t i = 0; i + 3 < flat.size(); i += 4) {
        out.push_back({flat[i], flat[i + 1], flat[i + 2], flat[i + 3]});
    }
    return out;
}

AprilGridPerm infer_aprilgrid_perm(
    const cv::Mat& image,
    const AprilTagDetectorFn& detect_fn,
    int rows,
    int cols,
    double tag_size,
    double tag_spacing) {
    cv::Mat gray = to_gray(image);
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    detect_fn(gray, corners, ids);
    if (ids.empty()) return {};

    int max_id = rows * cols;
    std::vector<std::vector<cv::Point2f>> raw_quads;
    std::vector<int> valid_ids;
    for (size_t i = 0; i < std::min(corners.size(), ids.size()); ++i) {
        if (ids[i] >= 0 && ids[i] < max_id) {
            raw_quads.push_back(corners[i]);
            valid_ids.push_back(ids[i]);
        }
    }
    if (valid_ids.size() < 4) return {};

    AprilGridPerm best;
    double best_err = 1e18;
    for (double expand : {7.0 / 6.0, 1.0}) {
        auto refined = refine_apriltag_corners(gray, raw_quads, expand);
        std::vector<cv::Point2f> imgs;
        imgs.reserve(refined.size() * 4);
        for (const auto& q : refined) {
            for (const auto& p : q) imgs.push_back(p);
        }
        for (bool bu : {false, true}) {
            for (bool rev : {false, true}) {
                for (int roll = 0; roll < 4; ++roll) {
                    std::vector<cv::Point2f> objs_2d;
                    objs_2d.reserve(valid_ids.size() * 4);
                    for (int tid : valid_ids) {
                        auto c3 = slam_common::aprilgrid_tag_corners_3d(
                            tid, rows, cols, tag_size, tag_spacing, bu, rev, roll);
                        for (const auto& p : c3) {
                            objs_2d.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()));
                        }
                    }
                    cv::Mat H = cv::findHomography(objs_2d, imgs, 0);
                    if (H.empty()) continue;
                    std::vector<cv::Point2f> proj;
                    cv::perspectiveTransform(objs_2d, proj, H);
                    double sq_sum = 0.0;
                    for (size_t k = 0; k < imgs.size(); ++k) {
                        cv::Point2f d = proj[k] - imgs[k];
                        sq_sum += static_cast<double>(d.x * d.x + d.y * d.y);
                    }
                    double err = std::sqrt(sq_sum / static_cast<double>(imgs.size()));
                    if (err < best_err) {
                        best_err = err;
                        best = {bu, rev, roll, expand, true};
                    }
                }
            }
        }
    }
    if (!best.valid || best_err >= 25.0) return {};
    return best;
}

TargetDetectionResult detect_aprilgrid_corners(
    const cv::Mat& image,
    const AprilTagDetectorFn& detect_fn,
    int rows,
    int cols,
    double tag_size,
    double tag_spacing,
    bool bottom_up,
    bool rev,
    int roll,
    double expand) {
    cv::Mat gray = to_gray(image);
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    detect_fn(gray, corners, ids);
    if (ids.empty()) return {};

    std::vector<std::vector<cv::Point2f>> selected_quads;
    std::vector<int> selected_ids;
    std::set<int> seen;
    int max_id = rows * cols;
    for (size_t i = 0; i < std::min(corners.size(), ids.size()); ++i) {
        int tid = ids[i];
        if (tid < 0 || tid >= max_id || seen.count(tid)) continue;
        seen.insert(tid);
        selected_quads.push_back(corners[i]);
        selected_ids.push_back(tid);
    }
    if (seen.size() < 2) return {};

    auto refined = refine_apriltag_corners(gray, selected_quads, expand);
    TargetDetectionResult res;
    res.found = true;
    res.obj_pts.reserve(selected_ids.size() * 4);
    res.img_pts.reserve(selected_ids.size() * 4);
    for (size_t i = 0; i < selected_ids.size(); ++i) {
        auto c3 = slam_common::aprilgrid_tag_corners_3d(
            selected_ids[i], rows, cols, tag_size, tag_spacing, bottom_up, rev, roll);
        for (int k = 0; k < 4; ++k) {
            res.obj_pts.emplace_back(
                static_cast<float>(c3[k].x()),
                static_cast<float>(c3[k].y()),
                static_cast<float>(c3[k].z()));
            res.img_pts.push_back(refined[i][k]);
        }
    }
    return res;
}

TargetDetectionResult detect_chessboard_corners(
    const cv::Mat& image,
    int inner_cols,
    int inner_rows,
    double square_size,
    bool median_blur,
    int subpix_win,
    double min_contrast) {
    cv::Mat gray = to_gray(image);
    cv::Mat proc;
    if (median_blur) {
        cv::medianBlur(gray, proc, 3);
    } else {
        proc = gray;
    }
    cv::Size pattern(inner_cols, inner_rows);
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCornersSB(
        proc, pattern, corners, cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE);
    if (!found || corners.empty()) {
        found = cv::findChessboardCorners(
            proc, pattern, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
        if (found && !corners.empty()) {
            int win = std::max(2, subpix_win);
            cv::TermCriteria term(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 1e-3);
            cv::cornerSubPix(proc, corners, cv::Size(win, win), cv::Size(-1, -1), term);
        }
    } else if (subpix_win > 0) {
        int win = std::max(2, subpix_win);
        cv::TermCriteria term(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 1e-4);
        cv::cornerSubPix(proc, corners, cv::Size(win, win), cv::Size(-1, -1), term);
    }
    if (!found || corners.empty()) return {};

    if (min_contrast > 0.0) {
        int r = std::max(5, subpix_win > 0 ? subpix_win : 5);
        size_t step = std::max<size_t>(1, corners.size() / 16);
        std::vector<double> contrasts;
        for (size_t i = 0; i < corners.size(); i += step) {
            int u = static_cast<int>(std::llround(corners[i].x));
            int v = static_cast<int>(std::llround(corners[i].y));
            int x0 = std::max(0, u - r), x1 = std::min(proc.cols, u + r + 1);
            int y0 = std::max(0, v - r), y1 = std::min(proc.rows, v + r + 1);
            if (x1 > x0 && y1 > y0) {
                double mn = 0.0, mx = 0.0;
                cv::minMaxLoc(proc(cv::Rect(x0, y0, x1 - x0, y1 - y0)), &mn, &mx);
                contrasts.push_back(mx - mn);
            }
        }
        if (contrasts.empty()) return {};
        std::sort(contrasts.begin(), contrasts.end());
        double med = contrasts[contrasts.size() / 2];
        if (med < min_contrast) return {};
    }

    TargetDetectionResult res;
    res.found = true;
    res.img_pts = std::move(corners);
    res.obj_pts.reserve(static_cast<size_t>(inner_cols * inner_rows));
    for (int r = 0; r < inner_rows; ++r) {
        for (int c = 0; c < inner_cols; ++c) {
            res.obj_pts.emplace_back(
                static_cast<float>(c * square_size),
                static_cast<float>(r * square_size),
                0.0f);
        }
    }
    return res;
}

TargetDetectionResult detect_circle_grid_corners(
    const cv::Mat& image,
    int cols,
    int rows,
    double spacing_m,
    bool asymmetric,
    double min_circularity,
    const std::string& blob_polarity) {
    cv::Mat gray = to_gray(image);
    cv::Size pattern(cols, rows);
    int base_flag = asymmetric ? cv::CALIB_CB_ASYMMETRIC_GRID : cv::CALIB_CB_SYMMETRIC_GRID;
    double area = static_cast<double>(gray.rows) * static_cast<double>(gray.cols);

    std::vector<int> polarities;
    if (blob_polarity == "light") polarities = {255};
    else if (blob_polarity == "dark") polarities = {0};
    else polarities = {0, 255};

    std::vector<cv::Point2f> centers;
    bool ok = false;
    for (int bcolor : polarities) {
        auto det = build_blob_detector(bcolor, min_circularity, area);
        for (int flag : {base_flag, base_flag | cv::CALIB_CB_CLUSTERING}) {
            centers.clear();
            if (cv::findCirclesGrid(gray, pattern, centers, flag, det) && !centers.empty()) {
                ok = true;
                break;
            }
        }
        if (ok) break;
    }
    if (!ok) return {};

    TargetDetectionResult res;
    res.found = true;
    res.img_pts = std::move(centers);
    res.obj_pts.reserve(static_cast<size_t>(rows * cols));
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            double x = asymmetric ? ((2.0 * c + (r % 2)) * 0.5 * spacing_m) : (c * spacing_m);
            double y = asymmetric ? (r * 0.5 * spacing_m) : (r * spacing_m);
            res.obj_pts.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0f);
        }
    }
    return res;
}

TargetDetectionResult detect_target(
    const cv::Mat& image,
    const std::string& target_type,
    int rows,
    int cols,
    double size_m,
    double spacing_ratio,
    const TargetDetectOptions& opts) {
    std::string kind = normalize_target_type(target_type);
    if (kind == "aprilgrid") {
        AprilTagDetectorFn fn = opts.detect_fn ? opts.detect_fn : create_apriltag_detector();
        AprilGridPerm perm = opts.april_perm;
        if (!perm.valid && opts.auto_perm) {
            perm = infer_aprilgrid_perm(image, fn, rows, cols, size_m, spacing_ratio);
        }
        if (!perm.valid) {
            perm = {false, false, 2, 7.0 / 6.0, true};
        }
        return detect_aprilgrid_corners(
            image, fn, rows, cols, size_m, spacing_ratio,
            perm.bottom_up, perm.rev, perm.roll, perm.expand);
    }
    if (kind == "chessboard") {
        return detect_chessboard_corners(
            image, cols, rows, size_m, opts.median_blur, opts.subpix_win, opts.min_contrast);
    }
    return detect_circle_grid_corners(
        image, cols, rows, size_m, kind == "acircles", opts.min_circularity, opts.blob_polarity);
}

}  // namespace calibration_targets
