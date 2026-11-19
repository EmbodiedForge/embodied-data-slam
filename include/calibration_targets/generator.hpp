#pragma once

#include <opencv2/core.hpp>
#include <array>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace calibration_targets {

struct PaperSize {
    std::string name;
    double w_mm;
    double h_mm;
};

const std::vector<PaperSize>& standard_paper_sizes();

struct BoardMeta {
    std::string kind;
    int rows = 6;
    int cols = 6;
    double size_mm = 34.5;
    double spacing_ratio = 0.3;
    double pattern_w_mm = 0.0;
    double pattern_h_mm = 0.0;
    double page_w_mm = 0.0;
    double page_h_mm = 0.0;
    double offset_x_mm = 0.0;
    double offset_y_mm = 0.0;
    int dpi = 150;
    std::array<int, 2> image_size{0, 0};
    std::string paper;
    std::string orientation;
    std::string summary;
    std::string path;
};

struct RenderedBoard {
    cv::Mat image;
    BoardMeta meta;
};

/// Normalize calibration target alias to aprilgrid / chessboard / circles / acircles.
std::string normalize_target_type(const std::string& target_type);

/// Render a 1:1 physical-scale grayscale calibration board image and metadata.
RenderedBoard render_calibration_board(
    const std::string& target_type = "aprilgrid",
    int rows = 6,
    int cols = 6,
    double size_mm = 34.5,
    double spacing_ratio = 0.3,
    const std::string& paper = "A3",
    int dpi = 150,
    const std::string& orientation = "landscape",
    double margin_mm = 12.0,
    const std::array<double, 2>* custom_page_mm = nullptr);

/// Save a 1:1 physical-scale calibration board to PDF, SVG, PNG (with pHYs DPI), or raster formats.
std::pair<std::filesystem::path, BoardMeta> save_calibration_board(
    const std::filesystem::path& output_path,
    const std::string& target_type = "aprilgrid",
    int rows = 6,
    int cols = 6,
    double size_mm = 34.5,
    double spacing_ratio = 0.3,
    const std::string& paper = "A3",
    int dpi = 300,
    const std::string& orientation = "landscape",
    double margin_mm = 12.0,
    const std::array<double, 2>* custom_page_mm = nullptr);

}  // namespace calibration_targets
