#include "calibration_targets/generator.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace calibration_targets {
namespace {

constexpr double kPtPerMm = 72.0 / 25.4;
constexpr double kKappa = 0.5522847498;

struct RectMm {
    double x, y, w, h;
};

struct CircleMm {
    double cx, cy, r;
};

struct BoardGeometry {
    BoardMeta meta;
    std::vector<RectMm> black_rects;
    std::vector<CircleMm> black_circles;
};

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

std::string fmt(double v, int prec) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}

cv::Mat aprilgrid_tag_bits(int tag_id) {
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Mat marker;
    cv::aruco::generateImageMarker(dict, tag_id, 8, marker, 1);
    return marker;
}

BoardGeometry compute_board_geometry(
    const std::string& target_type,
    int rows,
    int cols,
    double size_mm,
    double spacing_ratio) {
    std::string kind = normalize_target_type(target_type);
    if (rows < 2 || cols < 2 || !std::isfinite(size_mm) || size_mm <= 0.0) {
        throw std::invalid_argument("行列数必须 >= 2 且物理尺寸 > 0 mm");
    }

    BoardGeometry g;
    g.meta.kind = kind;
    g.meta.rows = rows;
    g.meta.cols = cols;
    g.meta.size_mm = size_mm;
    g.meta.spacing_ratio = spacing_ratio;

    if (kind == "aprilgrid") {
        if (rows * cols > 587) {
            throw std::invalid_argument("AprilTag 36h11 最多支持 587 个标签 (rows * cols <= 587)");
        }
        if (spacing_ratio < 0.05 || spacing_ratio > 1.0) {
            throw std::invalid_argument("AprilGrid 间距比建议在 [0.05, 1.0] 之间");
        }
        double gap_mm = size_mm * spacing_ratio;
        double pitch_mm = size_mm + gap_mm;
        g.meta.pattern_w_mm = cols * size_mm + std::max(0, cols - 1) * gap_mm;
        g.meta.pattern_h_mm = rows * size_mm + std::max(0, rows - 1) * gap_mm;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                int tid = (rows - 1 - r) * cols + c;
                double ox = c * pitch_mm;
                double oy = r * pitch_mm;
                cv::Mat grid = aprilgrid_tag_bits(tid);
                double cell = size_mm / static_cast<double>(grid.rows);
                for (int br = 0; br < grid.rows; ++br) {
                    for (int bc = 0; bc < grid.cols; ++bc) {
                        if (grid.at<uint8_t>(br, bc) < 128) {
                            g.black_rects.push_back({ox + bc * cell, oy + br * cell, cell, cell});
                        }
                    }
                }
            }
        }
        for (int r = 0; r < rows - 1; ++r) {
            for (int c = 0; c < cols - 1; ++c) {
                g.black_rects.push_back({c * pitch_mm + size_mm, r * pitch_mm + size_mm, gap_mm, gap_mm});
            }
        }
        g.meta.summary = "AprilGrid 36h11 " + std::to_string(rows) + "x" + std::to_string(cols) +
                         " tags | tag=" + fmt(size_mm, 2) + "mm gap=" + fmt(gap_mm, 2) +
                         "mm (" + fmt(spacing_ratio, 2) + ")";
    } else if (kind == "chessboard") {
        int sq_rows = rows + 1;
        int sq_cols = cols + 1;
        g.meta.pattern_w_mm = sq_cols * size_mm;
        g.meta.pattern_h_mm = sq_rows * size_mm;
        for (int r = 0; r < sq_rows; ++r) {
            for (int c = 0; c < sq_cols; ++c) {
                if ((r + c) % 2 == 0) {
                    g.black_rects.push_back({c * size_mm, r * size_mm, size_mm, size_mm});
                }
            }
        }
        g.meta.summary = "Checkerboard " + std::to_string(cols) + "x" + std::to_string(rows) +
                         " inner corners (" + std::to_string(sq_cols) + "x" + std::to_string(sq_rows) +
                         " squares) | square=" + fmt(size_mm, 2) + "mm";
    } else if (kind == "circles") {
        double diam_ratio = (spacing_ratio >= 0.12 && spacing_ratio <= 0.85) ? spacing_ratio : 0.5;
        double diam_mm = size_mm * diam_ratio;
        double rad_mm = diam_mm * 0.5;
        g.meta.pattern_w_mm = (cols - 1) * size_mm + diam_mm;
        g.meta.pattern_h_mm = (rows - 1) * size_mm + diam_mm;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                g.black_circles.push_back({rad_mm + c * size_mm, rad_mm + r * size_mm, rad_mm});
            }
        }
        g.meta.summary = "Symmetric Circles " + std::to_string(cols) + "x" + std::to_string(rows) +
                         " | pitch=" + fmt(size_mm, 2) + "mm dia=" + fmt(diam_mm, 2) + "mm";
    } else {
        double diam_ratio = std::min(0.62, (spacing_ratio >= 0.12 && spacing_ratio <= 0.62) ? spacing_ratio : 0.42);
        double diam_mm = size_mm * diam_ratio;
        double rad_mm = diam_mm * 0.5;
        double x_span = ((cols - 1) + (rows > 1 ? 0.5 : 0.0)) * size_mm;
        double y_span = (rows - 1) * 0.5 * size_mm;
        g.meta.pattern_w_mm = x_span + diam_mm;
        g.meta.pattern_h_mm = y_span + diam_mm;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                double cx = rad_mm + (2 * c + (r % 2)) * 0.5 * size_mm;
                double cy = rad_mm + r * 0.5 * size_mm;
                g.black_circles.push_back({cx, cy, rad_mm});
            }
        }
        g.meta.summary = "Asymmetric Circles " + std::to_string(cols) + "x" + std::to_string(rows) +
                         " | pitch=" + fmt(size_mm, 2) + "mm stepY=" + fmt(size_mm * 0.5, 2) +
                         "mm dia=" + fmt(diam_mm, 2) + "mm";
    }
    return g;
}

std::pair<double, double> resolve_page_size(
    const std::string& paper,
    const std::string& orientation,
    double pat_w,
    double pat_h,
    double margin_mm,
    const std::array<double, 2>* custom_page_mm) {
    constexpr double kFooterMm = 10.0;
    std::string p_low = trim_lower(paper);
    if (p_low == "auto") {
        return {pat_w + 2.0 * margin_mm, pat_h + 2.0 * margin_mm + kFooterMm};
    }
    double w = 0.0, h = 0.0;
    if (custom_page_mm != nullptr) {
        w = (*custom_page_mm)[0];
        h = (*custom_page_mm)[1];
    } else {
        bool found = false;
        for (const auto& ps : standard_paper_sizes()) {
            if (trim_lower(ps.name) == p_low) {
                w = ps.w_mm;
                h = ps.h_mm;
                found = true;
                break;
            }
        }
        if (!found) {
            throw std::invalid_argument("未知纸张尺寸 " + paper + "，可选 A4, A3, A2, Letter / Auto / Custom");
        }
    }
    if (trim_lower(orientation) == "portrait") {
        double nw = std::min(w, h), nh = std::max(w, h);
        w = nw; h = nh;
    } else {
        double nw = std::max(w, h), nh = std::min(w, h);
        w = nw; h = nh;
    }
    double avail_w = w - 2.0 * margin_mm;
    double avail_h = h - 2.0 * margin_mm - kFooterMm;
    if (pat_w > avail_w + 1e-6 || pat_h > avail_h + 1e-6) {
        throw std::invalid_argument(
            "标定板物理尺寸 (" + fmt(pat_w, 1) + "x" + fmt(pat_h, 1) + " mm) 超出 " + paper +
            " 可打印区域 (" + fmt(avail_w, 1) + "x" + fmt(avail_h, 1) +
            " mm)，请调小格距/标签尺寸、减少行列数、改用更大纸张或选择 Auto。");
    }
    return {w, h};
}

std::string build_pdf_bytes(const BoardGeometry& g) {
    const auto& m = g.meta;
    double w_pt = m.page_w_mm * kPtPerMm;
    double h_pt = m.page_h_mm * kPtPerMm;
    std::ostringstream cmds;
    cmds << "0 0 0 rg\n";
    for (const auto& r : g.black_rects) {
        double x_pt = (m.offset_x_mm + r.x) * kPtPerMm;
        double y_pt = (m.page_h_mm - (m.offset_y_mm + r.y + r.h)) * kPtPerMm;
        cmds << fmt(x_pt, 4) << ' ' << fmt(y_pt, 4) << ' '
             << fmt(r.w * kPtPerMm, 4) << ' ' << fmt(r.h * kPtPerMm, 4) << " re f\n";
    }
    for (const auto& c : g.black_circles) {
        double cx = (m.offset_x_mm + c.cx) * kPtPerMm;
        double cy = (m.page_h_mm - (m.offset_y_mm + c.cy)) * kPtPerMm;
        double r = c.r * kPtPerMm;
        double k = r * kKappa;
        cmds << fmt(cx + r, 4) << ' ' << fmt(cy, 4) << " m "
             << fmt(cx + r, 4) << ' ' << fmt(cy + k, 4) << ' ' << fmt(cx + k, 4) << ' ' << fmt(cy + r, 4) << ' ' << fmt(cx, 4) << ' ' << fmt(cy + r, 4) << " c "
             << fmt(cx - k, 4) << ' ' << fmt(cy + r, 4) << ' ' << fmt(cx - r, 4) << ' ' << fmt(cy + k, 4) << ' ' << fmt(cx - r, 4) << ' ' << fmt(cy, 4) << " c "
             << fmt(cx - r, 4) << ' ' << fmt(cy - k, 4) << ' ' << fmt(cx - k, 4) << ' ' << fmt(cy - r, 4) << ' ' << fmt(cx, 4) << ' ' << fmt(cy - r, 4) << " c "
             << fmt(cx + k, 4) << ' ' << fmt(cy - r, 4) << ' ' << fmt(cx + r, 4) << ' ' << fmt(cy - k, 4) << ' ' << fmt(cx + r, 4) << ' ' << fmt(cy, 4) << " c f\n";
    }
    std::string safe_label = m.summary;
    std::replace(safe_label.begin(), safe_label.end(), '(', '[');
    std::replace(safe_label.begin(), safe_label.end(), ')', ']');
    double footer_x = 12.0 * kPtPerMm;
    double footer_y = std::max(12.0, (m.page_h_mm - (m.offset_y_mm + m.pattern_h_mm + 7.0)) * kPtPerMm);
    cmds << "BT /F1 9 Tf " << fmt(footer_x, 2) << ' ' << fmt(footer_y, 2)
         << " Td (" << safe_label << " | 100% physical scale) Tj ET";

    std::string stream_bytes = cmds.str();
    std::vector<std::string> objs = {
        "1 0 obj << /Type /Catalog /Pages 2 0 R >> endobj\n",
        "2 0 obj << /Type /Pages /Kids [3 0 R] /Count 1 >> endobj\n",
        "3 0 obj << /Type /Page /Parent 2 0 R /MediaBox [0 0 " + fmt(w_pt, 4) + " " + fmt(h_pt, 4) +
            "] /Resources << /Font << /F1 4 0 R >> >> /Contents 5 0 R >> endobj\n",
        "4 0 obj << /Type /Font /Subtype /Type1 /BaseFont /Helvetica >> endobj\n",
        "5 0 obj << /Length " + std::to_string(stream_bytes.size()) + " >>\nstream\n" + stream_bytes + "\nendstream\nendobj\n",
    };

    std::string buf = "%PDF-1.4\n%\xe2\xe3\xcf\xd3\n";
    std::vector<size_t> offsets;
    for (const auto& obj : objs) {
        offsets.push_back(buf.size());
        buf += obj;
    }
    size_t xref_pos = buf.size();
    buf += "xref\n0 " + std::to_string(objs.size() + 1) + "\n0000000000 65535 f \n";
    for (size_t off : offsets) {
        std::ostringstream line;
        line << std::setw(10) << std::setfill('0') << off << " 00000 n \n";
        buf += line.str();
    }
    buf += "trailer << /Size " + std::to_string(objs.size() + 1) + " /Root 1 0 R >>\nstartxref\n" +
           std::to_string(xref_pos) + "\n%%EOF\n";
    return buf;
}

std::string build_svg_text(const BoardGeometry& g, double margin_mm) {
    const auto& m = g.meta;
    std::ostringstream os;
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
       << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << fmt(m.page_w_mm, 3)
       << "mm\" height=\"" << fmt(m.page_h_mm, 3) << "mm\" viewBox=\"0 0 "
       << fmt(m.page_w_mm, 3) << ' ' << fmt(m.page_h_mm, 3) << "\">\n"
       << "  <rect x=\"0\" y=\"0\" width=\"" << fmt(m.page_w_mm, 3)
       << "\" height=\"" << fmt(m.page_h_mm, 3) << "\" fill=\"#ffffff\"/>\n"
       << "  <g fill=\"#000000\" stroke=\"none\">\n";
    for (const auto& r : g.black_rects) {
        os << "    <rect x=\"" << fmt(m.offset_x_mm + r.x, 4)
           << "\" y=\"" << fmt(m.offset_y_mm + r.y, 4)
           << "\" width=\"" << fmt(r.w, 4)
           << "\" height=\"" << fmt(r.h, 4) << "\"/>\n";
    }
    for (const auto& c : g.black_circles) {
        os << "    <circle cx=\"" << fmt(m.offset_x_mm + c.cx, 4)
           << "\" cy=\"" << fmt(m.offset_y_mm + c.cy, 4)
           << "\" r=\"" << fmt(c.r, 4) << "\"/>\n";
    }
    double fy = m.offset_y_mm + m.pattern_h_mm + 6.5;
    os << "  </g>\n"
       << "  <text x=\"" << fmt(margin_mm, 2) << "\" y=\"" << fmt(fy, 2)
       << "\" font-family=\"sans-serif\" font-size=\"3.2\" fill=\"#333333\">"
       << m.summary << " | 1:1 physical print scale</text>\n</svg>\n";
    return os.str();
}

uint32_t crc32_ieee(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

void append_be32(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>((v >> 24) & 0xFFu));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFFu));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFFu));
    out.push_back(static_cast<uint8_t>(v & 0xFFu));
}

std::vector<uint8_t> inject_png_dpi(const std::vector<uint8_t>& png, int dpi) {
    static const uint8_t kSig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    if (png.size() < 33 || !std::equal(std::begin(kSig), std::end(kSig), png.begin())) {
        return png;
    }
    uint32_t ppm = static_cast<uint32_t>(std::llround(static_cast<double>(dpi) / 0.0254));
    std::vector<uint8_t> chunk_body;
    chunk_body.reserve(13);
    chunk_body.push_back('p'); chunk_body.push_back('H'); chunk_body.push_back('Y'); chunk_body.push_back('s');
    append_be32(chunk_body, ppm);
    append_be32(chunk_body, ppm);
    chunk_body.push_back(1);
    uint32_t crc = crc32_ieee(chunk_body.data(), chunk_body.size());

    uint32_t ihdr_len = (static_cast<uint32_t>(png[8]) << 24) |
                        (static_cast<uint32_t>(png[9]) << 16) |
                        (static_cast<uint32_t>(png[10]) << 8) |
                        static_cast<uint32_t>(png[11]);
    size_t insert_pos = 8 + 4 + 4 + static_cast<size_t>(ihdr_len) + 4;
    if (insert_pos > png.size()) return png;

    std::vector<uint8_t> out;
    out.reserve(png.size() + 21);
    out.insert(out.end(), png.begin(), png.begin() + static_cast<ptrdiff_t>(insert_pos));
    append_be32(out, 9);
    out.insert(out.end(), chunk_body.begin(), chunk_body.end());
    append_be32(out, crc);
    out.insert(out.end(), png.begin() + static_cast<ptrdiff_t>(insert_pos), png.end());
    return out;
}

}  // namespace

const std::vector<PaperSize>& standard_paper_sizes() {
    static const std::vector<PaperSize> kSizes = {
        {"A4", 297.0, 210.0},
        {"A3", 420.0, 297.0},
        {"A2", 594.0, 420.0},
        {"Letter", 279.4, 215.9},
    };
    return kSizes;
}

std::string normalize_target_type(const std::string& target_type) {
    std::string t = trim_lower(target_type);
    if (t == "aprilgrid" || t == "april" || t == "apriltag") return "aprilgrid";
    if (t == "chessboard" || t == "checkerboard" || t == "棋盘格") return "chessboard";
    if (t == "circles" || t == "circle" || t == "symmetric_circles" || t == "circle_symmetric" ||
        t == "对称圆" || t == "圆标定板") {
        return "circles";
    }
    if (t == "acircles" || t == "asymmetric_circles" || t == "circle_asymmetric" || t == "非对称圆") {
        return "acircles";
    }
    throw std::invalid_argument("不支持的标定板类型: " + target_type);
}

RenderedBoard render_calibration_board(
    const std::string& target_type,
    int rows,
    int cols,
    double size_mm,
    double spacing_ratio,
    const std::string& paper,
    int dpi,
    const std::string& orientation,
    double margin_mm,
    const std::array<double, 2>* custom_page_mm) {
    if (dpi < 36 || dpi > 1200) throw std::invalid_argument("DPI 必须在 36 到 1200 之间");
    BoardGeometry g = compute_board_geometry(target_type, rows, cols, size_mm, spacing_ratio);
    auto [page_w, page_h] = resolve_page_size(paper, orientation, g.meta.pattern_w_mm, g.meta.pattern_h_mm, margin_mm, custom_page_mm);

    double px_per_mm = static_cast<double>(dpi) / 25.4;
    int img_w = std::max(64, static_cast<int>(std::llround(page_w * px_per_mm)));
    int img_h = std::max(64, static_cast<int>(std::llround(page_h * px_per_mm)));
    cv::Mat img(img_h, img_w, CV_8UC1, cv::Scalar(255));

    double off_x_mm = (page_w - g.meta.pattern_w_mm) * 0.5;
    double off_y_mm = std::max(margin_mm, (page_h - 10.0 - g.meta.pattern_h_mm) * 0.5);

    for (const auto& r : g.black_rects) {
        int x0 = std::max(0, static_cast<int>(std::llround((off_x_mm + r.x) * px_per_mm)));
        int y0 = std::max(0, static_cast<int>(std::llround((off_y_mm + r.y) * px_per_mm)));
        int x1 = std::min(img_w, static_cast<int>(std::llround((off_x_mm + r.x + r.w) * px_per_mm)));
        int y1 = std::min(img_h, static_cast<int>(std::llround((off_y_mm + r.y + r.h) * px_per_mm)));
        if (x1 > x0 && y1 > y0) {
            img(cv::Rect(x0, y0, x1 - x0, y1 - y0)).setTo(0);
        }
    }
    for (const auto& c : g.black_circles) {
        cv::Point center(
            static_cast<int>(std::llround((off_x_mm + c.cx) * px_per_mm)),
            static_cast<int>(std::llround((off_y_mm + c.cy) * px_per_mm)));
        int rad = std::max(2, static_cast<int>(std::llround(c.r * px_per_mm)));
        cv::circle(img, center, rad, cv::Scalar(0), -1, cv::LINE_AA);
    }

    int footer_y = std::min(img_h - 8, static_cast<int>(std::llround((off_y_mm + g.meta.pattern_h_mm + 7.0) * px_per_mm)));
    int footer_x = std::max(8, static_cast<int>(std::llround(margin_mm * px_per_mm)));
    double font_scale = std::max(0.35, std::min(0.9, px_per_mm / 10.0));
    std::string label = g.meta.summary + " | Print 100% scale | Page " + fmt(page_w, 1) +
                        "x" + fmt(page_h, 1) + "mm (" + std::to_string(dpi) + " DPI)";
    cv::putText(img, label, cv::Point(footer_x, footer_y), cv::FONT_HERSHEY_SIMPLEX, font_scale,
                cv::Scalar(80), std::max(1, static_cast<int>(std::llround(font_scale * 2.0))), cv::LINE_AA);

    BoardMeta meta = g.meta;
    meta.page_w_mm = page_w;
    meta.page_h_mm = page_h;
    meta.offset_x_mm = off_x_mm;
    meta.offset_y_mm = off_y_mm;
    meta.dpi = dpi;
    meta.image_size = {img_w, img_h};
    meta.paper = paper;
    meta.orientation = orientation;
    return {img, meta};
}

std::pair<std::filesystem::path, BoardMeta> save_calibration_board(
    const std::filesystem::path& output_path,
    const std::string& target_type,
    int rows,
    int cols,
    double size_mm,
    double spacing_ratio,
    const std::string& paper,
    int dpi,
    const std::string& orientation,
    double margin_mm,
    const std::array<double, 2>* custom_page_mm) {
    std::filesystem::path out = std::filesystem::weakly_canonical(output_path);
    if (out.has_parent_path()) std::filesystem::create_directories(out.parent_path());
    std::string ext = trim_lower(out.extension().string());
    if (ext.empty()) ext = ".pdf";

    BoardGeometry g = compute_board_geometry(target_type, rows, cols, size_mm, spacing_ratio);
    auto [page_w, page_h] = resolve_page_size(paper, orientation, g.meta.pattern_w_mm, g.meta.pattern_h_mm, margin_mm, custom_page_mm);
    g.meta.page_w_mm = page_w;
    g.meta.page_h_mm = page_h;
    g.meta.offset_x_mm = (page_w - g.meta.pattern_w_mm) * 0.5;
    g.meta.offset_y_mm = std::max(margin_mm, (page_h - 10.0 - g.meta.pattern_h_mm) * 0.5);
    g.meta.dpi = dpi;
    g.meta.paper = paper;
    g.meta.orientation = orientation;
    g.meta.path = out.string();

    if (ext == ".pdf") {
        std::string bytes = build_pdf_bytes(g);
        std::ofstream f(out, std::ios::binary | std::ios::trunc);
        if (!f) throw std::runtime_error("Cannot write PDF: " + out.string());
        f.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        return {out, g.meta};
    }
    if (ext == ".svg") {
        std::string svg = build_svg_text(g, margin_mm);
        std::ofstream f(out, std::ios::binary | std::ios::trunc);
        if (!f) throw std::runtime_error("Cannot write SVG: " + out.string());
        f.write(svg.data(), static_cast<std::streamsize>(svg.size()));
        return {out, g.meta};
    }

    RenderedBoard rendered = render_calibration_board(
        target_type, rows, cols, size_mm, spacing_ratio, paper, dpi, orientation, margin_mm, custom_page_mm);
    std::vector<uint8_t> encoded;
    if (!cv::imencode(ext, rendered.image, encoded)) {
        throw std::runtime_error("无法编码标定板图像格式: " + ext);
    }
    if (ext == ".png") {
        encoded = inject_png_dpi(encoded, dpi);
    }
    std::ofstream f(out, std::ios::binary | std::ios::trunc);
    if (!f) throw std::runtime_error("Cannot write image: " + out.string());
    f.write(reinterpret_cast<const char*>(encoded.data()), static_cast<std::streamsize>(encoded.size()));
    rendered.meta.path = out.string();
    return {out, rendered.meta};
}

}  // namespace calibration_targets
