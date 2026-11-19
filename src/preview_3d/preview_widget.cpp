#include "preview_3d/preview_widget.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>

namespace preview_3d {
namespace {

constexpr double kPi = 3.14159265358979323846;

uint32_t hex_to_abgr(const char* hex, uint8_t alpha = 255) {
    unsigned int r = 255, g = 255, b = 255;
    if (hex && hex[0] == '#') {
        std::sscanf(hex + 1, "%02x%02x%02x", &r, &g, &b);
    }
    return IM_COL32(r, g, b, alpha);
}

const std::array<const char*, 5> kSegmentColors3D = {
    "#38bdf8", "#f59e0b", "#a855f7", "#34d399", "#fb7185"
};
const std::array<const char*, 5> kSegmentColors2D = {
    "#087f8c", "#df8f2d", "#855bb5", "#438943", "#d64e52"
};
const std::array<const char*, 8> kMapZPalette = {
    "#1d4ed8", "#0284c7", "#0891b2", "#0d9488", "#16a34a", "#65a30d", "#ca8a04", "#eab308"
};

std::filesystem::path find_report_html(const std::filesystem::path& folder) {
    if (folder.empty()) return {};
    for (const auto& cand : {folder / "result" / "report.html", folder / "report.html"}) {
        if (std::filesystem::is_regular_file(cand)) return cand;
    }
    return {};
}

}  // namespace

std::pair<Eigen::Vector3d, double> compute_bounds(const std::vector<Eigen::Vector3d>& pts) {
    if (pts.empty()) return {Eigen::Vector3d::Zero(), 1.0};
    Eigen::Vector3d lo = pts[0], hi = pts[0];
    for (const auto& p : pts) {
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
    Eigen::Vector3d center = 0.5 * (lo + hi);
    Eigen::Vector3d diff = hi - lo;
    double span = std::max({diff.x(), diff.y(), diff.z(), 0.25});
    return {center, span};
}

std::vector<ColoredMapPoint> colorize_map_points(const std::vector<Eigen::Vector3d>& map_points) {
    if (map_points.empty()) return {};
    std::vector<double> zs;
    zs.reserve(map_points.size());
    for (const auto& p : map_points) zs.push_back(p.z());
    std::sort(zs.begin(), zs.end());
    size_t idx_lo = static_cast<size_t>(0.05 * static_cast<double>(zs.size() - 1));
    size_t idx_hi = static_cast<size_t>(0.95 * static_cast<double>(zs.size() - 1));
    double lo = zs[idx_lo];
    double hi = zs[idx_hi];
    double span = std::max(hi - lo, 1e-6);
    int n_bins = static_cast<int>(kMapZPalette.size()) - 1;

    std::vector<ColoredMapPoint> colored;
    colored.reserve(map_points.size());
    for (const auto& p : map_points) {
        double t = std::min(1.0, std::max(0.0, (p.z() - lo) / span));
        int bin = static_cast<int>(std::llround(t * static_cast<double>(n_bins)));
        bin = std::max(0, std::min(n_bins, bin));
        const char* hx = kMapZPalette[static_cast<size_t>(bin)];
        colored.push_back({p, hx, hex_to_abgr(hx)});
    }
    return colored;
}

Eigen::Vector2d project_3d_point(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& center,
    double cos_y,
    double sin_y,
    double cos_p,
    double sin_p,
    double scale,
    double ox,
    double oy) {
    double dx = p.x() - center.x();
    double dy = p.y() - center.y();
    double dz = p.z() - center.z();
    double x1 = cos_y * dx - sin_y * dy;
    double y1 = sin_y * dx + cos_y * dy;
    double z2 = sin_p * y1 + cos_p * dz;
    return Eigen::Vector2d(ox + x1 * scale, oy - z2 * scale);
}

void OrbitCanvas3D::set_scene(
    const slam_common::PreviewData* data,
    const std::vector<ColoredMapPoint>* colored_map,
    const std::vector<slam_common::TrajectoryPoint>* active_points,
    const Eigen::Vector3d& center,
    double span) {
    data_ = data;
    colored_map_ = colored_map;
    active_points_ = active_points;
    center_ = center;
    span_ = span;
    yaw_ = kDefaultYaw;
    pitch_ = kDefaultPitch;
    zoom_ = 1.0;
    pan_x_ = 0.0;
    pan_y_ = 0.0;
}

void OrbitCanvas3D::set_options(bool show_map, bool show_kf) {
    show_map_ = show_map;
    show_kf_ = show_kf;
}

void OrbitCanvas3D::set_view(double yaw, double pitch) {
    yaw_ = yaw;
    pitch_ = pitch;
    zoom_ = 1.0;
    pan_x_ = 0.0;
    pan_y_ = 0.0;
}

void OrbitCanvas3D::render(const char* id, float width, float height) {
    if (width < 40.0f) width = 40.0f;
    if (height < 40.0f) height = 40.0f;

    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 p1(p0.x + width, p0.y + height);
    ImGui::InvisibleButton(id, ImVec2(width, height),
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
    bool hovered = ImGui::IsItemHovered();
    bool active = ImGui::IsItemActive();

    ImGuiIO& io = ImGui::GetIO();
    dragging_ = false;
    if (active && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        dragging_ = true;
        yaw_ -= static_cast<double>(io.MouseDelta.x) * 0.008;
        pitch_ = std::max(-kPi * 0.5, std::min(kPi * 0.5, pitch_ + static_cast<double>(io.MouseDelta.y) * 0.008));
    } else if (active && (ImGui::IsMouseDragging(ImGuiMouseButton_Right) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle))) {
        dragging_ = true;
        pan_x_ += static_cast<double>(io.MouseDelta.x);
        pan_y_ += static_cast<double>(io.MouseDelta.y);
    }
    if (hovered) {
        if (io.MouseWheel != 0.0f) {
            double factor = (io.MouseWheel > 0.0f) ? 1.15 : (1.0 / 1.15);
            zoom_ = std::max(0.1, std::min(100.0, zoom_ * factor));
        }
        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            set_view(kDefaultYaw, kDefaultPitch);
        }
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(p0, p1, true);
    dl->AddRectFilled(p0, p1, hex_to_abgr("#0f1722"));
    dl->AddRect(p0, p1, hex_to_abgr("#334155"));

    if (!data_ || !active_points_ || active_points_->empty()) {
        const char* empty_msg = "尚未加载轨迹或 3D 建图数据\n点击上方“加载建图数据目录预览…”，或在运行完成后自动显示";
        ImVec2 ts = ImGui::CalcTextSize(empty_msg);
        dl->AddText(ImVec2(p0.x + (width - ts.x) * 0.5f, p0.y + (height - ts.y) * 0.5f),
                    hex_to_abgr("#94a3b8"), empty_msg);
        dl->PopClipRect();
        return;
    }

    double cos_y = std::cos(yaw_), sin_y = std::sin(yaw_);
    double cos_p = std::cos(pitch_), sin_p = std::sin(pitch_);
    double scale = (static_cast<double>(std::min(width, height)) * 0.68 / std::max(span_, 1e-4)) * zoom_;
    double ox = static_cast<double>(p0.x) + static_cast<double>(width) * 0.5 + pan_x_;
    double oy = static_cast<double>(p0.y) + static_cast<double>(height) * 0.5 + pan_y_;

    auto to_im = [](const Eigen::Vector2d& v) { return ImVec2(static_cast<float>(v.x()), static_cast<float>(v.y())); };

    double axis_len = std::max(0.15, span_ * 0.18);
    ImVec2 origin_2d = to_im(project_3d_point(Eigen::Vector3d::Zero(), center_, cos_y, sin_y, cos_p, sin_p, scale, ox, oy));
    struct AxisSpec { Eigen::Vector3d v; const char* col; const char* name; };
    for (const auto& ax : std::array<AxisSpec, 3>{{
             {Eigen::Vector3d(axis_len, 0.0, 0.0), "#ef4444", "X"},
             {Eigen::Vector3d(0.0, axis_len, 0.0), "#22c55e", "Y"},
             {Eigen::Vector3d(0.0, 0.0, axis_len), "#3b82f6", "Z"}}}) {
        ImVec2 end_2d = to_im(project_3d_point(ax.v, center_, cos_y, sin_y, cos_p, sin_p, scale, ox, oy));
        uint32_t c = hex_to_abgr(ax.col);
        dl->AddLine(origin_2d, end_2d, c, 1.8f);
        dl->AddText(ImVec2(end_2d.x + 5.0f, end_2d.y - 6.0f), c, ax.name);
    }

    if (show_map_ && colored_map_ && !colored_map_->empty()) {
        size_t limit = dragging_ ? 4000 : 18000;
        size_t step = std::max<size_t>(1, colored_map_->size() / limit);
        for (size_t i = 0; i < colored_map_->size(); i += step) {
            const auto& mp = (*colored_map_)[i];
            Eigen::Vector2d pt = project_3d_point(mp.pos, center_, cos_y, sin_y, cos_p, sin_p, scale, ox, oy);
            if (pt.x() >= p0.x - 4 && pt.x() <= p1.x + 4 && pt.y() >= p0.y - 4 && pt.y() <= p1.y + 4) {
                float x = static_cast<float>(pt.x());
                float y = static_cast<float>(pt.y());
                dl->AddRectFilled(ImVec2(x - 1.0f, y - 1.0f), ImVec2(x + 1.0f, y + 1.0f), mp.abgr);
            }
        }
    }

    if (show_kf_ && !data_->keyframes.empty()) {
        size_t limit_kf = dragging_ ? 800 : 3000;
        size_t step_kf = std::max<size_t>(1, data_->keyframes.size() / limit_kf);
        uint32_t kf_col = hex_to_abgr("#fb923c");
        for (size_t i = 0; i < data_->keyframes.size(); i += step_kf) {
            Eigen::Vector2d pt = project_3d_point(data_->keyframes[i], center_, cos_y, sin_y, cos_p, sin_p, scale, ox, oy);
            if (pt.x() >= p0.x - 4 && pt.x() <= p1.x + 4 && pt.y() >= p0.y - 4 && pt.y() <= p1.y + 4) {
                dl->AddCircleFilled(to_im(pt), 2.2f, kf_col);
            }
        }
    }

    if (!active_points_->empty()) {
        size_t color_idx = 0;
        std::string cur_seg = (*active_points_)[0].segment_id;
        std::vector<ImVec2> poly;
        poly.reserve(active_points_->size());
        poly.push_back(to_im(project_3d_point((*active_points_)[0].pos, center_, cos_y, sin_y, cos_p, sin_p, scale, ox, oy)));
        for (size_t i = 1; i < active_points_->size(); ++i) {
            const auto& tp = (*active_points_)[i];
            ImVec2 pt = to_im(project_3d_point(tp.pos, center_, cos_y, sin_y, cos_p, sin_p, scale, ox, oy));
            if (tp.segment_id != cur_seg) {
                if (poly.size() >= 2) {
                    dl->AddPolyline(poly.data(), static_cast<int>(poly.size()),
                                    hex_to_abgr(kSegmentColors3D[color_idx % kSegmentColors3D.size()]),
                                    ImDrawFlags_None, 2.2f);
                }
                ++color_idx;
                cur_seg = tp.segment_id;
                poly.clear();
                poly.push_back(pt);
            } else {
                poly.push_back(pt);
            }
        }
        if (poly.size() >= 2) {
            dl->AddPolyline(poly.data(), static_cast<int>(poly.size()),
                            hex_to_abgr(kSegmentColors3D[color_idx % kSegmentColors3D.size()]),
                            ImDrawFlags_None, 2.2f);
        }

        for (const auto& ep : std::array<std::pair<Eigen::Vector3d, const char*>, 2>{
                 {{active_points_->front().pos, "#22c55e"}, {active_points_->back().pos, "#ef4444"}}}) {
            ImVec2 pt = to_im(project_3d_point(ep.first, center_, cos_y, sin_y, cos_p, sin_p, scale, ox, oy));
            dl->AddCircleFilled(pt, 4.5f, hex_to_abgr(ep.second));
            dl->AddCircle(pt, 4.5f, IM_COL32(255, 255, 255, 255), 0, 1.3f);
        }
    }

    double hud_x = static_cast<double>(p0.x) + 44.0;
    double hud_y = static_cast<double>(p1.y) - 44.0;
    for (const auto& ax : std::array<AxisSpec, 3>{{
             {Eigen::Vector3d(1.0, 0.0, 0.0), "#ef4444", "X"},
             {Eigen::Vector3d(0.0, 1.0, 0.0), "#22c55e", "Y"},
             {Eigen::Vector3d(0.0, 0.0, 1.0), "#3b82f6", "Z"}}}) {
        Eigen::Vector2d vp = project_3d_point(ax.v, Eigen::Vector3d::Zero(), cos_y, sin_y, cos_p, sin_p, 24.0, hud_x, hud_y);
        uint32_t c = hex_to_abgr(ax.col);
        dl->AddLine(ImVec2(static_cast<float>(hud_x), static_cast<float>(hud_y)), to_im(vp), c, 2.0f);
        dl->AddText(ImVec2(static_cast<float>(vp.x() + (vp.x() - hud_x) * 0.35 - 4.0),
                           static_cast<float>(vp.y() + (vp.y() - hud_y) * 0.35 - 6.0)),
                    c, ax.name);
    }

    std::ostringstream hint;
    hint << std::fixed << std::setprecision(2)
         << "左键旋转 · 滚轮缩放 (" << zoom_ << "x) · 右键平移 · 双击复位 · 绿起/红终/橙关键帧";
    std::string hint_s = hint.str();
    ImVec2 hs = ImGui::CalcTextSize(hint_s.c_str());
    dl->AddText(ImVec2(p1.x - hs.x - 10.0f, p1.y - hs.y - 6.0f), hex_to_abgr("#94a3b8"), hint_s.c_str());

    dl->PopClipRect();
}

ProjectionCanvas2D::ProjectionCanvas2D(int ax0, int ax1, std::string title)
    : ax0_(ax0), ax1_(ax1), title_(std::move(title)) {}

void ProjectionCanvas2D::set_scene(
    const std::vector<slam_common::TrajectoryPoint>* points,
    const std::vector<Eigen::Vector3d>* map_points,
    const std::vector<Eigen::Vector3d>* keyframes) {
    points_ = points;
    map_points_ = map_points;
    keyframes_ = keyframes;
}

void ProjectionCanvas2D::set_options(bool show_map, bool show_kf) {
    show_map_ = show_map;
    show_kf_ = show_kf;
}

void ProjectionCanvas2D::render(const char* id, float width, float height) {
    if (width < 40.0f) width = 40.0f;
    if (height < 40.0f) height = 40.0f;

    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 p1(p0.x + width, p0.y + height);
    ImGui::InvisibleButton(id, ImVec2(width, height));

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(p0, p1, true);
    dl->AddRectFilled(p0, p1, IM_COL32(250, 252, 255, 255));
    dl->AddRect(p0, p1, hex_to_abgr("#cbd5e1"));

    if (!points_ || points_->empty()) {
        std::string msg = title_ + "\n暂无数据";
        ImVec2 ts = ImGui::CalcTextSize(msg.c_str());
        dl->AddText(ImVec2(p0.x + (width - ts.x) * 0.5f, p0.y + (height - ts.y) * 0.5f),
                    hex_to_abgr("#64748b"), msg.c_str());
        dl->PopClipRect();
        return;
    }

    double minx = (*points_)[0].pos(ax0_), maxx = minx;
    double miny = (*points_)[0].pos(ax1_), maxy = miny;
    for (const auto& tp : *points_) {
        minx = std::min(minx, tp.pos(ax0_));
        maxx = std::max(maxx, tp.pos(ax0_));
        miny = std::min(miny, tp.pos(ax1_));
        maxy = std::max(maxy, tp.pos(ax1_));
    }
    double spanx = maxx - minx, spany = maxy - miny;
    constexpr double margin_x = 24.0, margin_y = 20.0;
    double usable_w = std::max(1.0, static_cast<double>(width) - 2.0 * margin_x);
    double usable_h = std::max(1.0, static_cast<double>(height) - 2.0 * margin_y - 16.0);
    double scale = std::min(usable_w / std::max(spanx, 1e-4), usable_h / std::max(spany, 1e-4));
    double cx = 0.5 * (maxx + minx), cy = 0.5 * (maxy + miny);
    double ox = static_cast<double>(p0.x) + static_cast<double>(width) * 0.5;
    double oy = static_cast<double>(p0.y) + static_cast<double>(height) * 0.5 + 4.0;

    uint32_t grid_col = hex_to_abgr("#e2e8f0");
    for (int k = 1; k < 4; ++k) {
        float gx = static_cast<float>(p0.x + margin_x + usable_w * k / 4.0);
        float gy = static_cast<float>(p0.y + margin_y + (height - 2.0 * margin_y) * k / 4.0);
        dl->AddLine(ImVec2(gx, static_cast<float>(p0.y + margin_y)), ImVec2(gx, static_cast<float>(p1.y - margin_y)), grid_col);
        dl->AddLine(ImVec2(static_cast<float>(p0.x + margin_x), gy), ImVec2(static_cast<float>(p1.x - margin_x), gy), grid_col);
    }

    dl->AddText(ImVec2(p0.x + 8.0f, p0.y + 4.0f), hex_to_abgr("#1e293b"), title_.c_str());

    if (show_map_ && map_points_ && !map_points_->empty()) {
        size_t step = std::max<size_t>(1, map_points_->size() / 2500);
        uint32_t mc = hex_to_abgr("#94a3b8");
        for (size_t i = 0; i < map_points_->size(); i += step) {
            float px = static_cast<float>(ox + ((*map_points_)[i](ax0_) - cx) * scale);
            float py = static_cast<float>(oy - ((*map_points_)[i](ax1_) - cy) * scale);
            if (px >= p0.x + 2 && px <= p1.x - 2 && py >= p0.y + 16 && py <= p1.y - 12) {
                dl->AddRectFilled(ImVec2(px, py), ImVec2(px + 1.5f, py + 1.5f), mc);
            }
        }
    }

    if (show_kf_ && keyframes_ && !keyframes_->empty()) {
        size_t step_kf = std::max<size_t>(1, keyframes_->size() / 800);
        uint32_t kfc = hex_to_abgr("#f97316");
        for (size_t i = 0; i < keyframes_->size(); i += step_kf) {
            float px = static_cast<float>(ox + ((*keyframes_)[i](ax0_) - cx) * scale);
            float py = static_cast<float>(oy - ((*keyframes_)[i](ax1_) - cy) * scale);
            if (px >= p0.x + 2 && px <= p1.x - 2 && py >= p0.y + 16 && py <= p1.y - 12) {
                dl->AddCircleFilled(ImVec2(px, py), 1.6f, kfc);
            }
        }
    }

    size_t color_idx = 0;
    std::string cur_seg = (*points_)[0].segment_id;
    std::vector<ImVec2> poly;
    poly.reserve(points_->size());
    poly.push_back(ImVec2(static_cast<float>(ox + ((*points_)[0].pos(ax0_) - cx) * scale),
                          static_cast<float>(oy - ((*points_)[0].pos(ax1_) - cy) * scale)));
    for (size_t i = 1; i < points_->size(); ++i) {
        const auto& tp = (*points_)[i];
        ImVec2 pt(static_cast<float>(ox + (tp.pos(ax0_) - cx) * scale),
                  static_cast<float>(oy - (tp.pos(ax1_) - cy) * scale));
        if (tp.segment_id != cur_seg) {
            if (poly.size() >= 2) {
                dl->AddPolyline(poly.data(), static_cast<int>(poly.size()),
                                hex_to_abgr(kSegmentColors2D[color_idx % kSegmentColors2D.size()]),
                                ImDrawFlags_None, 2.0f);
            }
            ++color_idx;
            cur_seg = tp.segment_id;
            poly.clear();
            poly.push_back(pt);
        } else {
            poly.push_back(pt);
        }
    }
    if (poly.size() >= 2) {
        dl->AddPolyline(poly.data(), static_cast<int>(poly.size()),
                        hex_to_abgr(kSegmentColors2D[color_idx % kSegmentColors2D.size()]),
                        ImDrawFlags_None, 2.0f);
    }

    std::ostringstream rng;
    rng << std::fixed << std::setprecision(2) << spanx << " x " << spany << " m";
    std::string rng_s = rng.str();
    ImVec2 rs = ImGui::CalcTextSize(rng_s.c_str());
    dl->AddText(ImVec2(p1.x - rs.x - 6.0f, p1.y - rs.y - 4.0f), hex_to_abgr("#64748b"), rng_s.c_str());

    dl->PopClipRect();
}

PreviewPane::PreviewPane(
    PopoutCallback on_popout,
    PickDirectoryCallback on_pick_dir,
    OpenPathCallback on_open_path)
    : on_popout_(std::move(on_popout)),
      on_pick_dir_(std::move(on_pick_dir)),
      on_open_path_(std::move(on_open_path)),
      xy_(0, 1, "XY 俯视 (m)"),
      xz_(0, 2, "XZ 侧视 (m)"),
      yz_(1, 2, "YZ 正视 (m)") {}

bool PreviewPane::load_path(const std::filesystem::path& path, bool silent) {
    try {
        slam_common::PreviewData loaded = slam_common::load_preview_data(path);
        current_path_ = std::filesystem::weakly_canonical(path);
        data_ = std::move(loaded);
        colored_map_ = colorize_map_points(data_->map_points);
        show_target_ = !data_->target_points.empty();
        last_error_.clear();
        apply_scene();
        return true;
    } catch (const std::exception& exc) {
        last_error_ = exc.what();
        if (!silent) {
            info_text_ = std::string("加载失败：") + exc.what();
        }
        return false;
    }
}

void PreviewPane::apply_scene() {
    if (!data_.has_value()) return;
    active_points_ = (show_target_ && !data_->target_points.empty()) ? data_->target_points : data_->points;

    std::vector<Eigen::Vector3d> all_pts;
    all_pts.reserve(active_points_.size() + data_->map_points.size());
    for (const auto& tp : active_points_) all_pts.push_back(tp.pos);
    if (show_map_) {
        size_t step = std::max<size_t>(1, data_->map_points.size() / 4000);
        for (size_t i = 0; i < data_->map_points.size(); i += step) all_pts.push_back(data_->map_points[i]);
    }
    auto [c, s] = compute_bounds(all_pts);
    center_ = c;
    span_ = s;

    orbit_.set_scene(&(*data_), &colored_map_, &active_points_, center_, span_);
    orbit_.set_options(show_map_, show_kf_);
    xy_.set_scene(&active_points_, &data_->map_points, &data_->keyframes);
    xz_.set_scene(&active_points_, &data_->map_points, &data_->keyframes);
    yz_.set_scene(&active_points_, &data_->map_points, &data_->keyframes);
    xy_.set_options(show_map_, show_kf_);
    xz_.set_options(show_map_, show_kf_);
    yz_.set_options(show_map_, show_kf_);

    std::ostringstream os;
    os << current_path_.filename().string()
       << " · 轨迹点 " << active_points_.size()
       << " · 关键帧 " << data_->keyframes.size()
       << " · 地图点 " << data_->map_points.size();
    if (data_->quality.contains("coverage_fraction") && data_->quality.get("coverage_fraction").is_number()) {
        os << " · 覆盖率 " << std::fixed << std::setprecision(1)
           << (data_->quality.get("coverage_fraction").as_double() * 100.0) << "%";
    }
    info_text_ = os.str();
}

void PreviewPane::render(const char* id_prefix) {
    ImGui::PushID(id_prefix);

    if (on_pick_dir_ && ImGui::Button("加载建图数据目录预览…")) {
        if (auto dir = on_pick_dir_(); dir.has_value()) {
            load_path(*dir, false);
        }
    }
    if (on_pick_dir_) ImGui::SameLine();

    auto rep = find_report_html(current_path_);
    ImGui::BeginDisabled(rep.empty() || !on_open_path_);
    if (ImGui::Button("打开 HTML 报告") && !rep.empty() && on_open_path_) {
        on_open_path_(rep);
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (ImGui::Button("复位视角")) orbit_.set_view(OrbitCanvas3D::kDefaultYaw, OrbitCanvas3D::kDefaultPitch);
    ImGui::SameLine();
    if (ImGui::Button("XY 俯视")) orbit_.set_view(0.0, kPi * 0.5);
    ImGui::SameLine();
    if (ImGui::Button("XZ 侧视")) orbit_.set_view(0.0, 0.0);
    ImGui::SameLine();
    if (ImGui::Button("YZ 正视")) orbit_.set_view(kPi * 0.5, 0.0);

    bool opts_changed = false;
    ImGui::SameLine();
    opts_changed |= ImGui::Checkbox("地图点", &show_map_);
    ImGui::SameLine();
    opts_changed |= ImGui::Checkbox("关键帧", &show_kf_);
    if (data_.has_value() && !data_->target_points.empty()) {
        ImGui::SameLine();
        if (ImGui::Checkbox("目标系轨迹", &show_target_)) {
            apply_scene();
        }
    }
    if (opts_changed) {
        orbit_.set_options(show_map_, show_kf_);
        xy_.set_options(show_map_, show_kf_);
        xz_.set_options(show_map_, show_kf_);
        yz_.set_options(show_map_, show_kf_);
    }

    if (on_popout_) {
        ImGui::SameLine();
        if (ImGui::Button("独立窗口预览")) on_popout_();
    }

    ImGui::TextDisabled("%s", info_text_.c_str());

    ImVec2 avail = ImGui::GetContentRegionAvail();
    float spacing = ImGui::GetStyle().ItemSpacing.x;
    float left_w = std::max(260.0f, avail.x * 0.66f - spacing);
    float right_w = std::max(180.0f, avail.x - left_w - spacing);
    float total_h = std::max(240.0f, avail.y);

    ImGui::BeginGroup();
    orbit_.render("##orbit3d", left_w, total_h);
    ImGui::EndGroup();

    ImGui::SameLine();
    ImGui::BeginGroup();
    float vsp = ImGui::GetStyle().ItemSpacing.y;
    float sub_h = std::max(70.0f, (total_h - 2.0f * vsp) / 3.0f);
    xy_.render("##proj_xy", right_w, sub_h);
    xz_.render("##proj_xz", right_w, sub_h);
    yz_.render("##proj_yz", right_w, sub_h);
    ImGui::EndGroup();

    ImGui::PopID();
}

}  // namespace preview_3d
