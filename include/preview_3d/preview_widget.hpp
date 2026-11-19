#pragma once

#include "slam_common/trajectory_io.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace preview_3d {

struct ColoredMapPoint {
    Eigen::Vector3d pos;
    std::string hex_color;
    uint32_t abgr = 0xFFFFFFFFu;
};

/// Return (center_xyz, span) for a list of 3D coordinates.
std::pair<Eigen::Vector3d, double> compute_bounds(const std::vector<Eigen::Vector3d>& pts);

/// Assign each 3D map point a palette color using 5th-95th Z percentiles.
std::vector<ColoredMapPoint> colorize_map_points(const std::vector<Eigen::Vector3d>& map_points);

/// Project a 3D point onto 2D screen coordinates for yaw/pitch orbit camera.
Eigen::Vector2d project_3d_point(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& center,
    double cos_y,
    double sin_y,
    double cos_p,
    double sin_p,
    double scale,
    double ox,
    double oy);

class OrbitCanvas3D {
public:
    OrbitCanvas3D() = default;

    void set_scene(
        const slam_common::PreviewData* data,
        const std::vector<ColoredMapPoint>* colored_map,
        const std::vector<slam_common::TrajectoryPoint>* active_points,
        const Eigen::Vector3d& center,
        double span);

    void set_options(bool show_map, bool show_kf);
    void set_view(double yaw, double pitch);
    void render(const char* id, float width, float height);

    static constexpr double kDefaultYaw = -0.65;
    static constexpr double kDefaultPitch = 0.55;

private:
    const slam_common::PreviewData* data_ = nullptr;
    const std::vector<ColoredMapPoint>* colored_map_ = nullptr;
    const std::vector<slam_common::TrajectoryPoint>* active_points_ = nullptr;
    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    double span_ = 1.0;
    double yaw_ = kDefaultYaw;
    double pitch_ = kDefaultPitch;
    double zoom_ = 1.0;
    double pan_x_ = 0.0;
    double pan_y_ = 0.0;
    bool show_map_ = true;
    bool show_kf_ = true;
    bool dragging_ = false;
};

class ProjectionCanvas2D {
public:
    ProjectionCanvas2D(int ax0, int ax1, std::string title);

    void set_scene(
        const std::vector<slam_common::TrajectoryPoint>* points,
        const std::vector<Eigen::Vector3d>* map_points,
        const std::vector<Eigen::Vector3d>* keyframes);
    void set_options(bool show_map, bool show_kf);
    void render(const char* id, float width, float height);

private:
    int ax0_ = 0;
    int ax1_ = 1;
    std::string title_;
    const std::vector<slam_common::TrajectoryPoint>* points_ = nullptr;
    const std::vector<Eigen::Vector3d>* map_points_ = nullptr;
    const std::vector<Eigen::Vector3d>* keyframes_ = nullptr;
    bool show_map_ = true;
    bool show_kf_ = true;
};

/// Interactive Dear ImGui 3D map/trajectory orbit viewer paired with XY/XZ/YZ orthogonal projections.
class PreviewPane {
public:
    using PopoutCallback = std::function<void()>;
    using PickDirectoryCallback = std::function<std::optional<std::filesystem::path>()>;
    using OpenPathCallback = std::function<void(const std::filesystem::path&)>;

    explicit PreviewPane(
        PopoutCallback on_popout = nullptr,
        PickDirectoryCallback on_pick_dir = nullptr,
        OpenPathCallback on_open_path = nullptr);

    bool load_path(const std::filesystem::path& path, bool silent = false);
    const std::filesystem::path& current_path() const { return current_path_; }
    const std::string& last_error() const { return last_error_; }

    /// Render the full preview pane inside the current ImGui window/tab.
    void render(const char* id_prefix = "preview");

private:
    void apply_scene();

    PopoutCallback on_popout_;
    PickDirectoryCallback on_pick_dir_;
    OpenPathCallback on_open_path_;

    std::filesystem::path current_path_;
    std::optional<slam_common::PreviewData> data_;
    std::vector<ColoredMapPoint> colored_map_;
    std::vector<slam_common::TrajectoryPoint> active_points_;
    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    double span_ = 1.0;

    bool show_map_ = true;
    bool show_kf_ = true;
    bool show_target_ = false;
    std::string info_text_ = "尚未加载建图结果";
    std::string last_error_;

    OrbitCanvas3D orbit_;
    ProjectionCanvas2D xy_;
    ProjectionCanvas2D xz_;
    ProjectionCanvas2D yz_;
};

}  // namespace preview_3d
