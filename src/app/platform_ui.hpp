#pragma once

#include <imgui.h>
#include <opencv2/core.hpp>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace slam_app {

/// Initialize native file dialog subsystem.
void init_platform_ui();

/// Shutdown native file dialog subsystem.
void shutdown_platform_ui();

/// Native OS directory picker (UTF-8 path).
std::optional<std::filesystem::path> pick_directory(const std::filesystem::path& default_dir = {});

/// Native OS open-file picker (filter_spec e.g. "json,yaml,yml" or "png,jpg,bmp").
std::optional<std::filesystem::path> pick_open_file(
    const char* filter_name = nullptr,
    const char* filter_spec = nullptr,
    const std::filesystem::path& default_dir = {});

/// Native OS save-file picker.
std::optional<std::filesystem::path> pick_save_file(
    const char* filter_name = nullptr,
    const char* filter_spec = nullptr,
    const std::string& default_name = "");

/// Open a local file or directory in the OS default handler (Explorer/Finder/xdg-open).
void open_in_os_shell(const std::filesystem::path& path);

/// Return the current user's home directory.
std::filesystem::path user_home_dir();

/// Load a system CJK font into ImGuiIO (Windows / macOS / Linux); falls back to default font.
void load_cjk_font(ImGuiIO& io, float size_px = 17.0f);

/// Apply a crisp engineering dark theme to ImGui.
void apply_station_style();

/// RAII OpenGL 2D texture for displaying cv::Mat previews via ImGui::Image.
class GlTexture {
public:
    GlTexture() = default;
    ~GlTexture();
    GlTexture(const GlTexture&) = delete;
    GlTexture& operator=(const GlTexture&) = delete;

    void upload(const cv::Mat& bgr_or_gray);
    void clear();

    bool valid() const { return tex_id_ != 0 && width_ > 0 && height_ > 0; }
    ImTextureID imgui_id() const { return static_cast<ImTextureID>(static_cast<uintptr_t>(tex_id_)); }
    int width() const { return width_; }
    int height() const { return height_; }

private:
    uint32_t tex_id_ = 0;
    int width_ = 0;
    int height_ = 0;
};

/// Dynamic-length ImGui::InputText wrapper for std::string.
bool input_text_std(const char* label, std::string& str, ImGuiInputTextFlags flags = 0);

/// Dynamic-length ImGui::InputTextMultiline wrapper for std::string.
bool input_text_multiline_std(
    const char* label,
    std::string& str,
    const ImVec2& size = ImVec2(0, 0),
    ImGuiInputTextFlags flags = 0);

}  // namespace slam_app
