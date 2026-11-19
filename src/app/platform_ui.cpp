#include "platform_ui.hpp"

#include <opencv2/imgproc.hpp>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <shellapi.h>
#include <shobjidl.h>
#include <GL/gl.h>
#elif defined(__APPLE__)
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl3.h>
#include <cstdlib>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#else
#include <GL/gl.h>
#include <cstdlib>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#include <sstream>
#include <vector>

namespace slam_app {
namespace {

#ifdef _WIN32
std::wstring utf8_to_utf16(const std::string& s) {
    if (s.empty()) return {};
    int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    if (n <= 0) return {};
    std::wstring w(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
    return w;
}

std::string utf16_to_utf8(const std::wstring& w) {
    if (w.empty()) return {};
    int n = WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), nullptr, 0, nullptr, nullptr);
    if (n <= 0) return {};
    std::string s(static_cast<size_t>(n), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), s.data(), n, nullptr, nullptr);
    return s;
}

std::wstring build_win_filter_pattern(const char* filter_spec) {
    if (!filter_spec || filter_spec[0] == '\0') return L"*.*";
    std::string s(filter_spec);
    std::wstring out;
    size_t start = 0;
    while (start < s.size()) {
        size_t comma = s.find(',', start);
        std::string tok = (comma == std::string::npos) ? s.substr(start) : s.substr(start, comma - start);
        if (!tok.empty()) {
            if (!out.empty()) out += L";";
            out += L"*." + utf8_to_utf16(tok);
        }
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return out.empty() ? L"*.*" : out;
}

std::optional<std::filesystem::path> win_file_dialog(
    bool is_save,
    bool pick_folder,
    const char* filter_name,
    const char* filter_spec,
    const std::string& default_name) {
    HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    bool uninit = SUCCEEDED(hr);

    IFileDialog* dlg = nullptr;
    CLSID clsid = is_save ? CLSID_FileSaveDialog : CLSID_FileOpenDialog;
    IID iid = is_save ? IID_IFileSaveDialog : IID_IFileOpenDialog;
    std::optional<std::filesystem::path> result;

    if (SUCCEEDED(CoCreateInstance(clsid, nullptr, CLSCTX_ALL, iid, reinterpret_cast<void**>(&dlg)))) {
        DWORD opts = 0;
        if (SUCCEEDED(dlg->GetOptions(&opts))) {
            opts |= FOS_FORCEFILESYSTEM;
            if (pick_folder) opts |= FOS_PICKFOLDERS;
            dlg->SetOptions(opts);
        }
        std::wstring wname, wspec;
        COMDLG_FILTERSPEC spec[2]{};
        if (!pick_folder && filter_name && filter_spec && filter_spec[0] != '\0') {
            wname = utf8_to_utf16(filter_name);
            wspec = build_win_filter_pattern(filter_spec);
            spec[0] = {wname.c_str(), wspec.c_str()};
            spec[1] = {L"All Files", L"*.*"};
            dlg->SetFileTypes(2, spec);
        }
        if (is_save && !default_name.empty()) {
            std::wstring wdef = utf8_to_utf16(default_name);
            dlg->SetFileName(wdef.c_str());
        }
        if (SUCCEEDED(dlg->Show(nullptr))) {
            IShellItem* item = nullptr;
            if (SUCCEEDED(dlg->GetResult(&item))) {
                PWSTR psz = nullptr;
                if (SUCCEEDED(item->GetDisplayName(SIGDN_FILESYSPATH, &psz)) && psz) {
                    result = std::filesystem::path(std::wstring(psz));
                    CoTaskMemFree(psz);
                }
                item->Release();
            }
        }
        dlg->Release();
    }
    if (uninit) CoUninitialize();
    return result;
}
#else
std::optional<std::string> run_exec_capture_trimmed(const std::vector<std::string>& args) {
    if (args.empty()) return std::nullopt;
    int pipefd[2];
    if (pipe(pipefd) != 0) return std::nullopt;
    pid_t pid = fork();
    if (pid == 0) {
        close(pipefd[0]);
        dup2(pipefd[1], STDOUT_FILENO);
        close(pipefd[1]);
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        std::vector<char*> argv;
        argv.reserve(args.size() + 1);
        for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
        argv.push_back(nullptr);
        execvp(args[0].c_str(), argv.data());
        _exit(127);
    }
    close(pipefd[1]);
    if (pid < 0) {
        close(pipefd[0]);
        return std::nullopt;
    }
    std::string out;
    char buf[512];
    ssize_t n = 0;
    while ((n = read(pipefd[0], buf, sizeof(buf))) > 0) {
        out.append(buf, static_cast<size_t>(n));
    }
    close(pipefd[0]);
    int st = 0;
    waitpid(pid, &st, 0);
    if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) return std::nullopt;
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
    if (out.empty()) return std::nullopt;
    return out;
}

void spawn_detached(const std::vector<std::string>& args) {
    if (args.empty()) return;
    pid_t pid = fork();
    if (pid == 0) {
        setsid();
        int devnull = open("/dev/null", O_RDWR);
        if (devnull >= 0) {
            dup2(devnull, STDIN_FILENO);
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        std::vector<char*> argv;
        argv.reserve(args.size() + 1);
        for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
        argv.push_back(nullptr);
        execvp(args[0].c_str(), argv.data());
        _exit(127);
    }
}
#endif

int std_string_resize_cb(ImGuiInputTextCallbackData* data) {
    if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
        auto* str = static_cast<std::string*>(data->UserData);
        str->resize(static_cast<size_t>(data->BufTextLen));
        data->Buf = str->data();
    }
    return 0;
}

}  // namespace

void init_platform_ui() {}

void shutdown_platform_ui() {}

std::optional<std::filesystem::path> pick_directory(const std::filesystem::path&) {
#ifdef _WIN32
    return win_file_dialog(false, true, nullptr, nullptr, "");
#elif defined(__APPLE__)
    if (auto s = run_exec_capture_trimmed({"osascript", "-e", "POSIX path of (choose folder)"})) {
        return std::filesystem::u8path(*s);
    }
    return std::nullopt;
#else
    if (auto s = run_exec_capture_trimmed({"zenity", "--file-selection", "--directory"})) {
        return std::filesystem::u8path(*s);
    }
    return std::nullopt;
#endif
}

std::optional<std::filesystem::path> pick_open_file(
    const char* filter_name,
    const char* filter_spec,
    const std::filesystem::path&) {
#ifdef _WIN32
    return win_file_dialog(false, false, filter_name, filter_spec, "");
#elif defined(__APPLE__)
    (void)filter_name; (void)filter_spec;
    if (auto s = run_exec_capture_trimmed({"osascript", "-e", "POSIX path of (choose file)"})) {
        return std::filesystem::u8path(*s);
    }
    return std::nullopt;
#else
    (void)filter_name; (void)filter_spec;
    if (auto s = run_exec_capture_trimmed({"zenity", "--file-selection"})) {
        return std::filesystem::u8path(*s);
    }
    return std::nullopt;
#endif
}

std::optional<std::filesystem::path> pick_save_file(
    const char* filter_name,
    const char* filter_spec,
    const std::string& default_name) {
#ifdef _WIN32
    return win_file_dialog(true, false, filter_name, filter_spec, default_name);
#elif defined(__APPLE__)
    (void)filter_name; (void)filter_spec;
    if (default_name.empty()) {
        if (auto s = run_exec_capture_trimmed({"osascript", "-e", "POSIX path of (choose file name)"})) {
            return std::filesystem::u8path(*s);
        }
        return std::nullopt;
    }
    if (auto s = run_exec_capture_trimmed({
            "osascript", "-e",
            "on run argv\nreturn POSIX path of (choose file name default name (item 1 of argv))\nend run",
            "--", default_name})) {
        return std::filesystem::u8path(*s);
    }
    return std::nullopt;
#else
    (void)filter_name; (void)filter_spec;
    std::vector<std::string> args = {"zenity", "--file-selection", "--save", "--confirm-overwrite"};
    if (!default_name.empty()) args.push_back("--filename=" + default_name);
    if (auto s = run_exec_capture_trimmed(args)) return std::filesystem::u8path(*s);
    return std::nullopt;
#endif
}

void open_in_os_shell(const std::filesystem::path& path) {
    if (path.empty()) return;
#ifdef _WIN32
    std::wstring w = path.wstring();
    ShellExecuteW(nullptr, L"open", w.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
#elif defined(__APPLE__)
    spawn_detached({"open", path.string()});
#else
    spawn_detached({"xdg-open", path.string()});
#endif
}

std::filesystem::path user_home_dir() {
#ifdef _WIN32
    if (const char* up = std::getenv("USERPROFILE"); up && up[0] != '\0') {
        return std::filesystem::u8path(up);
    }
#else
    if (const char* home = std::getenv("HOME"); home && home[0] != '\0') {
        return std::filesystem::u8path(home);
    }
#endif
    return std::filesystem::current_path();
}

void load_cjk_font(ImGuiIO& io, float size_px) {
    std::vector<std::string> candidates;
#ifdef _WIN32
    candidates = {
        "C:\\Windows\\Fonts\\msyh.ttc",
        "C:\\Windows\\Fonts\\msyh.ttf",
        "C:\\Windows\\Fonts\\simhei.ttf",
        "C:\\Windows\\Fonts\\simsun.ttc",
    };
#elif defined(__APPLE__)
    candidates = {
        "/System/Library/Fonts/PingFang.ttc",
        "/System/Library/Fonts/STHeiti Light.ttc",
        "/System/Library/Fonts/Hiragino Sans GB.ttc",
    };
#else
    candidates = {
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    };
#endif
    for (const auto& path : candidates) {
        if (std::filesystem::is_regular_file(std::filesystem::u8path(path))) {
            ImFont* font = io.Fonts->AddFontFromFileTTF(
                path.c_str(), size_px, nullptr, io.Fonts->GetGlyphRangesChineseFull());
            if (font != nullptr) {
                io.FontDefault = font;
                return;
            }
        }
    }
    io.Fonts->AddFontDefault();
}

void apply_station_style() {
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 5.0f;
    style.FrameRounding = 4.0f;
    style.TabRounding = 4.0f;
    style.ScrollbarRounding = 4.0f;
    style.FramePadding = ImVec2(8.0f, 4.0f);
    style.ItemSpacing = ImVec2(8.0f, 6.0f);
    ImVec4* c = style.Colors;
    c[ImGuiCol_WindowBg] = ImVec4(0.08f, 0.11f, 0.15f, 1.00f);
    c[ImGuiCol_ChildBg] = ImVec4(0.09f, 0.13f, 0.18f, 1.00f);
    c[ImGuiCol_FrameBg] = ImVec4(0.14f, 0.19f, 0.25f, 1.00f);
    c[ImGuiCol_FrameBgHovered] = ImVec4(0.18f, 0.25f, 0.33f, 1.00f);
    c[ImGuiCol_Button] = ImVec4(0.10f, 0.42f, 0.40f, 1.00f);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.14f, 0.52f, 0.49f, 1.00f);
    c[ImGuiCol_ButtonActive] = ImVec4(0.08f, 0.35f, 0.33f, 1.00f);
    c[ImGuiCol_Header] = ImVec4(0.12f, 0.40f, 0.40f, 0.80f);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.16f, 0.48f, 0.48f, 0.90f);
    c[ImGuiCol_Tab] = ImVec4(0.12f, 0.17f, 0.23f, 1.00f);
    c[ImGuiCol_TabHovered] = ImVec4(0.15f, 0.48f, 0.46f, 1.00f);
    c[ImGuiCol_TabActive] = ImVec4(0.10f, 0.42f, 0.40f, 1.00f);
}

GlTexture::~GlTexture() {
    clear();
}

void GlTexture::clear() {
    if (tex_id_ != 0) {
        GLuint id = static_cast<GLuint>(tex_id_);
        glDeleteTextures(1, &id);
        tex_id_ = 0;
    }
    width_ = 0;
    height_ = 0;
}

void GlTexture::upload(const cv::Mat& img) {
    if (img.empty()) {
        clear();
        return;
    }
    cv::Mat rgba;
    if (img.channels() == 1) cv::cvtColor(img, rgba, cv::COLOR_GRAY2RGBA);
    else if (img.channels() == 3) cv::cvtColor(img, rgba, cv::COLOR_BGR2RGBA);
    else if (img.channels() == 4) cv::cvtColor(img, rgba, cv::COLOR_BGRA2RGBA);
    else return;
    if (!rgba.isContinuous()) rgba = rgba.clone();

    if (tex_id_ == 0) {
        GLuint id = 0;
        glGenTextures(1, &id);
        tex_id_ = static_cast<uint32_t>(id);
    }
    glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(tex_id_));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgba.cols, rgba.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data);
    glBindTexture(GL_TEXTURE_2D, 0);
    width_ = rgba.cols;
    height_ = rgba.rows;
}

bool input_text_std(const char* label, std::string& str, ImGuiInputTextFlags flags) {
    flags |= ImGuiInputTextFlags_CallbackResize;
    return ImGui::InputText(label, str.data(), str.capacity() + 1, flags, std_string_resize_cb, &str);
}

bool input_text_multiline_std(const char* label, std::string& str, const ImVec2& size, ImGuiInputTextFlags flags) {
    flags |= ImGuiInputTextFlags_CallbackResize;
    return ImGui::InputTextMultiline(label, str.data(), str.capacity() + 1, size, flags, std_string_resize_cb, &str);
}

}  // namespace slam_app
