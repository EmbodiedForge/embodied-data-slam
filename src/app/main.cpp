#include "calibration_workbench.hpp"
#include "main_window.hpp"
#include "platform_ui.hpp"
#include "umi_slam/pipeline.hpp"
#include "umi_slam/runtime.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <iostream>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace {

int run_cli(const std::vector<std::string>& args) {
    try {
        bool check_only = false;
        bool wait = false;
        bool prepare_only = false;
        bool viewer = false;
        std::filesystem::path input, output, settings, device, backend_path;
        int stride = 5, max_frames = 0;
        double timeout = 0.0;

        for (size_t i = 0; i < args.size(); ++i) {
            const std::string& flag = args[i];
            auto next_val = [&](const std::string& f) {
                if (i + 1 >= args.size()) throw std::invalid_argument("Missing value for " + f);
                return args[++i];
            };
            if (flag == "--check") check_only = true;
            else if (flag == "--wait") wait = true;
            else if (flag == "--prepare-only") prepare_only = true;
            else if (flag == "--viewer") viewer = true;
            else if (flag == "--input") input = std::filesystem::u8path(next_val(flag));
            else if (flag == "--output") output = std::filesystem::u8path(next_val(flag));
            else if (flag == "--settings") settings = std::filesystem::u8path(next_val(flag));
            else if (flag == "--device") device = std::filesystem::u8path(next_val(flag));
            else if (flag == "--backend") backend_path = std::filesystem::u8path(next_val(flag));
            else if (flag == "--stride") stride = std::stoi(next_val(flag));
            else if (flag == "--max-frames") max_frames = std::stoi(next_val(flag));
            else if (flag == "--timeout") timeout = std::stod(next_val(flag));
        }

        if (check_only) {
            auto info = umi_slam::diagnose(
                backend_path.empty() ? std::nullopt : std::optional<std::filesystem::path>(backend_path));
            std::cout << info.dump(2) << std::endl;
            return 0;
        }
        if (input.empty() || output.empty() || settings.empty() || device.empty()) {
            std::cerr << "Usage: slam_station --cli [--check | --input DIR --output DIR --settings YAML --device JSON ...]\n";
            return 2;
        }
        umi_slam::Job job(
            input, output, settings, device, stride, max_frames, timeout, prepare_only,
            backend_path.empty() ? std::nullopt : std::optional<std::filesystem::path>(backend_path),
            viewer);
        if (wait) job.run();
        else { job.start(); job.wait(); }
        auto st = job.state();
        std::cout << st.dump(2) << std::endl;
        std::string s = st.get("status").is_string() ? st.get("status").as_string() : "";
        return (s == "completed" || s == "prepared") ? 0 : 1;
    } catch (const std::exception& exc) {
        std::cerr << "ERROR: " << exc.what() << std::endl;
        return 1;
    }
}

int run_imgui_gui(bool calibrator_only) {
    if (!glfwInit()) {
        std::cerr << "ERROR: Failed to initialize GLFW.\n";
        return 1;
    }

#if defined(__APPLE__)
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#else
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    const char* title = calibrator_only
        ? "SLAM Calibrator · 免 ROS 多模态与 VisionMaster 标定工作台"
        : "SLAM Station · UMI / EGO (C++17 / Dear ImGui)";
    GLFWwindow* window = glfwCreateWindow(1220, 860, title, nullptr, nullptr);
    if (!window) {
        std::cerr << "ERROR: Failed to create GLFW window.\n";
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.IniFilename = nullptr;

    slam_app::apply_station_style();
    slam_app::load_cjk_font(io, 17.0f);
    slam_app::init_platform_ui();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    {
        std::unique_ptr<slam_app::MainWindow> main_win;
        std::unique_ptr<slam_app::CalibrationPane> cal_pane;
        if (calibrator_only) cal_pane = std::make_unique<slam_app::CalibrationPane>();
        else main_win = std::make_unique<slam_app::MainWindow>();

        while (true) {
            if (glfwWindowShouldClose(window)) {
                if (!main_win || main_win->request_close()) break;
                glfwSetWindowShouldClose(window, GLFW_FALSE);
            }
            if (main_win && main_win->should_exit()) break;

            glfwPollEvents();
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            if (cal_pane) {
                const ImGuiViewport* vp = ImGui::GetMainViewport();
                ImGui::SetNextWindowPos(vp->WorkPos);
                ImGui::SetNextWindowSize(vp->WorkSize);
                ImGui::Begin("SLAM Calibrator", nullptr,
                             ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
                cal_pane->render("standalone_calibrator");
                ImGui::End();
            } else {
                main_win->render();
            }

            ImGui::Render();
            int display_w = 0, display_h = 0;
            glfwGetFramebufferSize(window, &display_w, &display_h);
            glViewport(0, 0, display_w, display_h);
            glClearColor(0.08f, 0.11f, 0.15f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window);
        }
    }

    slam_app::shutdown_platform_ui();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    if (argc > 1 && std::string(argv[1]) == "--run-pipeline") {
        std::vector<char*> sub_argv;
        sub_argv.push_back(argv[0]);
        for (int i = 2; i < argc; ++i) sub_argv.push_back(argv[i]);
        return umi_slam::run_pipeline_cli(static_cast<int>(sub_argv.size()), sub_argv.data());
    }
    if (argc > 1 && std::string(argv[1]) == "--cli") {
        std::vector<std::string> sub_args;
        for (int i = 2; i < argc; ++i) sub_args.emplace_back(argv[i]);
        return run_cli(sub_args);
    }
    bool calibrator_only = (argc > 1 && std::string(argv[1]) == "--calibrator");
    return run_imgui_gui(calibrator_only);
}
