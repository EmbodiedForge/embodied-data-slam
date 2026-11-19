cmake_minimum_required(VERSION 3.20)
project(embodied_data_slam_cpp LANGUAGES CXX)

option(EDS_BUILD_STATION "Build Dear ImGui station GUI, calibration libraries and station tests" ON)

set(EDS_UTF8_RC "")
if(MSVC)
    set(EDS_UTF8_RC ${CMAKE_CURRENT_SOURCE_DIR}/cmake/windows_utf8.manifest)
elseif(WIN32)
    enable_language(RC)
    set(EDS_UTF8_RC ${CMAKE_CURRENT_SOURCE_DIR}/cmake/windows_utf8.rc)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/utf-8 /W3 /bigobj /EHsc)
    add_compile_definitions(NOMINMAX _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
elseif(APPLE)
    set(CMAKE_BUILD_RPATH "@loader_path")
    set(CMAKE_INSTALL_RPATH "@loader_path")
elseif(UNIX)
    set(CMAKE_BUILD_RPATH "\$ORIGIN")
    set(CMAKE_INSTALL_RPATH "\$ORIGIN")
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
if(EDS_BUILD_STATION)
    find_package(imgui REQUIRED)
    find_package(glfw3 REQUIRED)
    find_package(OpenGL REQUIRED)
endif()

# ---------------------------------------------------------------------------
# Leaf Library: slam_common (C++17 stdlib + Eigen3 ONLY; zero OpenCV/ImGui/GLFW)
# ---------------------------------------------------------------------------
add_library(slam_common STATIC
    src/slam_common/sha256.cpp
    src/slam_common/json.cpp
    src/slam_common/geometry.cpp
    src/slam_common/ms901m.cpp
    src/slam_common/trajectory_io.cpp
)
target_include_directories(slam_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(slam_common PUBLIC Eigen3::Eigen)

if(EDS_BUILD_STATION)
    # -----------------------------------------------------------------------
    # Dear ImGui GLFW + OpenGL3 Backend (copied by conanfile.py into generators)
    # -----------------------------------------------------------------------
    set(EDS_IMGUI_BINDINGS_DIR "${CMAKE_BINARY_DIR}/generators/imgui_bindings" CACHE PATH "ImGui backend bindings directory")
    if(NOT EXISTS "${EDS_IMGUI_BINDINGS_DIR}/imgui_impl_glfw.cpp")
        # Fallback for multi-config / custom generator directories
        if(EXISTS "${CMAKE_BINARY_DIR}/imgui_bindings/imgui_impl_glfw.cpp")
            set(EDS_IMGUI_BINDINGS_DIR "${CMAKE_BINARY_DIR}/imgui_bindings")
        elseif(DEFINED CMAKE_TOOLCHAIN_FILE)
            get_filename_component(_tc_dir "${CMAKE_TOOLCHAIN_FILE}" DIRECTORY)
            if(EXISTS "${_tc_dir}/imgui_bindings/imgui_impl_glfw.cpp")
                set(EDS_IMGUI_BINDINGS_DIR "${_tc_dir}/imgui_bindings")
            endif()
        endif()
    endif()

    add_library(imgui_backends STATIC
        "${EDS_IMGUI_BINDINGS_DIR}/imgui_impl_glfw.cpp"
        "${EDS_IMGUI_BINDINGS_DIR}/imgui_impl_opengl3.cpp"
    )
    target_include_directories(imgui_backends PUBLIC "${EDS_IMGUI_BINDINGS_DIR}")
    target_link_libraries(imgui_backends PUBLIC imgui::imgui glfw OpenGL::GL)
    if(APPLE)
        target_compile_definitions(imgui_backends PUBLIC GL_SILENCE_DEPRECATION)
    endif()

    # -----------------------------------------------------------------------
    # 5 Isolated Functional Libraries (zero cross-dependencies)
    # -----------------------------------------------------------------------
    add_library(calibration_targets STATIC
        src/calibration_targets/generator.cpp
        src/calibration_targets/detector.cpp
    )
    target_include_directories(calibration_targets PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(calibration_targets PUBLIC slam_common ${OpenCV_LIBS})

    add_library(camera_imu_calibration STATIC
        src/camera_imu_calibration/engine.cpp
    )
    target_include_directories(camera_imu_calibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(camera_imu_calibration PUBLIC slam_common ${OpenCV_LIBS})

    add_library(handeye_calibration STATIC
        src/handeye_calibration/n_point_2d.cpp
        src/handeye_calibration/handeye_3d.cpp
    )
    target_include_directories(handeye_calibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(handeye_calibration PUBLIC slam_common ${OpenCV_LIBS})

    add_library(vision_calibration STATIC
        src/vision_calibration/planar_and_distortion.cpp
    )
    target_include_directories(vision_calibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(vision_calibration PUBLIC slam_common ${OpenCV_LIBS})

    add_library(preview_3d STATIC
        include/preview_3d/preview_widget.hpp
        src/preview_3d/preview_widget.cpp
    )
    target_include_directories(preview_3d PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(preview_3d PUBLIC slam_common imgui::imgui)
endif()

# ---------------------------------------------------------------------------
# Upper Pipeline & Runtime (Pure Native; no Docker)
# ---------------------------------------------------------------------------
add_library(umi_slam_core STATIC
    src/umi_slam/pipeline.cpp
    src/umi_slam/runtime.cpp
    src/umi_slam/build_backend.cpp
    src/umi_slam/process_util.cpp
)
target_include_directories(umi_slam_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(umi_slam_core PUBLIC slam_common ${OpenCV_LIBS})
if(DEFINED CMAKE_TOOLCHAIN_FILE AND NOT "${CMAKE_TOOLCHAIN_FILE}" STREQUAL "")
    file(TO_CMAKE_PATH "${CMAKE_TOOLCHAIN_FILE}" _eds_tc_cmake)
    target_compile_definitions(umi_slam_core PRIVATE EDS_CONAN_TOOLCHAIN_FILE="${_eds_tc_cmake}")
endif()

add_executable(umi_slam_pipeline
    src/umi_slam/pipeline_main.cpp
    ${EDS_UTF8_RC}
)
target_link_libraries(umi_slam_pipeline PRIVATE umi_slam_core)

if(EDS_BUILD_STATION)
    # -----------------------------------------------------------------------
    # Dear ImGui Desktop Application + CLI
    # -----------------------------------------------------------------------
    add_executable(slam_station
        src/app/platform_ui.hpp
        src/app/platform_ui.cpp
        src/app/calibration_workbench.hpp
        src/app/calibration_workbench.cpp
        src/app/main_window.hpp
        src/app/main_window.cpp
        src/app/main.cpp
        ${EDS_UTF8_RC}
    )
    target_include_directories(slam_station PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/app)
    target_link_libraries(slam_station PRIVATE
        slam_common
        calibration_targets
        camera_imu_calibration
        handeye_calibration
        vision_calibration
        preview_3d
        umi_slam_core
        imgui_backends
        ${OpenCV_LIBS}
    )
    if(WIN32)
        target_link_libraries(slam_station PRIVATE shell32 ole32)
    endif()
endif()

# ---------------------------------------------------------------------------
# CTest Suite
# ---------------------------------------------------------------------------
include(CTest)

if(BUILD_TESTING)
    add_executable(test_pipeline_cpp tests/test_pipeline_cpp.cpp ${EDS_UTF8_RC})
    target_compile_definitions(test_pipeline_cpp PRIVATE
        EDS_REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
        EDS_PIPELINE_EXE="$<TARGET_FILE:umi_slam_pipeline>"
        EDS_CMAKE_COMMAND="${CMAKE_COMMAND}"
    )
    target_include_directories(test_pipeline_cpp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(test_pipeline_cpp PRIVATE
        slam_common
        umi_slam_core
        ${OpenCV_LIBS}
    )
    add_dependencies(test_pipeline_cpp umi_slam_pipeline)
    add_test(NAME test_pipeline_cpp COMMAND test_pipeline_cpp)

    if(EDS_BUILD_STATION)
        add_executable(test_station_cpp tests/test_station_cpp.cpp ${EDS_UTF8_RC})
        target_compile_definitions(test_station_cpp PRIVATE EDS_REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
        target_link_libraries(test_station_cpp PRIVATE
            slam_common
            calibration_targets
            camera_imu_calibration
            handeye_calibration
            vision_calibration
            preview_3d
            umi_slam_core
            ${OpenCV_LIBS}
        )
        add_test(NAME test_station_cpp COMMAND test_station_cpp)
    endif()
endif()
