#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace umi_slam {

/// Inputs for an isolated ORB-SLAM3/Pangolin backend build.
struct BackendBuildOptions {
    /// Directory containing ORB_SLAM3/ and Pangolin/ source trees (never modified).
    std::filesystem::path source_root;
    /// Directory containing runner.cc + CMakeLists.txt for umi_slam_runner.
    std::filesystem::path runner_dir;
    /// Isolated build directory receiving staged sources, build trees and backend.json.
    std::filesystem::path build_dir;
    int jobs = 2;
    std::string cmake_exe = "cmake";
    /// Optional CMake toolchain file (e.g. conan_toolchain.cmake); defaults to compile-time or env CMAKE_TOOLCHAIN_FILE.
    std::filesystem::path toolchain_file;
};

/// SHA-256 over runner sources plus ORB_SLAM3/Pangolin compilable files; detects source or patch changes.
std::string backend_source_digest(
    const std::filesystem::path& source_root,
    const std::filesystem::path& runner_dir);

/// Stage patched source copies into build_dir; returns true when staged now, false when the stamp matches.
/// Throws when the digest changed or a previous staging was left incomplete.
bool stage_backend_sources(const BackendBuildOptions& opt, const std::string& digest);

/// Write build_dir/backend.json with paths relative to build_dir; returns the json path.
std::filesystem::path write_backend_config(
    const std::filesystem::path& build_dir,
    const std::filesystem::path& binary,
    const std::filesystem::path& vocabulary,
    const std::vector<std::filesystem::path>& library_dirs,
    const std::string& digest);

/// Stage (if needed), configure, build and bundle the backend; returns the backend.json path.
/// Throws std::runtime_error on any failed step.
std::filesystem::path build_backend(const BackendBuildOptions& opt);

}  // namespace umi_slam
