#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace umi_slam {

/// Quote a single argument for a Windows command line (CommandLineToArgvW rules).
std::string quote_win_arg(const std::string& arg);

/// Join arguments into a Windows command line string.
std::string build_win_cmdline(const std::vector<std::string>& args);

/// Run a subprocess capturing combined stdout+stderr. Returns {exit_code, output}; {-1, ""} on spawn failure.
std::pair<int, std::string> run_subprocess_capture(const std::vector<std::string>& args);

struct RunInheritOptions {
    /// Working directory for the child; empty inherits the current directory.
    std::filesystem::path cwd;
    /// Environment variable names removed from the child's environment.
    std::vector<std::string> unset_env;
};

/// Run a subprocess inheriting stdio, wait without timeout, return its exit code (-1 on spawn failure).
int run_inherit(const std::vector<std::string>& args, const RunInheritOptions& options = {});

/// Locate a program on PATH (appends .exe on Windows). Empty optional when not found.
std::optional<std::filesystem::path> which(const std::string& program);

/// Absolute directory containing the running executable (falls back to the current directory).
std::filesystem::path current_executable_dir();

}  // namespace umi_slam
