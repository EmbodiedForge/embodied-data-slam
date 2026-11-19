#include "umi_slam/build_backend.hpp"

#include "umi_slam/pipeline.hpp"
#include "process_util.hpp"
#include "slam_common/json.hpp"
#include "slam_common/sha256.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace umi_slam {
namespace {

using slam_common::JsonValue;
namespace fs = std::filesystem;

std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::string lower_ascii(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

void feed_file(slam_common::Sha256& sha, const fs::path& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot read " + path.string());
    std::array<char, 65536> buf{};
    while (f) {
        f.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        std::streamsize n = f.gcount();
        if (n > 0) sha.update(reinterpret_cast<const uint8_t*>(buf.data()), static_cast<size_t>(n));
    }
}

bool digest_source_file(const fs::path& path) {
    static const std::set<std::string> kExts = {".cc", ".cpp", ".c", ".h", ".hpp", ".cmake"};
    std::string ext = lower_ascii(path.extension().generic_u8string());
    return kExts.count(ext) || path.filename() == "CMakeLists.txt";
}

bool has_excluded_component(const fs::path& rel) {
    for (const auto& comp : rel) {
        std::string c = comp.generic_u8string();
        if (c == "build" || c == ".git") return true;
    }
    return false;
}

void run_step(const std::vector<std::string>& args, const RunInheritOptions& options = {}) {
    std::ostringstream line;
    for (size_t i = 0; i < args.size(); ++i) {
        if (i > 0) line << ' ';
        line << args[i];
    }
    std::cout << "+ " << line.str() << std::endl;
    int rc = run_inherit(args, options);
    if (rc != 0) {
        throw std::runtime_error("Command failed with exit code " + std::to_string(rc) + ": " + line.str());
    }
}

std::vector<std::string> cmake_base_args(const fs::path& explicit_toolchain) {
    std::vector<std::string> args;
    const char* gen = std::getenv("CMAKE_GENERATOR");
    if ((gen == nullptr || gen[0] == '\0') && which("ninja").has_value()) {
        args.insert(args.end(), {"-G", "Ninja"});
    }

    fs::path tc = explicit_toolchain;
    if (tc.empty()) {
        if (const char* env_tc = std::getenv("CMAKE_TOOLCHAIN_FILE"); env_tc != nullptr && env_tc[0] != '\0') {
            tc = fs::u8path(env_tc);
        }
#ifdef EDS_CONAN_TOOLCHAIN_FILE
        else if (fs::is_regular_file(fs::u8path(EDS_CONAN_TOOLCHAIN_FILE))) {
            tc = fs::u8path(EDS_CONAN_TOOLCHAIN_FILE);
        }
#endif
    }
    if (!tc.empty() && fs::is_regular_file(tc)) {
        args.push_back("-DCMAKE_TOOLCHAIN_FILE=" + fs::weakly_canonical(tc).string());
    }

#ifdef _WIN32
    args.push_back("-DBUILD_SHARED_LIBS=OFF");
#elif defined(__APPLE__)
    args.push_back("-DCMAKE_BUILD_RPATH=@loader_path");
    args.push_back("-DCMAKE_INSTALL_RPATH=@loader_path");
#else
    args.push_back("-DCMAKE_BUILD_RPATH=$ORIGIN");
    args.push_back("-DCMAKE_INSTALL_RPATH=$ORIGIN");
#endif
    return args;
}

bool ignored_copy_name(const std::string& name) {
    if (name == "build" || name == ".git" || name == "__pycache__" || name == "ORBvoc.txt" ||
        name == "Examples" || name == "Examples_old" || name == ".build") {
        return true;
    }
    if (name.find(".so") != std::string::npos) return true;  // *.so*
    if (name.size() >= 2 && name.compare(name.size() - 2, 2, ".o") == 0) return true;  // *.o
    if (name.size() >= 2 && name.compare(name.size() - 2, 2, ".a") == 0) return true;  // *.a
    return false;
}

void copy_filtered_tree(const fs::path& src, const fs::path& dst) {
    fs::create_directories(dst);
    for (const auto& entry : fs::directory_iterator(src)) {
        std::string name = entry.path().filename().generic_u8string();
        if (ignored_copy_name(name)) continue;
        fs::path target = dst / entry.path().filename();
        if (entry.is_directory()) {
            copy_filtered_tree(entry.path(), target);
        } else if (entry.is_regular_file()) {
            fs::copy_file(entry.path(), target, fs::copy_options::overwrite_existing);
        }
    }
}

void bundle_windows_dlls(const fs::path& exe_path, const std::vector<fs::path>& search_dirs) {
    auto tool = which("llvm-objdump");
    if (!tool.has_value()) tool = which("objdump");
    if (!tool.has_value()) return;

    fs::path dest_dir = exe_path.parent_path();
    std::map<std::string, fs::path> lookup;
    for (const auto& dir : search_dirs) {
        if (!fs::is_directory(dir)) continue;
        for (const auto& entry : fs::directory_iterator(dir)) {
            if (!entry.is_regular_file()) continue;
            std::string name = lower_ascii(entry.path().filename().generic_u8string());
            if (name.size() >= 4 && name.compare(name.size() - 4, 4, ".dll") == 0) {
                lookup.emplace(name, entry.path());
            }
        }
    }

    static const std::regex dll_name_re(R"(DLL Name:\s+(\S+\.dll))", std::regex::icase);
    std::set<std::string> seen;
    std::vector<fs::path> queue{exe_path};
    for (size_t qi = 0; qi < queue.size(); ++qi) {
        fs::path current = queue[qi];
        auto [rc, out] = run_subprocess_capture({tool->string(), "-p", current.string()});
        if (rc != 0) {
            throw std::runtime_error("objdump -p failed on " + current.string());
        }
        for (std::sregex_iterator it(out.begin(), out.end(), dll_name_re), end; it != end; ++it) {
            std::string name = lower_ascii((*it)[1].str());
            if (!seen.insert(name).second) continue;
            auto found = lookup.find(name);
            if (found == lookup.end()) continue;
            fs::path target = dest_dir / found->second.filename();
            std::error_code ec;
            if (fs::weakly_canonical(found->second, ec) != fs::weakly_canonical(target, ec)) {
                fs::copy_file(found->second, target, fs::copy_options::overwrite_existing);
            }
            queue.push_back(target);
        }
    }
}

}  // namespace

std::string backend_source_digest(
    const std::filesystem::path& source_root,
    const std::filesystem::path& runner_dir) {
    slam_common::Sha256 sha;
    sha.update(reinterpret_cast<const uint8_t*>("eds-backend-v1\n"), 15);
    for (const char* name : {"runner.cc", "CMakeLists.txt"}) {
        sha.update(std::string(name));
        feed_file(sha, runner_dir / name);
    }
    for (const char* tree : {"ORB_SLAM3", "Pangolin"}) {
        fs::path base = source_root / tree;
        if (!fs::is_directory(base)) continue;
        std::vector<fs::path> files;
        fs::recursive_directory_iterator end;
        for (auto it = fs::recursive_directory_iterator(base); it != end; ++it) {
            if (!it->is_regular_file()) continue;
            fs::path rel = it->path().lexically_relative(base);
            if (has_excluded_component(rel) || !digest_source_file(it->path())) continue;
            files.push_back(it->path());
        }
        std::sort(files.begin(), files.end(), [&](const fs::path& a, const fs::path& b) {
            return a.lexically_relative(base).generic_u8string() < b.lexically_relative(base).generic_u8string();
        });
        for (const auto& f : files) {
            std::string rel = f.lexically_relative(base).generic_u8string();
            sha.update(rel);
            feed_file(sha, f);
        }
    }
    return sha.final_hex();
}

bool stage_backend_sources(const BackendBuildOptions& opt, const std::string& digest) {
    for (const char* name : {"ORB_SLAM3", "Pangolin"}) {
        fs::path original = opt.source_root / name;
        if (!fs::is_directory(original)) {
            throw std::runtime_error("Missing source: " + original.string());
        }
    }

    fs::create_directories(opt.build_dir);
    fs::path stamp = opt.build_dir / "source.sha256";
    if (fs::is_regular_file(stamp)) {
        std::string old = trim(slam_common::read_text_utf8(stamp));
        if (old != digest) {
            throw std::runtime_error("Source changed: choose a new --build-dir to preserve the old build");
        }
        return false;
    }
    fs::path orb = opt.build_dir / "ORB_SLAM3";
    fs::path pang = opt.build_dir / "Pangolin";
    if (fs::exists(orb) || fs::exists(pang)) {
        throw std::runtime_error("Incomplete source staging: choose a new --build-dir");
    }

    copy_filtered_tree(opt.source_root / "ORB_SLAM3", orb);
    copy_filtered_tree(opt.source_root / "Pangolin", pang);
    patch_orb(orb);

    fs::path vocab_dir = orb / "Vocabulary";
    run_step({opt.cmake_exe, "-E", "tar", "xzf", "ORBvoc.txt.tar.gz"}, {vocab_dir});
    if (!fs::is_regular_file(vocab_dir / "ORBvoc.txt")) {
        throw std::runtime_error("Vocabulary extraction failed: missing " + (vocab_dir / "ORBvoc.txt").string());
    }

    {
        std::ofstream f(stamp, std::ios::binary | std::ios::trunc);
        if (!f) throw std::runtime_error("Cannot write " + stamp.string());
        f << digest;
    }
    return true;
}

std::filesystem::path write_backend_config(
    const std::filesystem::path& build_dir,
    const std::filesystem::path& binary,
    const std::filesystem::path& vocabulary,
    const std::vector<std::filesystem::path>& library_dirs,
    const std::string& digest) {
    auto rel = [&](const fs::path& p) {
        return fs::relative(p, build_dir).generic_u8string();
    };
    JsonValue cfg = JsonValue::object();
    cfg["binary"] = rel(binary);
    cfg["vocabulary"] = rel(vocabulary);
    JsonValue libs = JsonValue::array();
    for (const auto& dir : library_dirs) libs.push_back(rel(dir));
    cfg["library_paths"] = libs;
    cfg["source_sha256"] = digest;
    fs::path out = build_dir / "backend.json";
    slam_common::save_json(out, cfg, true);
    return out;
}

std::filesystem::path build_backend(const BackendBuildOptions& opt_in) {
    BackendBuildOptions opt = opt_in;
    opt.source_root = fs::weakly_canonical(fs::absolute(opt.source_root));
    opt.runner_dir = fs::weakly_canonical(fs::absolute(opt.runner_dir));
    fs::create_directories(opt.build_dir);
    opt.build_dir = fs::weakly_canonical(fs::absolute(opt.build_dir));
    if (opt.jobs < 1) throw std::invalid_argument("jobs must be >= 1");

    const std::string& cmake = opt.cmake_exe;
    const std::string jobs = std::to_string(opt.jobs);
    fs::path orb = opt.build_dir / "ORB_SLAM3";
    fs::path pang = opt.build_dir / "Pangolin";
    fs::path prefix = opt.build_dir / "install";
    fs::path dbow2 = orb / "Thirdparty/DBoW2";
    fs::path runner_build = opt.build_dir / "runner";

    std::string digest = backend_source_digest(opt.source_root, opt.runner_dir);
    stage_backend_sources(opt, digest);

    RunInheritOptions run_opt;
    std::vector<std::string> base = cmake_base_args(opt.toolchain_file);
    std::vector<std::string> with_base;
    with_base.push_back(cmake);
    with_base.insert(with_base.end(), base.begin(), base.end());

    auto cmake_cmd = [&](std::vector<std::string> tail) {
        std::vector<std::string> cmd = with_base;
        cmd.insert(cmd.end(), tail.begin(), tail.end());
        return cmd;
    };
    auto path_arg = [](const fs::path& p) { return p.string(); };

    run_step(cmake_cmd({"-S", path_arg(pang), "-B", path_arg(pang / "build"),
                        "-DCMAKE_BUILD_TYPE=Release",
                        "-DCMAKE_INSTALL_PREFIX=" + path_arg(prefix),
                        "-DBUILD_EXAMPLES=OFF", "-DBUILD_TOOLS=OFF",
                        "-DBUILD_PANGOLIN_PYTHON=OFF", "-DBUILD_PANGOLIN_FFMPEG=OFF",
                        "-DBUILD_PANGOLIN_LIBOPENEXR=OFF",
                        "-DBUILD_PANGOLIN_LIBPNG=OFF", "-DBUILD_PANGOLIN_LIBJPEG=OFF",
                        "-DBUILD_PANGOLIN_LIBTIFF=OFF", "-DBUILD_PANGOLIN_LZ4=OFF",
                        "-DBUILD_PANGOLIN_ZSTD=OFF"}), run_opt);
    run_step({cmake, "--build", path_arg(pang / "build"), "-j", jobs}, run_opt);
    run_step({cmake, "--install", path_arg(pang / "build")}, run_opt);

    run_step(cmake_cmd({"-S", path_arg(dbow2), "-B", path_arg(dbow2 / "build"),
                        "-DCMAKE_BUILD_TYPE=Release"}), run_opt);
    run_step({cmake, "--build", path_arg(dbow2 / "build"), "-j", jobs}, run_opt);

    run_step(cmake_cmd({"-S", path_arg(orb), "-B", path_arg(orb / "build"),
                        "-DCMAKE_BUILD_TYPE=Release",
                        "-DCMAKE_PREFIX_PATH=" + path_arg(prefix)}), run_opt);
    run_step({cmake, "--build", path_arg(orb / "build"), "--target", "g2o", "-j", jobs}, run_opt);
    run_step({cmake, "--build", path_arg(orb / "build"), "--target", "ORB_SLAM3", "-j", jobs}, run_opt);

    std::vector<std::string> runner_tail = {
        "-S", path_arg(opt.runner_dir),
        "-B", path_arg(runner_build),
        "-DORB_ROOT=" + path_arg(orb),
        "-DCMAKE_PREFIX_PATH=" + path_arg(prefix),
        "-DCMAKE_BUILD_TYPE=Release"};
#ifdef _WIN32
    fs::path utf8_manifest = (opt.runner_dir / "../../../cmake/windows_utf8.manifest").lexically_normal();
    if (fs::is_regular_file(utf8_manifest)) {
        runner_tail.push_back("-DEDS_WIN_UTF8_MANIFEST=" + fs::weakly_canonical(utf8_manifest).string());
    }
    fs::path utf8_rc = (opt.runner_dir / "../../../cmake/windows_utf8.rc").lexically_normal();
    if (fs::is_regular_file(utf8_rc)) {
        runner_tail.push_back("-DEDS_WIN_UTF8_RC=" + fs::weakly_canonical(utf8_rc).string());
    }
#endif
    run_step(cmake_cmd(runner_tail), run_opt);
    run_step({cmake, "--build", path_arg(runner_build), "-j", jobs}, run_opt);

#ifdef _WIN32
    fs::path exe = runner_build / "umi_slam_runner.exe";
#else
    fs::path exe = runner_build / "umi_slam_runner";
#endif
    if (!fs::is_regular_file(exe)) {
        throw std::runtime_error("Backend runner missing after build: " + exe.string());
    }

    std::vector<fs::path> lib_dirs = {orb / "lib", dbow2 / "lib", orb / "Thirdparty/g2o/lib", prefix / "lib"};
#ifdef _WIN32
    std::vector<fs::path> search_dirs = {runner_build};
    search_dirs.insert(search_dirs.end(), lib_dirs.begin(), lib_dirs.end());
    search_dirs.push_back(prefix / "bin");
    auto cxx = which("clang++");
    if (!cxx.has_value()) cxx = which("g++");
    if (cxx.has_value()) {
        search_dirs.push_back(fs::weakly_canonical(*cxx).parent_path());
    }
    bundle_windows_dlls(exe, search_dirs);
    lib_dirs.insert(lib_dirs.begin(), runner_build);
    lib_dirs.push_back(prefix / "bin");
#endif

    return write_backend_config(opt.build_dir, exe, orb / "Vocabulary/ORBvoc.txt", lib_dirs, digest);
}

}  // namespace umi_slam
