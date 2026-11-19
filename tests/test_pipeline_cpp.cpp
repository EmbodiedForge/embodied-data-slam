#include "slam_common/json.hpp"
#include "slam_common/ms901m.hpp"
#include "slam_common/sha256.hpp"
#include "slam_common/trajectory_io.hpp"
#include "umi_slam/build_backend.hpp"
#include "umi_slam/pipeline.hpp"
#include "umi_slam/process_util.hpp"
#include "umi_slam/runtime.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

#define REQUIRE(cond) do { if (!(cond)) { throw std::runtime_error(std::string("REQUIRE failed at line ") + std::to_string(__LINE__) + ": " #cond); } } while(0)
#define REQUIRE_NEAR(a, b, tol) do { double _va = (a), _vb = (b); if (!std::isfinite(_va) || std::abs(_va - _vb) > (tol)) { std::ostringstream _os; _os << "REQUIRE_NEAR failed at line " << __LINE__ << ": " << _va << " vs " << _vb; throw std::runtime_error(_os.str()); } } while(0)

template <typename Fn>
void require_throws(Fn&& fn, const std::string& substr = "") {
    bool threw = false;
    try {
        fn();
    } catch (const std::exception& exc) {
        threw = true;
        if (!substr.empty() && std::string(exc.what()).find(substr) == std::string::npos) {
            throw std::runtime_error(std::string("Exception mismatch: ") + exc.what());
        }
    }
    if (!threw) throw std::runtime_error("Expected exception not thrown");
}

struct TempDir {
    std::filesystem::path path;
    explicit TempDir(const std::string& tag) {
        static std::atomic<int> counter{0};
        auto base = std::filesystem::temp_directory_path() / ("eds_cpp_pipe_" + tag + "_" + std::to_string(++counter));
        std::filesystem::remove_all(base);
        std::filesystem::create_directories(base);
        path = base;
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

std::filesystem::path repo_root() { return std::filesystem::u8path(EDS_REPO_ROOT); }

void write_file(const std::filesystem::path& p, const std::string& content) {
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    f.write(content.data(), static_cast<std::streamsize>(content.size()));
}

void write_bytes(const std::filesystem::path& p, const std::vector<uint8_t>& bytes) {
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

std::vector<uint8_t> pack_block(uint64_t ts, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> out(12 + payload.size());
    for (int i = 0; i < 8; ++i) out[i] = static_cast<uint8_t>((ts >> (8 * i)) & 0xFFu);
    uint32_t sz = static_cast<uint32_t>(payload.size());
    for (int i = 0; i < 4; ++i) out[8 + i] = static_cast<uint8_t>((sz >> (8 * i)) & 0xFFu);
    std::copy(payload.begin(), payload.end(), out.begin() + 12);
    return out;
}

void test_uart_fragments_and_units() {
    TempDir td("uart");
    std::vector<uint8_t> pkt = {0x55, 0x55, 0x03, 0x0c};
    int16_t v[6] = {0, 0, 8192, 0, 16384, 0};
    for (int i = 0; i < 6; ++i) {
        uint16_t u = static_cast<uint16_t>(v[i]);
        pkt.push_back(static_cast<uint8_t>(u & 0xFFu));
        pkt.push_back(static_cast<uint8_t>((u >> 8) & 0xFFu));
    }
    uint32_t sum = 0;
    for (uint8_t b : pkt) sum += b;
    pkt.push_back(static_cast<uint8_t>(sum & 0xFFu));

    std::vector<uint8_t> part0(pkt.begin(), pkt.begin() + 3);
    std::vector<uint8_t> part1(pkt.begin() + 3, pkt.end());
    part1.insert(part1.end(), pkt.begin(), pkt.end());
    auto b0 = pack_block(90000000ULL, part0);
    auto b1 = pack_block(100000000ULL, part1);
    b0.insert(b0.end(), b1.begin(), b1.end());
    auto fp = td.path / "imu.bin";
    write_bytes(fp, b0);

    auto imu = slam_common::decode_ms901m_bin(fp, 250.0, 4.0, 2000.0);
    REQUIRE(imu.timestamps_ns.size() == 2);
    REQUIRE(imu.timestamps_ns[0] == 96000000LL);
    REQUIRE(imu.timestamps_ns[1] == 100000000LL);
    REQUIRE_NEAR(imu.samples[0](5), 9.80665, 1e-6);
    REQUIRE_NEAR(imu.samples[0](1), 1000.0 * kPi / 180.0, 1e-6);
}

void test_truncated_uart_rejected() {
    TempDir td("trunc");
    auto fp = td.path / "imu.bin";
    write_bytes(fp, pack_block(1ULL, {'1', '2', '3'}));
    // Corrupt block length header -> 17 while payload is 3 bytes
    std::vector<uint8_t> raw(12 + 3, 0);
    raw[0] = 1; raw[8] = 17; raw[12] = '1'; raw[13] = '2'; raw[14] = '3';
    write_bytes(fp, raw);
    require_throws([&]() { slam_common::decode_ms901m_bin(fp, 250.0, 4.0, 2000.0); }, "Truncated");
}

void test_timestamp_precision_and_order() {
    REQUIRE(slam_common::integer_ns("1789459434308037880.000000") == 1789459434308037880LL);
    for (const char* bad : {"nan", "inf", "1.1", "-1"}) {
        require_throws([&]() { slam_common::integer_ns(bad); });
    }
    require_throws([&]() { slam_common::strictly_increasing({1, 1}, "IMU"); });
}

slam_common::JsonValue make_export(const std::filesystem::path& dir, bool initialized = true) {
    write_file(dir / "imu_euroc.txt",
               "1000000000.000000 0 0 0 0 0 0 1\n"
               "1040000000.000000 1 0 0 0 0 0 1\n"
               "1200000000.000000 2 0 0 0 0 0 1\n");
    std::string init_s = initialized ? "1" : "0";
    write_file(dir / "tracking.csv",
               "timestamp_ns,state,imu_initialized,map_id\n"
               "1000000000,2," + init_s + ",0\n"
               "1040000000,2," + init_s + ",0\n"
               "1200000000,2," + init_s + ",0\n");
    slam_common::JsonValue rep = slam_common::JsonValue::object();
    rep["first_timestamp_ns"] = static_cast<int64_t>(1000000000LL);
    rep["selected_frames"] = static_cast<int64_t>(6);
    rep["median_frame_period_s"] = 0.04;
    rep["warnings"] = slam_common::JsonValue::array();
    return rep;
}

void test_export_preserves_imu_pose_and_gaps() {
    TempDir td("exp_gaps");
    auto q = umi_slam::export_result(td.path, make_export(td.path, true));
    REQUIRE(q.get("segments").as_int() == 2);
    REQUIRE_NEAR(q.get("coverage_fraction").as_double(), 0.5, 1e-9);
    REQUIRE(q.get("pose").as_string() == "T_world_imu");
    auto pts = slam_common::read_csv_points(td.path / "imu_trajectory.csv");
    REQUIRE(pts.size() == 3);
    REQUIRE_NEAR(pts.back().pos.x(), 2.0, 1e-9);
    REQUIRE(pts.back().segment_id == "1");
    REQUIRE(slam_common::read_text_utf8(td.path / "imu_trajectory.tum").rfind("1.000000000 ", 0) == 0);
}

void test_uninitialized_not_success() {
    TempDir td("uninit");
    require_throws([&]() { umi_slam::export_result(td.path, make_export(td.path, false)); }, "initialized");
}

void test_export_float_ns_roundoff_matches_original_stamp() {
    TempDir td("roundoff");
    auto rep = make_export(td.path, true);
    std::string txt = slam_common::read_text_utf8(td.path / "imu_euroc.txt");
    size_t pos = txt.find("1000000000.000000");
    txt.replace(pos, 17, "1000000000.015625");
    write_file(td.path / "imu_euroc.txt", txt);
    umi_slam::export_result(td.path, rep);
    std::string csv_txt = slam_common::read_text_utf8(td.path / "imu_trajectory.csv");
    REQUIRE(csv_txt.find("\n1000000000,") != std::string::npos);
}

void test_prepare_camera_imu_shift_and_size() {
    TempDir td("prep");
    auto src = td.path / "source";
    auto dst = td.path / "output";
    std::filesystem::create_directories(dst);
    cv::Mat blank(720, 1280, CV_8UC1, cv::Scalar(0));
    for (const char* cam : {"cam0", "cam1"}) {
        auto d = src / "mav0" / cam / "data";
        std::filesystem::create_directories(d);
        for (int64_t t : {1000000000LL, 1040000000LL, 1080000000LL}) {
            cv::imwrite((d / (std::to_string(t) + ".png")).string(), blank);
        }
    }
    write_file(src / "timestamps.txt", "1000000000\n1040000000\n1080000000\n");
    std::ostringstream imu_csv;
    for (int64_t t = 996000000LL; t < 1090000000LL; t += 4000000LL) {
        imu_csv << t << ",0,0,0,0,0,9.80665\n";
    }
    write_file(src / "mav0/imu0/data.csv", imu_csv.str());

    auto settings = repo_root() / "software/umi_slam/profiles/umi_20260908.yaml";
    slam_common::JsonValue prof = slam_common::JsonValue::object();
    prof["camera_to_imu_offset_ns"] = static_cast<int64_t>(1000000LL);
    auto rep = umi_slam::prepare(src, dst, prof, settings, 1, 0);
    REQUIRE(rep.get("selected_frames").as_int() == 3);
    REQUIRE(slam_common::read_text_utf8(dst / "imu.csv").rfind("995000000,", 0) == 0);
    REQUIRE(slam_common::read_text_utf8(dst / "settings.yaml").find("Camera.fps: 25") != std::string::npos);
}

void test_load_backend_config_resolves_relative_paths() {
    TempDir td("bcfg");
    auto bdir = td.path / "backend";
    std::filesystem::create_directories(bdir / "platforms");
    write_file(bdir / "umi_slam_runner.exe", "bin");
    write_file(bdir / "ORBvoc.txt", "voc");
    slam_common::JsonValue cfg = slam_common::JsonValue::object();
    cfg["binary"] = "umi_slam_runner.exe";
    cfg["vocabulary"] = "ORBvoc.txt";
    cfg["library_paths"] = slam_common::JsonValue(slam_common::JsonValue::Array{"."});
    auto cfg_p = bdir / "backend.json";
    slam_common::save_json(cfg_p, cfg);

    auto loaded = umi_slam::load_backend_config(cfg_p);
    REQUIRE(std::filesystem::u8path(loaded.binary) == std::filesystem::weakly_canonical(bdir / "umi_slam_runner.exe"));
    REQUIRE(std::filesystem::u8path(loaded.vocabulary) == std::filesystem::weakly_canonical(bdir / "ORBvoc.txt"));
    REQUIRE(loaded.library_paths.size() == 1);
    REQUIRE(std::filesystem::u8path(loaded.library_paths[0]) == std::filesystem::weakly_canonical(bdir));

    std::map<std::string, std::string> empty_env;
    auto env = umi_slam::backend_env(loaded, &empty_env);
#ifdef _WIN32
    REQUIRE(env["PATH"].find(std::filesystem::weakly_canonical(bdir).string()) != std::string::npos);
#elif defined(__APPLE__)
    REQUIRE(env["DYLD_LIBRARY_PATH"].find(std::filesystem::weakly_canonical(bdir).string()) != std::string::npos);
#else
    REQUIRE(env["LD_LIBRARY_PATH"].find(std::filesystem::weakly_canonical(bdir).string()) != std::string::npos);
#endif
}

void test_patch_orb_applies_all_patches_and_rejects_mismatch() {
    TempDir td("patch_orb");
    auto orb_src = repo_root() / "source/stereo_imu_trajectory/ORB_SLAM3";
    if (!std::filesystem::is_directory(orb_src)) return;

    auto staged = td.path / "orb_stage";
    std::filesystem::create_directories(staged / "include");
    std::filesystem::create_directories(staged / "src");
    std::filesystem::create_directories(staged / "Thirdparty/DBoW2");
    std::filesystem::copy_file(orb_src / "CMakeLists.txt", staged / "CMakeLists.txt");
    std::filesystem::copy_file(orb_src / "include/System.h", staged / "include/System.h");
    std::filesystem::copy_file(orb_src / "src/System.cc", staged / "src/System.cc");
    std::filesystem::copy_file(orb_src / "Thirdparty/DBoW2/CMakeLists.txt", staged / "Thirdparty/DBoW2/CMakeLists.txt");

    umi_slam::patch_orb(staged);
    REQUIRE(slam_common::read_text_utf8(staged / "CMakeLists.txt").find("# Build examples") == std::string::npos);
    REQUIRE(slam_common::read_text_utf8(staged / "Thirdparty/DBoW2/CMakeLists.txt").find("-march=native") == std::string::npos);
    REQUIRE(slam_common::read_text_utf8(staged / "include/System.h").find("OfflineSaveMapPoints") != std::string::npos);
    std::string cc = slam_common::read_text_utf8(staged / "src/System.cc");
    REQUIRE(cc.find("bool System::OfflineImuInitialized()") != std::string::npos);
    REQUIRE(cc.find("mpLoopCloser->isRunningGBA()") != std::string::npos);
    size_t guard_cnt = 0, pos = 0;
    const std::string guard = "if(!pBiggerMap || pBiggerMap->KeyFramesInMap() == 0) return;";
    while ((pos = cc.find(guard, pos)) != std::string::npos) { ++guard_cnt; pos += guard.size(); }
    REQUIRE(guard_cnt == 5);

    require_throws([&]() { umi_slam::patch_orb(staged); });
}

void test_paths_with_spaces_chinese_and_ampersand() {
    TempDir td("chars");
    auto src = td.path / std::filesystem::u8path("U盘 数据 & 测试");
    std::filesystem::create_directories(src / "mav0/cam0/data");
    std::filesystem::create_directories(src / "mav0/cam1/data");
    std::filesystem::create_directories(src / "mav0/imu0");
    write_file(src / "mav0/cam0/data.csv", "#\n");
    write_file(src / "mav0/cam1/data.csv", "#\n");
    write_file(src / "mav0/imu0/data.csv", "#\n");
    auto settings = td.path / "s.yaml";
    write_file(settings, "settings");
    auto device = repo_root() / "software/slam_station/devices/generic_euroc.json";

    auto bdir = td.path / "backend";
    std::filesystem::create_directories(bdir);
#ifdef _WIN32
    std::string bin_name = "umi_slam_runner.exe";
#else
    std::string bin_name = "umi_slam_runner";
#endif
    write_file(bdir / bin_name, "bin");
    write_file(bdir / "ORBvoc.txt", "voc");
    slam_common::JsonValue bcfg = slam_common::JsonValue::object();
    bcfg["binary"] = bin_name;
    bcfg["vocabulary"] = "ORBvoc.txt";
    bcfg["library_paths"] = slam_common::JsonValue(slam_common::JsonValue::Array{"."});
    auto bcfg_p = bdir / "backend.json";
    slam_common::save_json(bcfg_p, bcfg);

    auto cmd = umi_slam::command(src, td.path / std::filesystem::u8path("输出 目录 & 1/result"), settings, device, 5, 0, 0.0, true, bcfg_p);
    REQUIRE(std::find(cmd.begin(), cmd.end(), "--prepare-only") != cmd.end());
}

void test_refuses_nested_output() {
    TempDir td("nest_out");
    auto src = td.path / "input";
    std::filesystem::create_directories(src);
    auto settings = td.path / "s.yaml";
    write_file(settings, "settings");
    auto device = repo_root() / "software/slam_station/devices/generic_euroc.json";
    require_throws([&]() { umi_slam::command(src, src / "result", settings, device); });
}

void test_sha256_streaming_and_canonical_crlf() {
    slam_common::Sha256 h;
    h.update(std::string("hello "));
    h.update(reinterpret_cast<const uint8_t*>("wor"), 3);
    h.update(std::string("ld"));
    REQUIRE(h.final_hex() == slam_common::sha256_hex(std::string("hello world")));

    TempDir td("canon");
    write_file(td.path / "lf.txt", "line1\nline2\n");
    write_file(td.path / "crlf.txt", "line1\r\nline2\r\n");
    REQUIRE(slam_common::compute_canonical_sha256(td.path / "crlf.txt") ==
            slam_common::compute_canonical_sha256(td.path / "lf.txt"));

    // 含 NUL 的文件按原始字节哈希，不做 CRLF 规范化
    std::string bin("a\0b\r\nc", 6);
    write_file(td.path / "blob.dat", bin);
    REQUIRE(slam_common::compute_canonical_sha256(td.path / "blob.dat") == slam_common::sha256_hex(bin));
}

void test_update_repo_checksums_temp_tree() {
    TempDir td("sums");
    auto root = td.path;
    write_file(root / "README.md", "# readme\n");
    write_file(root / "notes.txt", "note\r\n");
    write_file(root / "src/foo.cpp", "int foo() { return 0; }\n");
    write_file(root / "src/sub/bar.h", "#pragma once\n");
    write_file(root / "src/build/skipped.cpp", "skip\n");
    write_file(root / "src/blob.pyc", "pyc");

    write_file(root / "SHA256SUMS",
               slam_common::sha256_hex("stale") + "  old/deleted.txt\n" +
               "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef  images/offline.bin\n" +
               slam_common::sha256_hex("outdated") + "  README.md\n");

    int n = slam_common::update_repo_checksums(root);
    REQUIRE(n == 4);  // README.md, notes.txt, src/foo.cpp, src/sub/bar.h

    auto report = slam_common::verify_repo_checksums(root);
    REQUIRE(report.checked_count == 4);
    REQUIRE(report.mismatches.empty());

    std::string sums = slam_common::read_text_utf8(root / "SHA256SUMS");
    REQUIRE(sums.find("old/deleted.txt") == std::string::npos);
    REQUIRE(sums.find("src/build/skipped.cpp") == std::string::npos);
    REQUIRE(sums.find("src/blob.pyc") == std::string::npos);
    REQUIRE(sums.find("0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef  images/offline.bin") != std::string::npos);

    std::vector<std::string> keys = {"README.md", "images/offline.bin", "notes.txt", "src/foo.cpp", "src/sub/bar.h"};
    size_t prev = 0;
    for (const auto& k : keys) {
        size_t pos = sums.find(k);
        REQUIRE(pos != std::string::npos);
        REQUIRE(pos > prev);
        prev = pos;
    }
    // CRLF 文本与 LF 形式哈希一致
    REQUIRE(sums.find(slam_common::compute_canonical_sha256(root / "notes.txt")) != std::string::npos);
}

void test_backend_source_digest_tracks_source_files_only() {
    TempDir td("bdigest");
    auto src = td.path / "source";
    auto runner = td.path / "runner";
    write_file(src / "ORB_SLAM3/src/a.cc", "int a() { return 0; }\n");
    write_file(src / "ORB_SLAM3/include/a.h", "#pragma once\n");
    write_file(src / "ORB_SLAM3/README.txt", "not hashed\n");
    write_file(src / "ORB_SLAM3/Thirdparty/x/build/y.cc", "int y() {}\n");
    write_file(src / "Pangolin/lib/p.cpp", "int p() { return 1; }\n");
    write_file(runner / "runner.cc", "int main() { return 0; }\n");
    write_file(runner / "CMakeLists.txt", "cmake_minimum_required(VERSION 3.16)\n");

    std::string d1 = umi_slam::backend_source_digest(src, runner);
    REQUIRE(d1.size() == 64);
    REQUIRE(d1 == umi_slam::backend_source_digest(src, runner));

    write_file(src / "ORB_SLAM3/src/a.cc", "int a() { return 1; }\n");
    std::string d2 = umi_slam::backend_source_digest(src, runner);
    REQUIRE(d2 != d1);

    write_file(src / "ORB_SLAM3/README.txt", "changed\n");
    write_file(src / "ORB_SLAM3/blob.bin", "bin");
    REQUIRE(umi_slam::backend_source_digest(src, runner) == d2);

    write_file(runner / "runner.cc", "int main() { return 2; }\n");
    REQUIRE(umi_slam::backend_source_digest(src, runner) != d2);
}

void test_stage_backend_sources_patches_stamps_and_guards() {
    auto real = repo_root() / "source/stereo_imu_trajectory/ORB_SLAM3";
    if (!std::filesystem::is_directory(real)) return;
    TempDir td("stage");
    auto src = td.path / "source";
    auto orb = src / "ORB_SLAM3";
    auto pang = src / "Pangolin";
    for (const char* rel : {"CMakeLists.txt", "include/System.h", "src/System.cc",
                            "Thirdparty/DBoW2/CMakeLists.txt"}) {
        auto target = orb / rel;
        std::filesystem::create_directories(target.parent_path());
        std::filesystem::copy_file(real / rel, target);
    }
    write_file(orb / "Examples/x.cc", "int x() {}\n");
    write_file(orb / "build/y.o", "obj\n");
    write_file(pang / "CMakeLists.txt", "cmake_minimum_required(VERSION 3.16)\n");

    auto vocab = orb / "Vocabulary";
    std::filesystem::create_directories(vocab);
    write_file(vocab / "ORBvoc.txt", "dummy vocabulary\n");
    umi_slam::RunInheritOptions tar_opt;
    tar_opt.cwd = vocab;
    int tar_rc = umi_slam::run_inherit(
        {EDS_CMAKE_COMMAND, "-E", "tar", "czf", "ORBvoc.txt.tar.gz", "ORBvoc.txt"}, tar_opt);
    REQUIRE(tar_rc == 0);

    std::map<std::string, std::string> before;
    for (const char* rel : {"CMakeLists.txt", "include/System.h", "src/System.cc",
                            "Thirdparty/DBoW2/CMakeLists.txt"}) {
        before[rel] = slam_common::sha256_file(orb / rel);
    }

    umi_slam::BackendBuildOptions opt;
    opt.source_root = src;
    opt.build_dir = td.path / "build";
    opt.cmake_exe = EDS_CMAKE_COMMAND;

    REQUIRE(umi_slam::stage_backend_sources(opt, "digest-one"));
    auto staged = opt.build_dir / "ORB_SLAM3";
    REQUIRE(slam_common::read_text_utf8(staged / "CMakeLists.txt").find("# Build examples") == std::string::npos);
    REQUIRE(slam_common::read_text_utf8(staged / "include/System.h").find("OfflineSaveMapPoints") != std::string::npos);
    REQUIRE(!std::filesystem::exists(staged / "Examples/x.cc"));
    REQUIRE(!std::filesystem::exists(staged / "build/y.o"));
    REQUIRE(std::filesystem::is_regular_file(staged / "Vocabulary/ORBvoc.txt"));
    REQUIRE(slam_common::read_text_utf8(opt.build_dir / "source.sha256") == "digest-one");

    for (const auto& kv : before) {
        REQUIRE(slam_common::sha256_file(orb / kv.first) == kv.second);
    }

    REQUIRE(!umi_slam::stage_backend_sources(opt, "digest-one"));
    require_throws([&]() { umi_slam::stage_backend_sources(opt, "digest-two"); }, "Source changed");

    TempDir td2("stage_inc");
    auto broken = td2.path / "build";
    std::filesystem::create_directories(broken / "ORB_SLAM3");
    umi_slam::BackendBuildOptions opt2;
    opt2.source_root = src;
    opt2.build_dir = broken;
    opt2.cmake_exe = EDS_CMAKE_COMMAND;
    require_throws([&]() { umi_slam::stage_backend_sources(opt2, "digest-x"); }, "Incomplete source staging");
}

void test_write_backend_config_survives_relocation() {
    TempDir td("bcfg_reloc");
    auto b1 = td.path / "one";
    auto b2 = td.path / "two";
    std::filesystem::create_directories(b1 / "runner");
    std::filesystem::create_directories(b1 / "Vocabulary");
    write_file(b1 / "runner/umi_slam_runner", "bin");
    write_file(b1 / "Vocabulary/ORBvoc.txt", "voc");

    umi_slam::write_backend_config(
        b1, b1 / "runner/umi_slam_runner", b1 / "Vocabulary/ORBvoc.txt",
        {b1 / "runner"}, "abc123");

    std::filesystem::copy(b1, b2, std::filesystem::copy_options::recursive);
    auto loaded = umi_slam::load_backend_config(b2 / "backend.json");
    REQUIRE(std::filesystem::weakly_canonical(std::filesystem::u8path(loaded.binary)) ==
            std::filesystem::weakly_canonical(b2 / "runner/umi_slam_runner"));
    REQUIRE(std::filesystem::weakly_canonical(std::filesystem::u8path(loaded.vocabulary)) ==
            std::filesystem::weakly_canonical(b2 / "Vocabulary/ORBvoc.txt"));
    REQUIRE(loaded.library_paths.size() == 1);
    REQUIRE(std::filesystem::weakly_canonical(std::filesystem::u8path(loaded.library_paths[0])) ==
            std::filesystem::weakly_canonical(b2 / "runner"));
}

void test_pipeline_cli_chinese_path_fails_cleanly() {
    TempDir td("cli_cn");
    auto in_dir = td.path / std::filesystem::u8path("输入 数据 & 测试");
    auto out_dir = td.path / std::filesystem::u8path("输出 结果 & 目录");
    std::filesystem::create_directories(in_dir);
    auto settings = repo_root() / "software/umi_slam/profiles/umi_20260908.yaml";
    auto profile = repo_root() / "software/umi_slam/profiles/umi_20260908.json";

    std::vector<std::string> args = {
        EDS_PIPELINE_EXE,
        "--input", in_dir.u8string(),
        "--output", out_dir.u8string(),
        "--settings", settings.u8string(),
        "--profile", profile.u8string(),
        "--prepare-only"
    };
    auto result = umi_slam::run_subprocess_capture(args);
    REQUIRE(result.first == 1);
    REQUIRE(result.second.find("ERROR") != std::string::npos);
    REQUIRE(std::filesystem::is_regular_file(out_dir / "job.json"));
    auto st = slam_common::read_json_safe(out_dir / "job.json");
    REQUIRE(st.get("status").as_string() == "failed");
}

}  // namespace

int main() {
    int passed = 0;
    auto run_test = [&](const char* name, auto&& fn) {
        std::cout << "[RUN ] " << name << std::endl;
        fn();
        std::cout << "[ OK ] " << name << std::endl;
        ++passed;
    };
    try {
        run_test("test_uart_fragments_and_units", test_uart_fragments_and_units);
        run_test("test_truncated_uart_rejected", test_truncated_uart_rejected);
        run_test("test_timestamp_precision_and_order", test_timestamp_precision_and_order);
        run_test("test_export_preserves_imu_pose_and_gaps", test_export_preserves_imu_pose_and_gaps);
        run_test("test_uninitialized_not_success", test_uninitialized_not_success);
        run_test("test_export_float_ns_roundoff_matches_original_stamp", test_export_float_ns_roundoff_matches_original_stamp);
        run_test("test_prepare_camera_imu_shift_and_size", test_prepare_camera_imu_shift_and_size);
        run_test("test_load_backend_config_resolves_relative_paths", test_load_backend_config_resolves_relative_paths);
        run_test("test_patch_orb_applies_all_patches_and_rejects_mismatch", test_patch_orb_applies_all_patches_and_rejects_mismatch);
        run_test("test_paths_with_spaces_chinese_and_ampersand", test_paths_with_spaces_chinese_and_ampersand);
        run_test("test_refuses_nested_output", test_refuses_nested_output);
        run_test("test_sha256_streaming_and_canonical_crlf", test_sha256_streaming_and_canonical_crlf);
        run_test("test_update_repo_checksums_temp_tree", test_update_repo_checksums_temp_tree);
        run_test("test_backend_source_digest_tracks_source_files_only", test_backend_source_digest_tracks_source_files_only);
        run_test("test_stage_backend_sources_patches_stamps_and_guards", test_stage_backend_sources_patches_stamps_and_guards);
        run_test("test_write_backend_config_survives_relocation", test_write_backend_config_survives_relocation);
        run_test("test_pipeline_cli_chinese_path_fails_cleanly", test_pipeline_cli_chinese_path_fails_cleanly);
    } catch (const std::exception& exc) {
        std::cerr << "[FAIL] " << exc.what() << std::endl;
        return 1;
    }
    std::cout << "All " << passed << " pipeline tests passed." << std::endl;
    return 0;
}
