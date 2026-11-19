#include "calibration_targets/detector.hpp"
#include "calibration_targets/generator.hpp"
#include "camera_imu_calibration/engine.hpp"
#include "handeye_calibration/handeye_3d.hpp"
#include "handeye_calibration/n_point_2d.hpp"
#include "preview_3d/preview_widget.hpp"
#include "slam_common/geometry.hpp"
#include "slam_common/json.hpp"
#include "slam_common/ms901m.hpp"
#include "slam_common/sha256.hpp"
#include "slam_common/trajectory_io.hpp"
#include "umi_slam/runtime.hpp"
#include "vision_calibration/planar_and_distortion.hpp"

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <csignal>
#include <sys/types.h>
#include <unistd.h>
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;

#define REQUIRE(cond) do { if (!(cond)) { throw std::runtime_error(std::string("REQUIRE failed at line ") + std::to_string(__LINE__) + ": " #cond); } } while(0)
#define REQUIRE_NEAR(a, b, tol) do { double _va = (a), _vb = (b); if (!std::isfinite(_va) || std::abs(_va - _vb) > (tol)) { std::ostringstream _os; _os << "REQUIRE_NEAR failed at line " << __LINE__ << ": " << _va << " vs " << _vb << " (tol=" << (tol) << ")"; throw std::runtime_error(_os.str()); } } while(0)

template <typename Fn>
void require_throws(Fn&& fn, const std::string& substr = "") {
    bool threw = false;
    try {
        fn();
    } catch (const std::exception& exc) {
        threw = true;
        if (!substr.empty() && std::string(exc.what()).find(substr) == std::string::npos) {
            throw std::runtime_error(std::string("Exception message mismatch: got '") + exc.what() + "', expected '" + substr + "'");
        }
    }
    if (!threw) throw std::runtime_error("Expected exception was not thrown");
}

struct TempDir {
    std::filesystem::path path;
    explicit TempDir(const std::string& tag) {
        static std::atomic<int> counter{0};
        auto base = std::filesystem::temp_directory_path() / ("eds_cpp_station_" + tag + "_" + std::to_string(++counter));
        std::filesystem::remove_all(base);
        std::filesystem::create_directories(base);
        path = base;
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

std::filesystem::path repo_root() {
    return std::filesystem::u8path(EDS_REPO_ROOT);
}

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

bool pid_alive(int64_t pid) {
#ifdef _WIN32
    HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, static_cast<DWORD>(pid));
    if (!h) return false;
    DWORD code = 0;
    BOOL ok = GetExitCodeProcess(h, &code);
    CloseHandle(h);
    return ok && code == STILL_ACTIVE;
#else
    return kill(static_cast<pid_t>(pid), 0) == 0;
#endif
}

std::vector<uint8_t> make_ms901m_pkt(uint8_t kind, const std::vector<uint8_t>& vals) {
    std::vector<uint8_t> b = {0x55, 0x55, kind, static_cast<uint8_t>(vals.size())};
    b.insert(b.end(), vals.begin(), vals.end());
    uint32_t sum = 0;
    for (uint8_t x : b) sum += x;
    b.push_back(static_cast<uint8_t>(sum & 0xFFu));
    return b;
}

std::vector<uint8_t> pack_i16x6(int16_t a0, int16_t a1, int16_t a2, int16_t a3, int16_t a4, int16_t a5) {
    std::vector<uint8_t> out(12);
    int16_t v[6] = {a0, a1, a2, a3, a4, a5};
    for (int i = 0; i < 6; ++i) {
        uint16_t u = static_cast<uint16_t>(v[i]);
        out[i * 2] = static_cast<uint8_t>(u & 0xFFu);
        out[i * 2 + 1] = static_cast<uint8_t>((u >> 8) & 0xFFu);
    }
    return out;
}

std::vector<uint8_t> pack_uart_block(uint64_t ts, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> out(12 + payload.size());
    for (int i = 0; i < 8; ++i) out[i] = static_cast<uint8_t>((ts >> (8 * i)) & 0xFFu);
    uint32_t sz = static_cast<uint32_t>(payload.size());
    for (int i = 0; i < 4; ++i) out[8 + i] = static_cast<uint8_t>((sz >> (8 * i)) & 0xFFu);
    std::copy(payload.begin(), payload.end(), out.begin() + 12);
    return out;
}

std::filesystem::path make_fake_backend(const std::filesystem::path& dir) {
    std::filesystem::create_directories(dir);
#ifdef _WIN32
    std::string bin_name = "umi_slam_runner.exe";
#else
    std::string bin_name = "umi_slam_runner";
#endif
    write_file(dir / bin_name, "bin");
    write_file(dir / "ORBvoc.txt", "voc");
    slam_common::JsonValue cfg = slam_common::JsonValue::object();
    cfg["binary"] = bin_name;
    cfg["vocabulary"] = "ORBvoc.txt";
    cfg["library_paths"] = slam_common::JsonValue(slam_common::JsonValue::Array{"."});
    auto cfg_p = dir / "backend.json";
    slam_common::save_json(cfg_p, cfg);
    return cfg_p;
}

void test_native_chinese_paths_and_command() {
    TempDir td("win_native_cn");
    auto src = td.path / std::filesystem::u8path("中文 输入 &");
    std::filesystem::create_directories(src);
    auto settings = td.path / std::filesystem::u8path("标定.yaml");
    write_file(settings, "settings");
    auto device = repo_root() / "software/slam_station/devices/generic_euroc.json";
    auto cfg_p = make_fake_backend(td.path / "backend");

    auto cmd = umi_slam::command(src, td.path / "out/result", settings, device, 5, 0, 0.0, false, cfg_p, true);
    REQUIRE(std::find(cmd.begin(), cmd.end(), "--backend") != cmd.end());
    REQUIRE(std::find(cmd.begin(), cmd.end(), "--viewer") != cmd.end());
    REQUIRE(std::find(cmd.begin(), cmd.end(), "--input") != cmd.end());
}

void test_refuse_nested_existing_and_invalid_params() {
    TempDir td("nested");
    auto src = td.path / "src";
    std::filesystem::create_directories(src);
    auto settings = td.path / "s.yaml";
    write_file(settings, "settings");
    auto device = repo_root() / "software/slam_station/devices/generic_euroc.json";
    auto cfg_p = make_fake_backend(td.path / "backend");

    require_throws([&]() { umi_slam::command(src, src / "out", settings, device, 5, 0, 0.0, false, cfg_p); });
    require_throws([&]() { umi_slam::command(src, td.path, settings, device, 5, 0, 0.0, false, cfg_p); });
    require_throws([&]() { umi_slam::command(src, td.path / "out", settings, device, 0, 0, 0.0, false, cfg_p); });
    require_throws([&]() { umi_slam::command(src, td.path / "out", settings, device, 5, 0, std::nan(""), false, cfg_p); });
    require_throws([&]() { umi_slam::command(src, td.path / "out", settings, device, 5, -1, 0.0, false, cfg_p); });
}

void test_unknown_protocol_units_and_bad_transform() {
    TempDir td("proto");
    auto device = repo_root() / "software/slam_station/devices/generic_euroc.json";
    auto d = slam_common::load_device(device);

    for (const auto& kv : std::vector<std::pair<std::string, slam_common::JsonValue>>{
             {"adapter", slam_common::JsonValue("ego_video")},
             {"gyro_unit", slam_common::JsonValue("deg/s")},
             {"camera_to_imu_offset_ns", slam_common::JsonValue(0.1)},
         }) {
        auto bad = d;
        bad[kv.first] = kv.second;
        auto p = td.path / "device.json";
        slam_common::save_json(p, bad);
        require_throws([&]() { slam_common::load_device(p); });
    }
    for (const auto& entry : std::filesystem::directory_iterator(repo_root() / "software/slam_station/devices")) {
        if (entry.path().extension() == ".json") {
            slam_common::load_device(entry.path());
        }
    }
    slam_common::JsonValue refl = slam_common::JsonValue::array();
    refl.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{1.0, 0.0, 0.0, 0.0}));
    refl.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{0.0, 1.0, 0.0, 0.0}));
    refl.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{0.0, 0.0, -1.0, 0.0}));
    refl.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{0.0, 0.0, 0.0, 1.0}));
    require_throws([&]() { slam_common::validate_transform(refl); });
}

void test_euroc_does_not_accept_legacy_raw() {
    TempDir td("euroc_raw");
    auto src = td.path / "src";
    std::filesystem::create_directories(src);
    write_file(src / "stereo.mkv", "fake");
    auto dev = slam_common::load_device(repo_root() / "software/slam_station/devices/generic_euroc.json");
    require_throws([&]() { slam_common::check_dataset(src, dev); }, "EuRoC");
}

void test_target_transform_rotates_translation_and_preserves_gaps() {
    TempDir td("target_tf");
    auto csv_p = td.path / "imu_trajectory.csv";
    std::string orig = "timestamp_ns,time_from_input_start_s,x_m,y_m,z_m,qx,qy,qz,qw,segment_id\n"
                       "1789459434308037880,0,1,2,3,0,0,0.7071067811865476,0.7071067811865476,3\n";
    write_file(csv_p, orig);

    slam_common::JsonValue dev = slam_common::JsonValue::object();
    dev["target_frame"] = "gripper_tcp";
    dev["timestamp_clock"] = "hardware";
    slam_common::JsonValue tf = slam_common::JsonValue::array();
    tf.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{1.0, 0.0, 0.0, 1.0}));
    tf.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{0.0, 1.0, 0.0, 0.0}));
    tf.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{0.0, 0.0, 1.0, 0.0}));
    tf.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{0.0, 0.0, 0.0, 1.0}));
    dev["T_imu_target"] = tf;

    auto target = slam_common::export_target(td.path, dev);
    REQUIRE(target.has_value());
    auto pts = slam_common::read_csv_points(*target);
    REQUIRE(pts.size() == 1);
    REQUIRE_NEAR(pts[0].pos.x(), 1.0, 1e-6);
    REQUIRE_NEAR(pts[0].pos.y(), 3.0, 1e-6);
    REQUIRE(pts[0].segment_id == "3");
    REQUIRE(slam_common::read_text_utf8(csv_p) == orig);
    require_throws([&]() { slam_common::export_target(td.path, dev); });
    slam_common::JsonValue dev_null = slam_common::JsonValue::object();
    dev_null["T_imu_target"] = slam_common::JsonValue(nullptr);
    REQUIRE(!slam_common::export_target(td.path, dev_null).has_value());
}

void test_rotation_roundtrip_at_180_degrees() {
    for (const auto& q : std::vector<Eigen::Vector4d>{
             {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0.5, 0.5, 0.5, 0.5}, {0, 0, 0, 1}}) {
        Eigen::Matrix3d r = slam_common::quaternion_to_rot_strict(q);
        Eigen::Matrix3d actual = slam_common::quaternion_to_rot_strict(slam_common::rot_to_quat_xyzw(r));
        REQUIRE((r - actual).cwiseAbs().maxCoeff() < 1e-6);
    }
}

void test_cancel_before_start_never_launches_process() {
    std::atomic<bool> cancel{true};
    TempDir td("cancel_pre");
    REQUIRE(umi_slam::execute_command({"nonexistent_binary_should_not_run"}, cancel, td.path / "log.txt") == 130);
    REQUIRE(!std::filesystem::exists(td.path / "log.txt"));
}

void test_job_no_false_success_and_config_snapshot() {
    TempDir td("job_snap");
    auto src = td.path / "src";
    std::filesystem::create_directories(src / "mav0/cam0/data");
    std::filesystem::create_directories(src / "mav0/cam1/data");
    std::filesystem::create_directories(src / "mav0/imu0");
    write_file(src / "mav0/imu0/data.csv", "#\n");
    write_file(src / "timestamps.txt", "");
    auto settings = td.path / "settings.yaml";
    write_file(settings, "settings");
    auto device = repo_root() / "software/slam_station/devices/generic_euroc.json";
    auto cfg_p = make_fake_backend(td.path / "backend");

    umi_slam::JobHooks jhooks;
    jhooks.diagnose_fn = [](const std::optional<std::filesystem::path>&) {
        slam_common::JsonValue d = slam_common::JsonValue::object();
        d["mode"] = "native";
        return d;
    };
    jhooks.execute_fn = [](const std::vector<std::string>&, std::atomic<bool>&, const std::filesystem::path&) {
        return 0;
    };

    umi_slam::Job job(src, td.path / "results", settings, device, 5, 0, 0.0, false, cfg_p, false, jhooks);
    write_file(settings, "changed");
    REQUIRE(slam_common::read_text_utf8(job.folder() / "config/settings.yaml") == "settings");
    job.run();
    auto st = job.state();
    REQUIRE(st.get("status").as_string() == "failed");
    REQUIRE(st.get("error").as_string().find("后端未完成") != std::string::npos);
}

void test_native_backend_resolution_and_cancel() {
    TempDir td("native_res");
    auto bdir = td.path / "backend";
    auto cfg_p = make_fake_backend(bdir);
#ifdef _WIN32
    std::string bin_name = "umi_slam_runner.exe";
#else
    std::string bin_name = "umi_slam_runner";
#endif

    auto info = umi_slam::find_native_backend(cfg_p);
    REQUIRE(info.has_value());
    REQUIRE(info->binary == std::filesystem::weakly_canonical(bdir / bin_name));

    auto src = td.path / "src";
    std::filesystem::create_directories(src);
    auto settings = td.path / "s.yaml";
    write_file(settings, "settings");
    auto device = repo_root() / "software/slam_station/devices/generic_euroc.json";

    auto cmd = umi_slam::native_command(src, td.path / "out/result", settings, device, 5, 0, 0.0, true, cfg_p);
    REQUIRE(std::find(cmd.begin(), cmd.end(), "--backend") != cmd.end());
    REQUIRE(std::find(cmd.begin(), cmd.end(), "--prepare-only") != cmd.end());
    REQUIRE(std::find(cmd.begin(), cmd.end(), "--viewer") == cmd.end());

    auto cmd_v = umi_slam::native_command(src, td.path / "out2/result", settings, device, 5, 0, 0.0, false, cfg_p, true);
    REQUIRE(std::find(cmd_v.begin(), cmd_v.end(), "--viewer") != cmd_v.end());

    auto diag = umi_slam::diagnose(cfg_p);
    REQUIRE(diag.get("mode").as_string() == "native");
}

void test_native_execute_kills_grandchild_tree_and_reconciles_job_state(const std::string& self_exe) {
    TempDir td("tree_kill");
    auto pids_file = td.path / "pids.json";
    std::atomic<bool> cancel{false};
    int rc = -1;
    std::thread worker([&]() {
        rc = umi_slam::execute_command(
            {self_exe, "--helper-spawn-tree", pids_file.string()},
            cancel,
            td.path / "tree.log");
    });

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!std::filesystem::is_regular_file(pids_file) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    REQUIRE(std::filesystem::is_regular_file(pids_file));
    auto pids = slam_common::read_json_file(pids_file);
    int64_t child_pid = pids.get("child").as_int();
    int64_t gc_pid = pids.get("grandchild").as_int();
    REQUIRE(pid_alive(gc_pid));

    cancel.store(true);
    worker.join();
    REQUIRE(rc == 130);
    for (int i = 0; i < 20; ++i) {
        if (!pid_alive(child_pid) && !pid_alive(gc_pid)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    REQUIRE(!pid_alive(child_pid));
    REQUIRE(!pid_alive(gc_pid));

    // Verify Job::run reconciles stuck result/job.json -> cancelled
    auto src = td.path / "src_cancel";
    std::filesystem::create_directories(src / "mav0/cam0/data");
    std::filesystem::create_directories(src / "mav0/cam1/data");
    std::filesystem::create_directories(src / "mav0/imu0");
    write_file(src / "mav0/cam0/data.csv", "#\n");
    write_file(src / "mav0/cam1/data.csv", "#\n");
    write_file(src / "mav0/imu0/data.csv", "#\n");
    auto settings = td.path / "s.yaml";
    write_file(settings, "settings");
    auto device = repo_root() / "software/slam_station/devices/generic_euroc.json";
    auto cfg_p = make_fake_backend(td.path / "backend_cancel");

    umi_slam::JobHooks jhooks;
    jhooks.diagnose_fn = [](const std::optional<std::filesystem::path>&) {
        slam_common::JsonValue d = slam_common::JsonValue::object();
        d["mode"] = "native";
        return d;
    };
    jhooks.execute_fn = [](const std::vector<std::string>&, std::atomic<bool>&, const std::filesystem::path&) {
        return 130;
    };

    umi_slam::Job job(src, td.path / "jobs_cancel", settings, device, 5, 0, 0.0, false, cfg_p, false, jhooks);
    std::filesystem::create_directories(job.result_dir());
    slam_common::JsonValue stuck = slam_common::JsonValue::object();
    stuck["status"] = "running";
    stuck["started_unix_s"] = 1.0;
    slam_common::save_json(job.result_dir() / "job.json", stuck);

    job.run();
    REQUIRE(job.state().get("status").as_string() == "cancelled");
    auto reconciled = slam_common::read_json_file(job.result_dir() / "job.json");
    REQUIRE(reconciled.get("status").as_string() == "cancelled");
    REQUIRE(reconciled.contains("finished_unix_s"));
}

void test_load_preview_data_full_job_and_fallbacks() {
    TempDir td("preview");
    auto job_dir = td.path / "run_test";
    auto res_dir = job_dir / "result";
    std::filesystem::create_directories(res_dir);

    slam_common::JsonValue sj = slam_common::JsonValue::object(); sj["status"] = "completed";
    slam_common::save_json(job_dir / "station_job.json", sj);
    slam_common::JsonValue qj = slam_common::JsonValue::object(); qj["coverage_fraction"] = 0.95; qj["map_points"] = 5;
    slam_common::save_json(res_dir / "quality.json", qj);
    write_file(res_dir / "imu_trajectory.csv",
               "timestamp_ns,time_from_input_start_s,x_m,y_m,z_m,qx,qy,qz,qw,segment_id\n"
               "100,0.0,0.0,0.0,0.0,0,0,0,1,0\n"
               "200,0.1,1.0,2.0,3.0,0,0,0,1,0\n");
    write_file(res_dir / "keyframes_euroc.txt", "# comment\n100 0.0 0.0 0.0 0 0 0 1\n200 1.0 2.0 3.0 0 0 0 1\n");
    write_file(res_dir / "map_points.csv", "x_m,y_m,z_m\n0.1,0.2,0.3\n1.1,1.2,1.3\n2.1,2.2,2.3\n3.1,3.2,3.3\n4.1,4.2,4.3\n");

    slam_common::JsonValue dev = slam_common::JsonValue::object();
    dev["target_frame"] = "gripper";
    dev["timestamp_clock"] = "hardware";
    slam_common::JsonValue tf = slam_common::JsonValue::array();
    tf.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{1.0, 0.0, 0.0, 0.5}));
    tf.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{0.0, 1.0, 0.0, 0.0}));
    tf.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{0.0, 0.0, 1.0, 0.0}));
    tf.push_back(slam_common::JsonValue(slam_common::JsonValue::Array{0.0, 0.0, 0.0, 1.0}));
    dev["T_imu_target"] = tf;
    slam_common::export_target(res_dir, dev);

    auto prev = slam_common::load_preview_data(job_dir, 3);
    REQUIRE(prev.traj_source == "imu_trajectory.csv");
    REQUIRE(prev.points.size() == 2);
    REQUIRE_NEAR(prev.points[1].pos.x(), 1.0, 1e-6);
    REQUIRE(prev.target_frame == "gripper");
    REQUIRE_NEAR(prev.target_points[0].pos.x(), 0.5, 1e-6);
    REQUIRE(prev.keyframes.size() == 2);
    REQUIRE(prev.total_map_points == 5);
    REQUIRE(prev.map_points.size() <= 3);
    REQUIRE(prev.station_job.get("status").as_string() == "completed");
    REQUIRE_NEAR(prev.quality.get("coverage_fraction").as_double(), 0.95, 1e-6);

    auto [ctr, span] = preview_3d::compute_bounds({{0, 0, 0}, {2, 4, 6}});
    REQUIRE_NEAR(ctr.x(), 1.0, 1e-6);
    REQUIRE_NEAR(span, 6.0, 1e-6);
    auto col_mp = preview_3d::colorize_map_points(prev.map_points);
    REQUIRE(col_mp.size() == prev.map_points.size());

    auto tum_dir = td.path / "tum_only";
    std::filesystem::create_directories(tum_dir);
    write_file(tum_dir / "imu_euroc.txt", "# header\n1000 0.5 -0.5 1.5 0 0 0 1\n");
    auto tum_prev = slam_common::load_preview_data(tum_dir);
    REQUIRE(tum_prev.traj_source == "imu_euroc.txt");
    REQUIRE_NEAR(tum_prev.points[0].pos.x(), 0.5, 1e-6);

    auto empty_dir = td.path / "empty_dir";
    std::filesystem::create_directories(empty_dir);
    require_throws([&]() { slam_common::load_preview_data(empty_dir); });
    require_throws([&]() { slam_common::load_preview_data(td.path / "missing"); });
}

void test_convert_kalibr_reference_matches_orb_yaml() {
    TempDir td("kalibr");
    auto camchain = repo_root() / "calibration/reference_20260908/stereo_imu-camchain-imucam.yaml";
    auto imu_yaml = repo_root() / "calibration/reference_20260908/stereo_imu-imu.yaml";
    auto [yp, jp] = camera_imu_calibration::convert_kalibr_files(camchain, imu_yaml, td.path / "kalibr_converted", 125.0);
    REQUIRE(std::filesystem::is_regular_file(yp));
    REQUIRE(std::filesystem::is_regular_file(jp));

    cv::FileStorage fs(yp.string(), cv::FileStorage::READ);
    REQUIRE(fs.isOpened());
    REQUIRE_NEAR(fs["Camera1.fx"].real(), 650.6226640994397, 1e-4);
    REQUIRE_NEAR(fs["Camera2.fx"].real(), 653.8838303309726, 1e-4);
    REQUIRE_NEAR(fs["Stereo.b"].real(), 0.06635707674365307, 1e-5);
    cv::Mat T_c1_c2, T_b_c1;
    fs["Stereo.T_c1_c2"] >> T_c1_c2;
    fs["IMU.T_b_c1"] >> T_b_c1;
    T_c1_c2.convertTo(T_c1_c2, CV_64F);
    T_b_c1.convertTo(T_b_c1, CV_64F);
    REQUIRE_NEAR(T_c1_c2.at<double>(0, 3), 0.06627415, 1e-5);
    REQUIRE_NEAR(T_b_c1.at<double>(0, 3), -0.06877166, 1e-5);

    auto prof = slam_common::load_device(jp);
    REQUIRE(prof.get("camera_to_imu_offset_ns").as_int() == 249373);
}

void test_synthetic_stereo_and_imu_extrinsic_calibration() {
    cv::Size image_size(640, 480);
    cv::Mat K0_true(cv::Matx33d(420.0, 0.0, 322.0, 0.0, 421.0, 238.0, 0.0, 0.0, 1.0));
    cv::Mat K1_true(cv::Matx33d(418.0, 0.0, 319.0, 0.0, 419.0, 241.0, 0.0, 0.0, 1.0));
    cv::Mat D0_true(cv::Vec<double, 5>(-0.18, 0.04, 0.0005, -0.0004, 0.0));
    cv::Mat D1_true(cv::Vec<double, 5>(-0.19, 0.05, -0.0003, 0.0006, 0.0));

    cv::Mat R_12_cv;
    cv::Rodrigues(cv::Vec3d(0.01, -0.015, 0.008), R_12_cv);
    Eigen::Matrix4d T_c1_c2_true = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) T_c1_c2_true(r, c) = R_12_cv.at<double>(r, c);
    }
    T_c1_c2_true.topRightCorner<3, 1>() = Eigen::Vector3d(0.070, 0.001, -0.002);
    Eigen::Matrix4d T_c2_c1_true = T_c1_c2_true.inverse();

    cv::Mat R_bc_cv;
    cv::Rodrigues(cv::Vec3d(0.04, -0.03, 1.57), R_bc_cv);
    Eigen::Matrix3d R_b_c1_true;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) R_b_c1_true(r, c) = R_bc_cv.at<double>(r, c);
    }
    Eigen::Vector3d t_b_c1_true(-0.045, 0.020, -0.015);
    Eigen::Vector3d p_c1_b_true = -(R_b_c1_true.transpose() * t_b_c1_true);

    std::vector<cv::Point3f> obj_pts;
    for (int r = 0; r < 5; ++r) {
        for (int c = 0; c < 6; ++c) {
            obj_pts.emplace_back(static_cast<float>(c * 0.04), static_cast<float>(r * 0.04), 0.0f);
        }
    }

    auto Rwc_at = [](double tt) {
        cv::Mat R_cv;
        cv::Rodrigues(cv::Vec3d(0.28 * std::sin(1.7 * tt), 0.32 * std::cos(1.3 * tt), 0.25 * std::sin(2.1 * tt)), R_cv);
        Eigen::Matrix3d R;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) R(r, c) = R_cv.at<double>(r, c);
        }
        return R;
    };
    auto pw_b_at = [&](double tt) -> Eigen::Vector3d {
        Eigen::Vector3d pw_c(0.08 * std::sin(tt), 0.06 * std::cos(1.4 * tt), -0.65 - 0.08 * std::sin(0.9 * tt));
        return pw_c + Rwc_at(tt) * p_c1_b_true;
    };

    std::vector<camera_imu_calibration::Observation> obs_list;
    for (int idx = 0; idx < 24; ++idx) {
        double t = 2.3 * static_cast<double>(idx) / 23.0;
        Eigen::Matrix3d R_wc1 = Rwc_at(t);
        Eigen::Vector3d t_wc1(0.08 * std::sin(t), 0.06 * std::cos(1.4 * t), -0.65 - 0.08 * std::sin(0.9 * t));
        Eigen::Matrix4d T_w_c1 = Eigen::Matrix4d::Identity();
        T_w_c1.topLeftCorner<3, 3>() = R_wc1;
        T_w_c1.topRightCorner<3, 1>() = t_wc1;
        Eigen::Matrix4d T_c1_w = T_w_c1.inverse();
        Eigen::Matrix4d T_c2_w = T_c2_c1_true * T_c1_w;

        auto proj_cam = [&](const Eigen::Matrix4d& T_cw, const cv::Mat& K, const cv::Mat& D) {
            cv::Mat R_cv(3, 3, CV_64F), rv, tv(3, 1, CV_64F);
            for (int r = 0; r < 3; ++r) {
                tv.at<double>(r) = T_cw(r, 3);
                for (int c = 0; c < 3; ++c) R_cv.at<double>(r, c) = T_cw(r, c);
            }
            cv::Rodrigues(R_cv, rv);
            std::vector<cv::Point2f> pts;
            cv::projectPoints(obj_pts, rv, tv, K, D, pts);
            return pts;
        };

        camera_imu_calibration::Observation obs;
        obs.ts_ns = static_cast<int64_t>(std::llround((1.0 + t) * 1e9));
        obs.obj_pts[0] = obj_pts;
        obs.img_pts[0] = proj_cam(T_c1_w, K0_true, D0_true);
        obs.obj_pts[1] = obj_pts;
        obs.img_pts[1] = proj_cam(T_c2_w, K1_true, D1_true);
        obs_list.push_back(std::move(obs));
    }

    auto st = camera_imu_calibration::calibrate_stereo_from_observations(obs_list, image_size);
    REQUIRE(st.stereo_rms < 0.2);
    REQUIRE_NEAR(st.baseline, (T_c1_c2_true.topRightCorner<3, 1>().norm()), 0.002);
    REQUIRE_NEAR(st.T_c1_c2(0, 3), T_c1_c2_true(0, 3), 0.002);

    double shift_true = 0.015;
    std::vector<int64_t> imu_times(676);
    std::vector<Eigen::Matrix<double, 6, 1>> imu_vals(676);
    Eigen::Vector3d g_world(0.0, 0.0, -9.80665);
    for (int k = 0; k < 676; ++k) {
        double ti = -0.2 + 2.7 * static_cast<double>(k) / 675.0;
        imu_times[k] = static_cast<int64_t>(std::llround((1.0 + ti) * 1e9));
        double tc = ti - shift_true;
        double eps = 1e-4;
        Eigen::Matrix3d dR = Rwc_at(tc - eps).transpose() * Rwc_at(tc + eps);
        cv::Mat dR_cv(3, 3, CV_64F), rv;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) dR_cv.at<double>(r, c) = dR(r, c);
        }
        cv::Rodrigues(dR_cv, rv);
        rv.convertTo(rv, CV_64F);
        Eigen::Vector3d w_c(rv.at<double>(0) / (2 * eps), rv.at<double>(1) / (2 * eps), rv.at<double>(2) / (2 * eps));
        Eigen::Vector3d p_dd = (pw_b_at(tc + eps) - 2.0 * pw_b_at(tc) + pw_b_at(tc - eps)) / (eps * eps);
        Eigen::Matrix3d R_wb_true = Rwc_at(tc) * R_b_c1_true.transpose();
        imu_vals[k].head<3>() = R_b_c1_true * w_c;
        imu_vals[k].tail<3>() = R_wb_true.transpose() * (p_dd - g_world);
    }

    auto imu_res = camera_imu_calibration::estimate_cam_imu_extrinsic(obs_list, st.K0, st.D0, imu_times, imu_vals);
    REQUIRE_NEAR(static_cast<double>(imu_res.timeshift_ns) / 1e9, shift_true, 0.004);
    REQUIRE(imu_res.translation_valid);
    REQUIRE(imu_res.translation_rows >= 6);
    REQUIRE(!imu_res.timeshift_at_bound);
    REQUIRE(!imu_res.timeshift_low_corr);
    double rot_err = (imu_res.T_b_c1.topLeftCorner<3, 3>() * R_b_c1_true.transpose() - Eigen::Matrix3d::Identity()).norm();
    REQUIRE(rot_err < 0.06);

    // Case 1: Zero rotational excitation raises
    auto imu_zero = imu_vals;
    for (auto& v : imu_zero) v.head<3>().setZero();
    require_throws([&]() {
        camera_imu_calibration::estimate_cam_imu_extrinsic(obs_list, st.K0, st.D0, imu_times, imu_zero);
    }, "旋转激励不足");

    // Case 2: Large timeshift (+90ms) beyond ±50ms bound
    double shift_large = 0.090;
    std::vector<int64_t> imu_t_large(726);
    std::vector<Eigen::Matrix<double, 6, 1>> imu_large(726);
    for (int k = 0; k < 726; ++k) {
        double ti = -0.3 + 2.9 * static_cast<double>(k) / 725.0;
        imu_t_large[k] = static_cast<int64_t>(std::llround((1.0 + ti) * 1e9));
        double tc = ti - shift_large;
        Eigen::Matrix3d dR = Rwc_at(tc - 1e-4).transpose() * Rwc_at(tc + 1e-4);
        cv::Mat dR_cv(3, 3, CV_64F), rv;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) dR_cv.at<double>(r, c) = dR(r, c);
        }
        cv::Rodrigues(dR_cv, rv);
        rv.convertTo(rv, CV_64F);
        Eigen::Vector3d w_c(rv.at<double>(0) / 2e-4, rv.at<double>(1) / 2e-4, rv.at<double>(2) / 2e-4);
        imu_large[k].head<3>() = R_b_c1_true * w_c;
        imu_large[k].tail<3>() = Rwc_at(tc).transpose() * Eigen::Vector3d(0.0, 0.0, 9.80665);
    }
    auto bounded = camera_imu_calibration::estimate_cam_imu_extrinsic(obs_list, st.K0, st.D0, imu_t_large, imu_large);
    REQUIRE(bounded.timeshift_at_bound);
    REQUIRE(!bounded.warnings.empty());

    auto wide = camera_imu_calibration::estimate_cam_imu_extrinsic(obs_list, st.K0, st.D0, imu_t_large, imu_large, 0, 0.12);
    REQUIRE(!wide.timeshift_at_bound);
    REQUIRE_NEAR(static_cast<double>(wide.timeshift_ns) / 1e9, shift_large, 0.004);

    // Case 3: Sparse camera steps -> translation_valid = false
    std::vector<camera_imu_calibration::Observation> sparse_obs;
    for (size_t i = 0; i < obs_list.size(); i += 3) sparse_obs.push_back(obs_list[i]);
    auto rot_only = camera_imu_calibration::estimate_cam_imu_extrinsic(sparse_obs, st.K0, st.D0, imu_times, imu_vals);
    REQUIRE(!rot_only.translation_valid);
    REQUIRE(rot_only.translation_rows == 0);
    REQUIRE((rot_only.T_b_c1.topRightCorner<3, 1>().norm() < 1e-12));
}

void test_zero_cross_dependencies_between_functional_packages() {
    std::vector<std::string> pkgs = {
        "calibration_targets",
        "camera_imu_calibration",
        "handeye_calibration",
        "vision_calibration",
        "preview_3d",
    };
    std::vector<std::string> upper = {"umi_slam", "slam_app", "calibration_workbench", "main_window"};
    std::regex inc_re(R"xxx(^\s*#\s*include\s*[<"]([^">]+)[">])xxx");

    for (const auto& pkg : pkgs) {
        std::set<std::string> forbidden(pkgs.begin(), pkgs.end());
        forbidden.erase(pkg);
        forbidden.insert(upper.begin(), upper.end());
        forbidden.insert("Qt");
        forbidden.insert("GLFW");
        forbidden.insert("nfd");
        if (pkg != "preview_3d") forbidden.insert("imgui");

        for (const auto& dir_base : {repo_root() / "include" / pkg, repo_root() / "src" / pkg}) {
            REQUIRE(std::filesystem::is_directory(dir_base));
            for (const auto& entry : std::filesystem::directory_iterator(dir_base)) {
                if (!entry.is_regular_file()) continue;
                std::string text = slam_common::read_text_utf8(entry.path());
                std::istringstream ss(text);
                std::string line;
                while (std::getline(ss, line)) {
                    std::smatch m;
                    if (std::regex_search(line, m, inc_re)) {
                        std::string hdr = m[1].str();
                        for (const auto& f : forbidden) {
                            if (hdr.rfind(f + "/", 0) == 0 || hdr.find(f) != std::string::npos) {
                                throw std::runtime_error(pkg + "/" + entry.path().filename().string() + " must not include " + hdr);
                            }
                        }
                    }
                }
            }
        }
    }

    // slam_common must remain pure leaf (no functional pkg, upper, cv2/opencv, Qt, imgui, or GLFW)
    std::vector<std::string> common_forbidden = {"opencv", "Qt", "QWidget", "imgui", "GLFW", "nfd",
                                               "calibration_targets", "camera_imu_calibration",
                                               "handeye_calibration", "vision_calibration",
                                               "preview_3d", "umi_slam"};
    for (const auto& dir_base : {repo_root() / "include/slam_common", repo_root() / "src/slam_common"}) {
        for (const auto& entry : std::filesystem::directory_iterator(dir_base)) {
            if (!entry.is_regular_file()) continue;
            std::string text = slam_common::read_text_utf8(entry.path());
            std::istringstream ss(text);
            std::string line;
            while (std::getline(ss, line)) {
                std::smatch m;
                if (std::regex_search(line, m, inc_re)) {
                    std::string hdr = m[1].str();
                    for (const auto& f : common_forbidden) {
                        if (hdr.find(f) != std::string::npos) {
                            throw std::runtime_error("slam_common must not include " + hdr);
                        }
                    }
                }
            }
        }
    }
}

void test_ms901m_decoder_parity() {
    TempDir td("ms901m");
    auto p1 = make_ms901m_pkt(3, pack_i16x6(100, -200, 8192, 50, -60, 70));
    auto p2 = make_ms901m_pkt(3, pack_i16x6(120, -180, 8100, -30, 40, 90));
    auto p_other = make_ms901m_pkt(1, {1, 2, 3, 4});
    std::vector<uint8_t> stream = p_other;
    stream.insert(stream.end(), p1.begin(), p1.end());
    stream.insert(stream.end(), p2.begin(), p2.end());

    std::vector<uint8_t> frag0(stream.begin(), stream.begin() + 10);
    std::vector<uint8_t> frag1(stream.begin() + 10, stream.end());
    auto b0 = pack_uart_block(1000000000ULL, frag0);
    auto b1 = pack_uart_block(1008000000ULL, frag1);
    b0.insert(b0.end(), b1.begin(), b1.end());
    auto bin_file = td.path / "ms901m.bin";
    write_bytes(bin_file, b0);

    auto imu = slam_common::decode_ms901m_bin(bin_file, 250.0, 4.0, 2000.0);
    REQUIRE(imu.timestamps_ns.size() == 2);
    REQUIRE(imu.timestamps_ns[0] == 1004000000LL);
    REQUIRE(imu.timestamps_ns[1] == 1008000000LL);

    auto bad = p1;
    bad.back() ^= 0xFFu;
    std::vector<uint8_t> bad_stream = bad;
    bad_stream.insert(bad_stream.end(), p1.begin(), p1.end());
    bad_stream.insert(bad_stream.end(), p2.begin(), p2.end());
    auto bad_file = td.path / "corrupt.bin";
    write_bytes(bad_file, pack_uart_block(1000000000ULL, bad_stream));
    require_throws([&]() { slam_common::decode_ms901m_bin(bad_file); });
}

void test_sha256sums_matches_existing_repo_files() {
    auto report = slam_common::verify_repo_checksums(repo_root());
    REQUIRE(report.checked_count >= 20);
    if (!report.mismatches.empty()) {
        std::ostringstream os;
        for (const auto& m : report.mismatches) {
            os << m.rel_path << " exp=" << m.expected << " act=" << m.actual << "; ";
        }
        throw std::runtime_error("SHA256SUMS out of sync: " + os.str());
    }
}

void test_dynamic_board_generation_and_roundtrip_detection() {
    TempDir td("boards");
    struct Case { std::string t; int r, c; double sz, ratio; };
    std::vector<Case> cases = {
        {"aprilgrid", 4, 4, 25.0, 0.3},
        {"chessboard", 5, 7, 18.0, 0.0},
        {"circles", 4, 6, 22.0, 0.45},
        {"acircles", 7, 4, 24.0, 0.42},
    };
    for (const auto& cs : cases) {
        auto rb = calibration_targets::render_calibration_board(cs.t, cs.r, cs.c, cs.sz, cs.ratio, "A4", 120);
        REQUIRE(rb.meta.pattern_w_mm > 10.0);
        auto det = calibration_targets::detect_target(rb.image, cs.t, cs.r, cs.c, cs.sz / 1000.0, cs.ratio);
        REQUIRE(det.found);
        size_t expected = (cs.t == "aprilgrid") ? static_cast<size_t>(cs.r * cs.c * 4) : static_cast<size_t>(cs.r * cs.c);
        REQUIRE(det.img_pts.size() == expected);

        auto [pdf_p, _m1] = calibration_targets::save_calibration_board(td.path / (cs.t + ".pdf"), cs.t, cs.r, cs.c, cs.sz, cs.ratio, "A4");
        auto [svg_p, _m2] = calibration_targets::save_calibration_board(td.path / (cs.t + ".svg"), cs.t, cs.r, cs.c, cs.sz, cs.ratio, "A4");
        auto [png_p, _m3] = calibration_targets::save_calibration_board(td.path / (cs.t + ".png"), cs.t, cs.r, cs.c, cs.sz, cs.ratio, "A4", 300);

        std::string pdf_txt = slam_common::read_text_utf8(pdf_p);
        REQUIRE(pdf_txt.rfind("%PDF-1.4", 0) == 0);
        REQUIRE(pdf_txt.find("/MediaBox") != std::string::npos);
        std::string svg_txt = slam_common::read_text_utf8(svg_p);
        REQUIRE(svg_txt.find("mm\"") != std::string::npos);

        std::ifstream pf(png_p, std::ios::binary);
        std::string png_bytes((std::istreambuf_iterator<char>(pf)), std::istreambuf_iterator<char>());
        size_t phys = png_bytes.find("pHYs");
        REQUIRE(phys != std::string::npos && phys + 13 <= png_bytes.size());
        auto be32 = [&](size_t off) {
            return (static_cast<uint32_t>(static_cast<uint8_t>(png_bytes[off])) << 24) |
                   (static_cast<uint32_t>(static_cast<uint8_t>(png_bytes[off + 1])) << 16) |
                   (static_cast<uint32_t>(static_cast<uint8_t>(png_bytes[off + 2])) << 8) |
                   static_cast<uint32_t>(static_cast<uint8_t>(png_bytes[off + 3]));
        };
        uint32_t ppm_x = be32(phys + 4);
        uint32_t ppm_y = be32(phys + 8);
        uint8_t unit = static_cast<uint8_t>(png_bytes[phys + 12]);
        REQUIRE(unit == 1);
        REQUIRE(ppm_x == ppm_y);
        REQUIRE_NEAR(static_cast<double>(ppm_x) * 0.0254, 300.0, 1.5);
        cv::Mat dec = cv::imread(png_p.string(), cv::IMREAD_GRAYSCALE);
        REQUIRE(!dec.empty());
    }
}

void test_multicam_mono_and_imu_only_calibration_modes() {
    TempDir td("multicam");
    // 1. Pure IMU standalone calibration
    double hz = 200.0;
    int n_samples = 600;
    std::vector<int64_t> t_ns(n_samples);
    std::vector<Eigen::Matrix<double, 6, 1>> imu_vals(n_samples);
    std::mt19937_64 rng(7);
    std::normal_distribution<double> dg(0.0, 0.004), da(0.0, 0.03);
    for (int k = 0; k < n_samples; ++k) {
        t_ns[k] = static_cast<int64_t>(std::llround(1e9 + static_cast<double>(k) * 1e9 / hz));
        imu_vals[k] << dg(rng) + 0.01, dg(rng) - 0.005, dg(rng) + 0.002,
                       da(rng), da(rng), da(rng) + 9.81;
    }
    auto imu_res = camera_imu_calibration::calibrate_imu_standalone(t_ns, imu_vals);
    REQUIRE_NEAR(imu_res.measured_hz, 200.0, 0.5);
    REQUIRE_NEAR(imu_res.gravity_norm_mps2, 9.81, 0.1);

    auto imu_dir = td.path / "imu_only_data";
    std::filesystem::create_directories(imu_dir);
    {
        std::ostringstream os;
        os << "#timestamp_ns,gx,gy,gz,ax,ay,az\n" << std::fixed << std::setprecision(8);
        for (int k = 0; k < n_samples; ++k) {
            os << t_ns[k];
            for (int c = 0; c < 6; ++c) os << ',' << imu_vals[k](c);
            os << '\n';
        }
        write_file(imu_dir / "imu.csv", os.str());
    }
    camera_imu_calibration::SuiteCalibrationParams sp_imu;
    sp_imu.source_dir = imu_dir;
    sp_imu.output_dir = td.path / "out_imu_only";
    sp_imu.camera_mode = "none";
    sp_imu.enable_imu = true;
    auto suite_imu = camera_imu_calibration::calibrate_sensor_suite(sp_imu);
    REQUIRE(suite_imu.summary.get("sensor_mode").as_string() == "imu_only");
    auto prof_imu = slam_common::load_device(suite_imu.json_path);
    REQUIRE(prof_imu.get("num_cameras").as_int() == 0);

    // 2. Monocular & 3-camera multi-rig calibration
    cv::Mat K(cv::Matx33d(450.0, 0.0, 320.0, 0.0, 450.0, 240.0, 0.0, 0.0, 1.0));
    cv::Mat D(cv::Vec<double, 5>(-0.15, 0.03, 0.0, 0.0, 0.0));
    Eigen::Matrix4d T01 = Eigen::Matrix4d::Identity(); T01(0, 3) = 0.08;
    Eigen::Matrix4d T02 = Eigen::Matrix4d::Identity(); T02(1, 3) = 0.12;
    std::vector<Eigen::Matrix4d> T_c_c0 = {Eigen::Matrix4d::Identity(), T01.inverse(), T02.inverse()};
    std::vector<cv::Point3f> obj;
    for (int r = 0; r < 5; ++r) {
        for (int c = 0; c < 6; ++c) obj.emplace_back(static_cast<float>(c * 0.04), static_cast<float>(r * 0.04), 0.0f);
    }

    std::vector<camera_imu_calibration::Observation> obs_multi;
    for (int idx = 0; idx < 20; ++idx) {
        double t = 2.0 * static_cast<double>(idx) / 19.0;
        cv::Mat R_cv;
        cv::Rodrigues(cv::Vec3d(0.25 * std::sin(1.7 * t), 0.30 * std::cos(1.2 * t), 0.2 * std::sin(2.0 * t)), R_cv);
        Eigen::Matrix4d T_c0_w = Eigen::Matrix4d::Identity();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) T_c0_w(r, c) = R_cv.at<double>(r, c);
        }
        T_c0_w.topRightCorner<3, 1>() = Eigen::Vector3d(0.05 * std::sin(t), 0.04 * std::cos(t), 0.65 + 0.06 * std::sin(0.8 * t));

        camera_imu_calibration::Observation obs;
        obs.ts_ns = static_cast<int64_t>(std::llround((1.0 + t) * 1e9));
        std::vector<int> active = {1};
        if (idx < 14) active.push_back(0);
        if (idx >= 6) active.push_back(2);
        for (int ci : active) {
            Eigen::Matrix4d Tiw = T_c_c0[ci] * T_c0_w;
            cv::Mat Ri(3, 3, CV_64F), rv, tv(3, 1, CV_64F);
            for (int r = 0; r < 3; ++r) {
                tv.at<double>(r) = Tiw(r, 3);
                for (int c = 0; c < 3; ++c) Ri.at<double>(r, c) = Tiw(r, c);
            }
            cv::Rodrigues(Ri, rv);
            std::vector<cv::Point2f> im;
            cv::projectPoints(obj, rv, tv, K, D, im);
            obs.obj_pts[ci] = obj;
            obs.img_pts[ci] = std::move(im);
        }
        obs_multi.push_back(std::move(obs));
    }

    auto mono_res = camera_imu_calibration::calibrate_monocular_from_observations(obs_multi, cv::Size(640, 480), 1);
    REQUIRE(mono_res.mono_rms < 0.2);

    auto mc_res = camera_imu_calibration::calibrate_multicam_from_observations(
        obs_multi, {cv::Size(640, 480), cv::Size(640, 480), cv::Size(640, 480)});
    REQUIRE_NEAR(mc_res.cameras[1].baseline_to_c0_m, 0.08, 0.003);
    REQUIRE_NEAR(mc_res.cameras[2].baseline_to_c0_m, 0.12, 0.004);
    REQUIRE_NEAR(mc_res.cameras[2].T_c0_ci(1, 3), 0.12, 0.004);

    camera_imu_calibration::WriteOrbYamlParams wp;
    wp.output_dir = td.path / "out_multi";
    wp.cam0_K = mc_res.cameras[0].K; wp.cam0_D = mc_res.cameras[0].D;
    wp.has_cam1 = true; wp.cam1_K = mc_res.cameras[1].K; wp.cam1_D = mc_res.cameras[1].D;
    wp.width = 640; wp.height = 480;
    wp.has_T_c1_c2 = true; wp.T_c1_c2 = mc_res.cameras[1].T_c0_ci;
    wp.sensor_mode = "multi";
    wp.extra_cameras = mc_res.cameras;
    auto [ym, jm] = camera_imu_calibration::write_orbslam_calibration(wp);
    REQUIRE(std::filesystem::is_regular_file(td.path / "out_multi/multicam_calibration.json"));
    REQUIRE(slam_common::load_device(jm).get("sensor_mode").as_string() == "multi");
    REQUIRE(slam_common::read_text_utf8(ym).find("Rig.T_c1_c3") != std::string::npos);

    // 3. Warped circle-grid mono suite + unsupported mode rejection + missing IMU status
    auto rb = calibration_targets::render_calibration_board("circles", 4, 5, 20.0, 0.45, "Auto", 96);
    int bh = rb.image.rows, bw = rb.image.cols;
    std::vector<cv::Point2f> src_pts = {{0, 0}, {static_cast<float>(bw), 0}, {static_cast<float>(bw), static_cast<float>(bh)}, {0, static_cast<float>(bh)}};
    auto mono_dir = td.path / "mono_circles";
    std::filesystem::create_directories(mono_dir);
    for (int k = 0; k < 10; ++k) {
        double dx = 40.0 + k * 7.0;
        double dy = 35.0 + (k % 3) * 18.0;
        std::vector<cv::Point2f> dst_pts = {
            {static_cast<float>(120 + (k % 4) * 12), static_cast<float>(90 + dy * 0.3)},
            {static_cast<float>(520 - (k % 3) * 10), static_cast<float>(95 + dx * 0.2)},
            {static_cast<float>(535 - dy * 0.2), static_cast<float>(395 - (k % 4) * 10)},
            {static_cast<float>(110 + dx * 0.2), static_cast<float>(385 - (k % 3) * 8)},
        };
        cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
        cv::Mat warped;
        cv::warpPerspective(rb.image, warped, M, cv::Size(640, 480), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
        cv::imwrite((mono_dir / (std::to_string(1000000000LL + k * 50000000LL) + ".png")).string(), warped);
    }

    camera_imu_calibration::SuiteCalibrationParams sp_mono;
    sp_mono.source_dir = mono_dir;
    sp_mono.output_dir = td.path / "out_mono_suite";
    sp_mono.camera_mode = "mono";
    sp_mono.enable_imu = false;
    sp_mono.target_type = "circles";
    sp_mono.rows = 4; sp_mono.cols = 5; sp_mono.tag_size_m = 0.02; sp_mono.tag_spacing = 0.45; sp_mono.stride = 1;
    auto mono_suite = camera_imu_calibration::calibrate_sensor_suite(sp_mono);
    REQUIRE(mono_suite.summary.get("sensor_mode").as_string() == "mono");
    REQUIRE(slam_common::load_device(mono_suite.json_path).get("num_cameras").as_int() == 1);

    for (const auto& bad_j : {mono_suite.json_path, jm, suite_imu.json_path}) {
        require_throws([&]() { slam_common::validate_slam_sensor_mode(slam_common::load_device(bad_j)); }, "stereo_imu");
    }
    auto src_euroc = td.path / "src_e";
    std::filesystem::create_directories(src_euroc / "mav0/cam0/data");
    std::filesystem::create_directories(src_euroc / "mav0/cam1/data");
    std::filesystem::create_directories(src_euroc / "mav0/imu0");
    write_file(src_euroc / "mav0/cam0/data.csv", "#\n");
    write_file(src_euroc / "mav0/cam1/data.csv", "#\n");
    write_file(src_euroc / "mav0/imu0/data.csv", "#\n");
    require_throws([&]() { umi_slam::Job(src_euroc, td.path / "jobs_rej", mono_suite.yaml_path, mono_suite.json_path, 5, 0, 0.0, false); }, "stereo_imu");

    sp_mono.output_dir = td.path / "out_mono_imu_missing";
    sp_mono.enable_imu = true;
    auto no_imu = camera_imu_calibration::calibrate_sensor_suite(sp_mono);
    REQUIRE(no_imu.summary.get("imu_extrinsic_status").as_string() == "no_imu_data");
    auto missing_prof = slam_common::load_device(no_imu.json_path);
    REQUIRE(missing_prof.get("imu_extrinsic_status").as_string() == "no_imu_data");
    REQUIRE(missing_prof.get("imu_extrinsic_warning").as_string().find("占位单位阵") != std::string::npos);
    REQUIRE(slam_common::read_text_utf8(no_imu.yaml_path).find("# WARNING:") != std::string::npos);
}

void test_visionmaster_npoint_alignment_and_3d_handeye() {
    auto plan = handeye_calibration::generate_n_point_plan(9, 3, 4, 0.0, 0.0, 5.0, 5.0);
    REQUIRE_NEAR(plan.trans_points[4].x, 0.0, 1e-9);
    REQUIRE_NEAR(plan.trans_points[4].y, 0.0, 1e-9);
    REQUIRE_NEAR(plan.trans_points[0].x, -5.0, 1e-9);
    REQUIRE_NEAR(plan.trans_points[0].y, -5.0, 1e-9);
    REQUIRE_NEAR(plan.trans_points[3].x, 5.0, 1e-9);
    REQUIRE_NEAR(plan.trans_points[3].y, 0.0, 1e-9);

    Eigen::Matrix3d M_w2p;
    M_w2p << 18.0, 2.0, 500.0,
             -1.5, -19.0, 400.0,
             0.0, 0.0, 1.0;
    std::vector<Eigen::Vector2d> world_pts, pixel_pts;
    for (const auto& p : plan.trans_points) {
        Eigen::Vector2d w(p.x, p.y);
        world_pts.push_back(w);
        pixel_pts.push_back(M_w2p.topLeftCorner<2, 2>() * w + M_w2p.topRightCorner<2, 1>());
    }

    double uc_true = 515.0, vc_true = 388.0;
    Eigen::Vector2d v0(35.0, -20.0);
    std::vector<Eigen::Vector2d> rot_px;
    for (double ang : plan.rot_angles_deg) {
        double rad = ang * kPi / 180.0;
        Eigen::Matrix2d R2;
        R2 << std::cos(rad), -std::sin(rad), std::sin(rad), std::cos(rad);
        rot_px.push_back(Eigen::Vector2d(uc_true, vc_true) + R2 * v0);
    }

    for (const char* dof : {"similarity", "affine", "homography"}) {
        auto res = handeye_calibration::calibrate_n_point_2d(pixel_pts, world_pts, rot_px, plan.rot_angles_deg, "static_up", dof);
        if (std::string(dof) != "similarity") {
            REQUIRE(res.rms_mm < 1e-6);
            REQUIRE_NEAR(res.rotation_center_px->x(), uc_true, 1e-4);
            REQUIRE_NEAR(res.rotation_center_px->y(), vc_true, 1e-4);
            auto pt4 = handeye_calibration::transform_point_2d(res.matrix, pixel_pts[4].x(), pixel_pts[4].y());
            REQUIRE_NEAR(pt4.x(), 0.0, 1e-4);
            REQUIRE_NEAR(pt4.y(), 0.0, 1e-4);
        }
    }

    auto bad_px = pixel_pts;
    bad_px[2] += Eigen::Vector2d(150.0, -90.0);
    auto res_tukey = handeye_calibration::calibrate_n_point_2d(bad_px, world_pts, {}, {}, "static_up", "affine", "tukey");
    REQUIRE(res_tukey.rms_mm < 1e-4);
    REQUIRE(res_tukey.inlier_count == 8);
    auto res_ransac = handeye_calibration::calibrate_n_point_2d(bad_px, world_pts, {}, {}, "static_up", "homography", "ransac", 1.345, 0.1);
    REQUIRE(res_ransac.rms_mm < 1e-4);

    Eigen::Vector2d rc_w(2.0, -1.0);
    std::vector<Eigen::Vector2d> cur_w = {{10.0, 2.0}, {20.0, 2.0}, {15.0, 8.0}};
    double dth_true = 6.5;
    Eigen::Vector2d shift_true(-3.2, 4.7);
    double rad = dth_true * kPi / 180.0;
    Eigen::Matrix2d R_true;
    R_true << std::cos(rad), -std::sin(rad), std::sin(rad), std::cos(rad);
    std::vector<Eigen::Vector2d> tgt_w;
    for (const auto& p : cur_w) tgt_w.push_back(rc_w + R_true * (p - rc_w) + shift_true);
    auto al = handeye_calibration::compute_alignment_offset_2d(tgt_w, cur_w, rc_w);
    REQUIRE_NEAR(al.dx_mm, shift_true.x(), 1e-4);
    REQUIRE_NEAR(al.dy_mm, shift_true.y(), 1e-4);
    REQUIRE_NEAR(al.dtheta_deg, dth_true, 1e-4);

    Eigen::Matrix4d T_bt = handeye_calibration::pose_to_matrix(std::vector<double>{0.6, -0.1, 0.4, 0.1, -0.15, 0.2});
    Eigen::Matrix4d X_eih_true = handeye_calibration::pose_to_matrix(std::vector<double>{0.05, -0.03, 0.09, 0.3, -0.2, 1.1});
    std::vector<std::array<double, 3>> axes = {
        {0.4, 0.1, -0.2}, {-0.3, 0.5, 0.3}, {0.2, -0.4, 0.6},
        {0.6, 0.3, 0.1}, {-0.5, -0.2, 0.5}, {0.1, 0.6, -0.4},
    };
    std::vector<Eigen::Matrix4d> T_bg_list, T_ct_eih;
    for (size_t idx = 0; idx < axes.size(); ++idx) {
        std::vector<double> pv = {
            0.4 + 0.08 * std::sin(static_cast<double>(idx)),
            -0.15 + 0.07 * std::cos(static_cast<double>(idx)),
            0.75 + 0.05 * std::sin(2.0 * static_cast<double>(idx)),
            axes[idx][0], axes[idx][1], axes[idx][2]};
        Eigen::Matrix4d T_bg = handeye_calibration::pose_to_matrix(pv);
        T_bg_list.push_back(T_bg);
        T_ct_eih.push_back((T_bg * X_eih_true).inverse() * T_bt);
    }

    for (const char* method : {"tsai", "park", "horaud", "andreff", "daniilidis"}) {
        auto he = handeye_calibration::calibrate_handeye_3d(T_bg_list, T_ct_eih, "eye_in_hand", method);
        REQUIRE((he.T_result.topRightCorner<3, 1>() - X_eih_true.topRightCorner<3, 1>()).norm() < 1e-4);
        REQUIRE((he.T_result.topLeftCorner<3, 3>() * X_eih_true.topLeftCorner<3, 3>().transpose() - Eigen::Matrix3d::Identity()).norm() < 1e-4);
    }

    Eigen::Matrix4d T_bc_true = handeye_calibration::pose_to_matrix(std::vector<double>{1.1, 0.2, 0.9, 0.25, -0.4, 0.8});
    Eigen::Matrix4d T_gt_true = handeye_calibration::pose_to_matrix(std::vector<double>{0.02, 0.04, 0.06, -0.1, 0.2, 0.3});
    std::vector<Eigen::Matrix4d> T_ct_eth;
    for (const auto& T_bg : T_bg_list) T_ct_eth.push_back(T_bc_true.inverse() * T_bg * T_gt_true);
    auto eth = handeye_calibration::calibrate_handeye_3d(T_bg_list, T_ct_eth, "eye_to_hand", "park");
    REQUIRE((eth.T_result.topRightCorner<3, 1>() - T_bc_true.topRightCorner<3, 1>()).norm() < 1e-4);
}

void test_visionmaster_planar_distortion_and_dual_cam_mapping() {
    TempDir td("vision");
    std::vector<Eigen::Vector2d> obj_mm, img_pts;
    for (int x = 0; x < 6; ++x) {
        for (int y = 0; y < 5; ++y) {
            Eigen::Vector2d o(x * 10.0, y * 10.0);
            obj_mm.push_back(o);
            img_pts.push_back(o * 15.0 + Eigen::Vector2d(120.0, 80.0));
        }
    }
    auto cal = vision_calibration::calibrate_planar_board_2d(img_pts, obj_mm, 0.0, 0.0, 0.0, true, "homography");
    REQUIRE_NEAR(cal.pixel_size_mm, 1.0 / 15.0, 1e-5);
    REQUIRE(cal.rms_mm < 1e-6);
    auto xy = vision_calibration::convert_pixel_to_world_2d(cal, 120.0 + 30.0 * 15.0, 80.0 + 20.0 * 15.0);
    REQUIRE_NEAR(xy.x(), 30.0, 1e-4);
    REQUIRE_NEAR(xy.y(), 20.0, 1e-4);
    REQUIRE_NEAR(vision_calibration::convert_pixel_length_to_mm(cal, 150.0, 200.0, 150.0), 10.0, 1e-4);

    auto iw_path = vision_calibration::save_iwcal_file(td.path / "test.iwcal", cal.to_json());
    auto loaded = vision_calibration::PlanarCalib2D::from_json(vision_calibration::load_iwcal_file(iw_path));
    REQUIRE_NEAR(loaded.pixel_size_mm, cal.pixel_size_mm, 1e-9);

    cv::Mat K_true(cv::Matx33d(500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0));
    cv::Mat D_true(cv::Vec<double, 5>(-0.22, 0.05, 0.0, 0.0, 0.0));
    std::vector<cv::Point3f> obj3;
    for (int x = 0; x < 6; ++x) {
        for (int y = 0; y < 5; ++y) obj3.emplace_back(static_cast<float>(x * 20.0), static_cast<float>(y * 20.0), 0.0f);
    }
    cv::Vec3d rv(0.22, -0.18, 0.05);
    cv::Vec3d tv(-50.0, -40.0, 320.0);
    std::vector<cv::Point2f> dist_pts;
    cv::projectPoints(obj3, rv, tv, K_true, D_true, dist_pts);
    auto dmodel = vision_calibration::calibrate_image_distortion(
        {obj3}, {dist_pts}, cv::Size(640, 480), "perspective", Eigen::Vector2d(320.0, 240.0));
    cv::Mat test_img(480, 640, CV_8UC1, cv::Scalar(255));
    for (const auto& p : dist_pts) {
        cv::circle(test_img, cv::Point(static_cast<int>(std::llround(p.x)), static_cast<int>(std::llround(p.y))), 4, cv::Scalar(0), -1);
    }
    cv::Mat out_img = vision_calibration::undistort_image_with_model(test_img, dmodel);
    REQUIRE(out_img.size() == test_img.size());

    std::vector<Eigen::Vector2d> cam2_pts;
    for (const auto& p : img_pts) cam2_pts.push_back(p * 1.1 + Eigen::Vector2d(15.0, -8.0));
    auto dmap = vision_calibration::calibrate_dual_camera_mapping_2d(img_pts, cam2_pts, "affine");
    REQUIRE(dmap.rms_px < 1e-6);
    REQUIRE_NEAR(dmap.scale_x, 1.1, 1e-4);
}

int helper_sleep() {
    std::this_thread::sleep_for(std::chrono::seconds(30));
    return 0;
}

int helper_spawn_tree(const std::string& self_exe, const std::string& pids_file) {
#ifdef _WIN32
    std::string cmd = "\"" + self_exe + "\" --helper-sleep";
    STARTUPINFOA si{};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    if (!CreateProcessA(nullptr, cmd.data(), nullptr, nullptr, FALSE,
                        CREATE_NEW_PROCESS_GROUP | CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi)) {
        return 1;
    }
    slam_common::JsonValue pj = slam_common::JsonValue::object();
    pj["child"] = static_cast<int64_t>(GetCurrentProcessId());
    pj["grandchild"] = static_cast<int64_t>(pi.dwProcessId);
    slam_common::save_json(std::filesystem::u8path(pids_file), pj);
    std::this_thread::sleep_for(std::chrono::seconds(30));
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return 0;
#else
    pid_t gc = fork();
    if (gc == 0) {
        std::this_thread::sleep_for(std::chrono::seconds(30));
        _exit(0);
    }
    slam_common::JsonValue pj = slam_common::JsonValue::object();
    pj["child"] = static_cast<int64_t>(getpid());
    pj["grandchild"] = static_cast<int64_t>(gc);
    slam_common::save_json(std::filesystem::u8path(pids_file), pj);
    std::this_thread::sleep_for(std::chrono::seconds(30));
    return 0;
#endif
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--helper-sleep") {
        return helper_sleep();
    }
    if (argc > 2 && std::string(argv[1]) == "--helper-spawn-tree") {
        return helper_spawn_tree(argv[0], argv[2]);
    }

    int passed = 0;
    auto run_test = [&](const char* name, auto&& fn) {
        std::cout << "[RUN ] " << name << std::endl;
        fn();
        std::cout << "[ OK ] " << name << std::endl;
        ++passed;
    };

    try {
        run_test("test_native_chinese_paths_and_command", test_native_chinese_paths_and_command);
        run_test("test_refuse_nested_existing_and_invalid_params", test_refuse_nested_existing_and_invalid_params);
        run_test("test_unknown_protocol_units_and_bad_transform", test_unknown_protocol_units_and_bad_transform);
        run_test("test_euroc_does_not_accept_legacy_raw", test_euroc_does_not_accept_legacy_raw);
        run_test("test_target_transform_rotates_translation_and_preserves_gaps", test_target_transform_rotates_translation_and_preserves_gaps);
        run_test("test_rotation_roundtrip_at_180_degrees", test_rotation_roundtrip_at_180_degrees);
        run_test("test_cancel_before_start_never_launches_process", test_cancel_before_start_never_launches_process);
        run_test("test_job_no_false_success_and_config_snapshot", test_job_no_false_success_and_config_snapshot);
        run_test("test_native_backend_resolution_and_cancel", test_native_backend_resolution_and_cancel);
        run_test("test_native_execute_kills_grandchild_tree_and_reconciles_job_state", [&]() {
            test_native_execute_kills_grandchild_tree_and_reconciles_job_state(argv[0]);
        });
        run_test("test_load_preview_data_full_job_and_fallbacks", test_load_preview_data_full_job_and_fallbacks);
        run_test("test_convert_kalibr_reference_matches_orb_yaml", test_convert_kalibr_reference_matches_orb_yaml);
        run_test("test_synthetic_stereo_and_imu_extrinsic_calibration", test_synthetic_stereo_and_imu_extrinsic_calibration);
        run_test("test_zero_cross_dependencies_between_functional_packages", test_zero_cross_dependencies_between_functional_packages);
        run_test("test_ms901m_decoder_parity", test_ms901m_decoder_parity);
        run_test("test_sha256sums_matches_existing_repo_files", test_sha256sums_matches_existing_repo_files);
        run_test("test_dynamic_board_generation_and_roundtrip_detection", test_dynamic_board_generation_and_roundtrip_detection);
        run_test("test_multicam_mono_and_imu_only_calibration_modes", test_multicam_mono_and_imu_only_calibration_modes);
        run_test("test_visionmaster_npoint_alignment_and_3d_handeye", test_visionmaster_npoint_alignment_and_3d_handeye);
        run_test("test_visionmaster_planar_distortion_and_dual_cam_mapping", test_visionmaster_planar_distortion_and_dual_cam_mapping);
    } catch (const std::exception& exc) {
        std::cerr << "[FAIL] " << exc.what() << std::endl;
        return 1;
    }
    std::cout << "All " << passed << " station tests passed." << std::endl;
    return 0;
}
