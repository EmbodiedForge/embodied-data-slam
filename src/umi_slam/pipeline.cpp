#include "umi_slam/pipeline.hpp"

#include "umi_slam/build_backend.hpp"
#include "process_util.hpp"
#include "slam_common/ms901m.hpp"
#include "slam_common/sha256.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <tlhelp32.h>
#else
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace umi_slam {
namespace {

using slam_common::JsonValue;

std::atomic<bool> g_cancel_requested{false};

void signal_cancel_handler(int) {
    g_cancel_requested.store(true);
}

#ifdef _WIN32
BOOL WINAPI console_ctrl_handler(DWORD ctrl_type) {
    if (ctrl_type == CTRL_C_EVENT || ctrl_type == CTRL_BREAK_EVENT || ctrl_type == CTRL_CLOSE_EVENT) {
        g_cancel_requested.store(true);
        return TRUE;
    }
    return FALSE;
}

std::vector<DWORD> windows_descendant_pids(DWORD root_pid) {
    std::vector<DWORD> result;
    if (root_pid == 0) return result;
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE) return result;
    std::map<DWORD, std::vector<DWORD>> children;
    PROCESSENTRY32W pe{};
    pe.dwSize = sizeof(pe);
    if (Process32FirstW(snap, &pe)) {
        do {
            children[pe.th32ParentProcessID].push_back(pe.th32ProcessID);
        } while (Process32NextW(snap, &pe));
    }
    CloseHandle(snap);

    std::set<DWORD> seen;
    std::vector<DWORD> queue = {root_pid};
    while (!queue.empty()) {
        DWORD cur = queue.back();
        queue.pop_back();
        auto it = children.find(cur);
        if (it == children.end()) continue;
        for (DWORD ch : it->second) {
            if (ch != 0 && ch != root_pid && !seen.count(ch)) {
                seen.insert(ch);
                result.push_back(ch);
                queue.push_back(ch);
            }
        }
    }
    return result;
}

void windows_kill_pids(const std::vector<DWORD>& pids) {
    for (DWORD p : pids) {
        if (p == 0) continue;
        HANDLE h = OpenProcess(PROCESS_TERMINATE | SYNCHRONIZE, FALSE, p);
        if (!h) h = OpenProcess(PROCESS_TERMINATE, FALSE, p);
        if (h) {
            TerminateProcess(h, 1);
            WaitForSingleObject(h, 2000);
            CloseHandle(h);
        }
    }
}
#endif

std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

bool is_subpath(const std::filesystem::path& child, const std::filesystem::path& parent) {
    std::error_code ec;
    auto c = std::filesystem::weakly_canonical(child, ec);
    auto p = std::filesystem::weakly_canonical(parent, ec);
    auto it_c = c.begin(), it_p = p.begin();
    for (; it_p != p.end(); ++it_p, ++it_c) {
        if (it_c == c.end() || *it_c != *it_p) return false;
    }
    return true;
}

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    bool in_q = false;
    for (size_t i = 0; i < line.size(); ++i) {
        char c = line[i];
        if (in_q) {
            if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
                cur.push_back('"');
                ++i;
            } else if (c == '"') {
                in_q = false;
            } else {
                cur.push_back(c);
            }
        } else {
            if (c == '"') in_q = true;
            else if (c == ',') { out.push_back(cur); cur.clear(); }
            else if (c != '\r') cur.push_back(c);
        }
    }
    out.push_back(cur);
    return out;
}

double median_vec(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t mid = v.size() / 2;
    return (v.size() % 2 == 1) ? v[mid] : 0.5 * (v[mid - 1] + v[mid]);
}

/// Round non-negative decimal timestamp string (e.g. "1000000000.015625") to nearest integer without double loss.
int64_t round_decimal_timestamp_ns(const std::string& token) {
    std::string s = trim(token);
    if (s.empty() || s.front() == '-') throw std::invalid_argument("Invalid exported timestamp");
    if (s.front() == '+') s.erase(s.begin());
    size_t dot = s.find('.');
    std::string int_part = (dot == std::string::npos) ? s : s.substr(0, dot);
    std::string frac_part = (dot == std::string::npos) ? "" : s.substr(dot + 1);
    if (int_part.empty()) throw std::invalid_argument("Invalid exported timestamp");
    for (unsigned char c : int_part) {
        if (!std::isdigit(c)) throw std::invalid_argument("Invalid exported timestamp");
    }
    for (unsigned char c : frac_part) {
        if (!std::isdigit(c)) throw std::invalid_argument("Invalid exported timestamp");
    }
    int64_t base = slam_common::integer_ns(int_part);
    if (!frac_part.empty()) {
        char d = frac_part[0];
        if (d > '5') {
            ++base;
        } else if (d == '5') {
            bool tail_nonzero = false;
            for (size_t i = 1; i < frac_part.size(); ++i) {
                if (frac_part[i] != '0') { tail_nonzero = true; break; }
            }
            if (tail_nonzero || (base % 2 != 0)) ++base;
        }
    }
    return base;
}

std::string replace_exact(
    const std::string& text,
    const std::string& old_str,
    const std::string& new_str,
    size_t expected_count,
    const std::string& label) {
    size_t count = 0;
    size_t pos = 0;
    while ((pos = text.find(old_str, pos)) != std::string::npos) {
        ++count;
        pos += old_str.size();
    }
    if (count != expected_count) {
        throw std::runtime_error(
            "ORB-SLAM3 patch mismatch (" + label + "): expected " +
            std::to_string(expected_count) + " match(es), got " + std::to_string(count));
    }
    std::string out;
    out.reserve(text.size() + count * (new_str.size() > old_str.size() ? new_str.size() - old_str.size() : 0));
    pos = 0;
    size_t found = 0;
    while ((found = text.find(old_str, pos)) != std::string::npos) {
        out.append(text, pos, found - pos);
        out.append(new_str);
        pos = found + old_str.size();
    }
    out.append(text, pos, text.size() - pos);
    return out;
}

std::string replace_all(std::string text, const std::string& old_str, const std::string& new_str) {
    size_t pos = 0;
    while ((pos = text.find(old_str, pos)) != std::string::npos) {
        text.replace(pos, old_str.size(), new_str);
        pos += new_str.size();
    }
    return text;
}

void write_text_utf8(const std::filesystem::path& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) throw std::runtime_error("Cannot write file: " + path.string());
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
    if (!f) throw std::runtime_error("Failed writing file: " + path.string());
}

template <typename Pred>
std::filesystem::path nearest_ancestor_matching(std::filesystem::path dir, Pred pred) {
    std::error_code ec;
    dir = std::filesystem::weakly_canonical(dir, ec);
    while (true) {
        if (pred(dir)) return dir;
        if (!dir.has_parent_path() || dir.parent_path() == dir) return {};
        dir = dir.parent_path();
    }
}

std::filesystem::path find_checksum_repo_root() {
    for (const auto& start : {std::filesystem::current_path(), current_executable_dir()}) {
        auto root = nearest_ancestor_matching(start, [](const std::filesystem::path& d) {
            return std::filesystem::is_regular_file(d / "SHA256SUMS");
        });
        if (!root.empty()) return root;
    }
    throw std::runtime_error("Cannot locate repository root containing SHA256SUMS");
}

std::filesystem::path find_backend_repo_root() {
    const std::filesystem::path orb_marker = std::filesystem::u8path("source/stereo_imu_trajectory/ORB_SLAM3");
    const std::filesystem::path runner_marker = std::filesystem::u8path("software/umi_slam/backend/runner.cc");
    for (const auto& start : {std::filesystem::current_path(), current_executable_dir()}) {
        auto root = nearest_ancestor_matching(start, [&](const std::filesystem::path& d) {
            return std::filesystem::is_directory(d / orb_marker) &&
                   std::filesystem::is_regular_file(d / runner_marker);
        });
        if (!root.empty()) return root;
    }
    throw std::runtime_error("Cannot locate repository root containing source/stereo_imu_trajectory/ORB_SLAM3");
}

void print_pipeline_usage() {
    std::cerr << "Usage: umi_slam_pipeline --input DIR --output DIR --settings YAML [--profile JSON] [--backend JSON] [--stride N] [--max-frames N] [--timeout S] [--prepare-only] [--prepared-input] [--viewer]\n"
              << "       umi_slam_pipeline --build-backend [--source-root DIR] [--runner-dir DIR] [--build-dir DIR] [--jobs N] [--cmake EXE]\n"
              << "       umi_slam_pipeline --verify-checksums | --update-checksums [--repo-root DIR]\n"
              << "       umi_slam_pipeline --help\n";
}

}  // namespace

SettingsInfo settings_info(const std::filesystem::path& path) {
    cv::FileStorage fs(path.string(), cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::invalid_argument("Cannot read ORB-SLAM3 settings");

    for (const char* key : {"Camera.width", "Camera.height", "Camera.fps", "IMU.Frequency",
                            "IMU.T_b_c1", "Stereo.T_c1_c2", "IMU.NoiseGyro", "IMU.NoiseAcc"}) {
        if (fs[key].empty()) {
            throw std::invalid_argument(std::string("Missing settings field: ") + key);
        }
    }

    int w = static_cast<int>(fs["Camera.width"].real());
    int h = static_cast<int>(fs["Camera.height"].real());
    for (const char* name : {"IMU.T_b_c1", "Stereo.T_c1_c2"}) {
        cv::Mat mat;
        fs[name] >> mat;
        if (mat.rows != 4 || mat.cols != 4) {
            throw std::invalid_argument(std::string("Invalid transform: ") + name);
        }
        cv::Mat d;
        mat.convertTo(d, CV_64F);
        Eigen::Matrix4d m;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) m(r, c) = d.at<double>(r, c);
        }
        if (!m.allFinite() || (m.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).cwiseAbs().maxCoeff() > 1e-4) {
            throw std::invalid_argument(std::string("Invalid transform: ") + name);
        }
        Eigen::Matrix3d R = m.topLeftCorner<3, 3>();
        if ((R.transpose() * R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1e-3 ||
            std::abs(R.determinant() - 1.0) > 1e-3) {
            throw std::invalid_argument(std::string("Invalid rotation: ") + name);
        }
    }

    SettingsInfo out;
    out.shape = cv::Size(w, h);
    out.fps = fs["Camera.fps"].real();
    out.imu_hz = fs["IMU.Frequency"].real();
    out.fast_init = (fs["IMU.fastInit"].real() != 0.0);
    return out;
}

JsonValue prepare(
    const std::filesystem::path& source,
    const std::filesystem::path& dest,
    const JsonValue& profile,
    const std::filesystem::path& settings,
    int stride,
    int limit) {
    SettingsInfo info = settings_info(settings);
    std::vector<std::string> warnings;
    if (info.fast_init) {
        warnings.push_back("IMU.fastInit enabled: experimental slow-motion preset; not an accuracy guarantee.");
    }
    if (stride < 1 || limit < 0) throw std::invalid_argument("stride must be >=1 and max-frames >=0");

    bool legacy = std::filesystem::is_regular_file(source / "stereo.mkv");
    std::vector<int64_t> times;
    std::vector<std::string> left_names, right_names;
    slam_common::ImuSeries imu;

    if (legacy) {
        if (!profile.get("adapter").is_string() || profile.get("adapter").as_string() != "umi_ms901m_uniform") {
            throw std::invalid_argument("This raw capture needs an explicit umi_ms901m_uniform device profile");
        }
        std::ifstream vf(source / "umi_video_timestamps.csv");
        if (!vf) throw std::runtime_error("Cannot open umi_video_timestamps.csv");
        std::string hdr;
        std::getline(vf, hdr);
        auto cols = split_csv(hdr);
        int ts_col = -1;
        for (size_t i = 0; i < cols.size(); ++i) {
            if (trim(cols[i]) == "driver_ts_ns") ts_col = static_cast<int>(i);
        }
        if (ts_col < 0) throw std::invalid_argument("Missing driver_ts_ns in umi_video_timestamps.csv");
        std::string line;
        while (std::getline(vf, line)) {
            if (trim(line).empty()) continue;
            auto row = split_csv(line);
            times.push_back(slam_common::integer_ns(row[static_cast<size_t>(ts_col)]));
        }
        double hz = profile.get("imu_hz").as_double();
        if (hz <= 0.0 || !std::isfinite(hz)) throw std::invalid_argument("Invalid IMU frequency");
        imu = slam_common::decode_ms901m_bin(
            source / "umi_imu_timestamps.bin",
            hz,
            profile.get("accel_range_g").as_double(),
            profile.get("gyro_range_dps").as_double());
        warnings.push_back("IMU sample times reconstructed at fixed frequency from UART arrival blocks; clock drift/dropped samples not independently verified.");
    } else {
        std::filesystem::path ts_txt = source / "timestamps.txt";
        if (std::filesystem::is_regular_file(ts_txt)) {
            std::ifstream tf(ts_txt);
            std::string line;
            while (std::getline(tf, line)) {
                std::string s = trim(line);
                if (s.empty() || s.front() == '#') continue;
                int64_t t = slam_common::integer_ns(s);
                times.push_back(t);
                std::string ts_s = std::to_string(t);
                left_names.push_back(std::filesystem::is_regular_file(source / "mav0/cam0/data" / (ts_s + ".jpg")) ? (ts_s + ".jpg") : (ts_s + ".png"));
                right_names.push_back(std::filesystem::is_regular_file(source / "mav0/cam1/data" / (ts_s + ".jpg")) ? (ts_s + ".jpg") : (ts_s + ".png"));
            }
        } else {
            auto read_cam_csv = [](const std::filesystem::path& p) {
                std::ifstream cf(p);
                if (!cf) throw std::runtime_error("Cannot open " + p.string());
                std::vector<std::pair<int64_t, std::string>> rows;
                std::string line;
                while (std::getline(cf, line)) {
                    std::string s = trim(line);
                    if (s.empty() || s.front() == '#') continue;
                    auto cols = split_csv(s);
                    if (cols.size() >= 2) rows.emplace_back(slam_common::integer_ns(cols[0]), trim(cols[1]));
                }
                return rows;
            };
            auto l = read_cam_csv(source / "mav0/cam0/data.csv");
            auto r = read_cam_csv(source / "mav0/cam1/data.csv");
            if (l.size() != r.size()) throw std::invalid_argument("Stereo timestamps must match; explicit synchronization required");
            for (size_t i = 0; i < l.size(); ++i) {
                if (l[i].first != r[i].first) throw std::invalid_argument("Stereo timestamps must match; explicit synchronization required");
                times.push_back(l[i].first);
                left_names.push_back(l[i].second);
                right_names.push_back(r[i].second);
            }
        }
        imu = slam_common::read_imu_csv(source / "mav0/imu0/data.csv");
    }

    slam_common::strictly_increasing(times, "Camera");
    slam_common::strictly_increasing(imu.timestamps_ns, "IMU");

    std::vector<double> imu_diffs_ns;
    std::vector<double> accel_norms;
    double max_imu_gap_s = 0.0;
    for (size_t i = 0; i < imu.samples.size(); ++i) {
        if (!imu.samples[i].allFinite()) throw std::invalid_argument("Invalid IMU measurements");
        accel_norms.push_back(imu.samples[i].tail<3>().norm());
        if (i > 0) {
            double dns = static_cast<double>(imu.timestamps_ns[i] - imu.timestamps_ns[i - 1]);
            imu_diffs_ns.push_back(dns);
            max_imu_gap_s = std::max(max_imu_gap_s, dns * 1e-9);
        }
    }
    double hz = 1e9 / median_vec(imu_diffs_ns);
    if (std::abs(hz / info.imu_hz - 1.0) > 0.1) {
        throw std::invalid_argument("IMU rate does not match calibration");
    }
    if (max_imu_gap_s > 5.0 / info.imu_hz) {
        std::ostringstream oss;
        oss << "IMU gap " << std::fixed << std::setprecision(6) << max_imu_gap_s << "s exceeds five sampling intervals";
        throw std::invalid_argument(oss.str());
    }
    double accel_median = median_vec(accel_norms);
    if (!(accel_median > 5.0 && accel_median < 15.0)) {
        std::ostringstream oss;
        oss << "Acceleration median " << std::fixed << std::setprecision(2) << accel_median
            << " m/s²: check units/range or unusually dynamic motion.";
        warnings.push_back(oss.str());
    }

    int64_t offset = 0;
    if (profile.contains("camera_to_imu_offset_ns") && profile.get("camera_to_imu_offset_ns").is_int()) {
        offset = profile.get("camera_to_imu_offset_ns").as_int();
    }

    std::vector<size_t> chosen;
    size_t strided_total = 0;
    for (size_t i = 0; i < times.size(); i += static_cast<size_t>(stride)) {
        ++strided_total;
        int64_t t_imu = times[i] + offset;
        if (t_imu >= imu.timestamps_ns.front() && t_imu <= imu.timestamps_ns.back()) {
            chosen.push_back(i);
        }
    }
    size_t trimmed = strided_total - chosen.size();
    if (trimmed > 0) {
        warnings.push_back(std::to_string(trimmed) + " image pairs outside IMU coverage were excluded.");
    }
    if (limit > 0 && chosen.size() > static_cast<size_t>(limit)) {
        chosen.resize(static_cast<size_t>(limit));
    }
    if (chosen.size() < 2) {
        throw std::invalid_argument("Insufficient image pairs inside IMU coverage");
    }

    std::vector<double> cam_dts_s;
    for (size_t i = 1; i < chosen.size(); ++i) {
        cam_dts_s.push_back(static_cast<double>(times[chosen[i]] - times[chosen[i - 1]]) * 1e-9);
    }
    double period = median_vec(cam_dts_s);
    int eff_fps = std::max(1, static_cast<int>(std::llround(1.0 / period)));

    std::string yaml_text = slam_common::read_text_utf8(settings);
    std::regex fps_re(R"(^Camera\.fps:.*$)", std::regex::multiline);
    std::ptrdiff_t fps_matches = std::distance(
        std::sregex_iterator(yaml_text.begin(), yaml_text.end(), fps_re),
        std::sregex_iterator());
    if (fps_matches != 1) throw std::invalid_argument("Expected one Camera.fps setting");
    yaml_text = std::regex_replace(yaml_text, fps_re, "Camera.fps: " + std::to_string(eff_fps));
    write_text_utf8(dest / "settings.yaml", yaml_text);

    std::filesystem::create_directories(dest / "left");
    std::filesystem::create_directories(dest / "right");

    cv::VideoCapture cap;
    if (legacy) {
        cap.open((source / "stereo.mkv").string());
        if (!cap.isOpened()) throw std::runtime_error("Cannot open stereo.mkv");
    }

    std::set<size_t> wanted(chosen.begin(), chosen.end());
    size_t written = 0;
    std::ofstream frames_out(dest / "frames.csv", std::ios::binary | std::ios::trunc);
    if (!frames_out) throw std::runtime_error("Cannot write frames.csv");

    for (size_t i = 0; i <= chosen.back(); ++i) {
        cv::Mat left_im, right_im;
        if (legacy) {
            if (!cap.grab()) throw std::invalid_argument("Video ends before timestamp row " + std::to_string(i + 1));
            if (!wanted.count(i)) continue;
            cv::Mat im;
            if (!cap.retrieve(im) || im.empty()) {
                throw std::invalid_argument("Video ends before timestamp row " + std::to_string(i + 1));
            }
            if (im.rows != info.shape.height || im.cols != 2 * info.shape.width) {
                throw std::invalid_argument("Video size does not match stereo calibration");
            }
            left_im = im.colRange(0, info.shape.width);
            right_im = im.colRange(info.shape.width, 2 * info.shape.width);
        } else {
            if (!wanted.count(i)) continue;
            std::filesystem::path cam0_dir = source / "mav0/cam0/data";
            std::filesystem::path cam1_dir = source / "mav0/cam1/data";
            std::filesystem::path f0 = cam0_dir / std::filesystem::u8path(left_names[i]);
            std::filesystem::path f1 = cam1_dir / std::filesystem::u8path(right_names[i]);
            if (!is_subpath(f0, cam0_dir) || !is_subpath(f1, cam1_dir)) {
                throw std::invalid_argument("Image filename leaves dataset");
            }
            left_im = cv::imread(f0.string(), cv::IMREAD_GRAYSCALE);
            right_im = cv::imread(f1.string(), cv::IMREAD_GRAYSCALE);
            if (left_im.empty() || right_im.empty() ||
                left_im.rows != info.shape.height || left_im.cols != info.shape.width ||
                right_im.rows != info.shape.height || right_im.cols != info.shape.width) {
                throw std::invalid_argument("Invalid image or size: " + f0.string());
            }
        }

        if (left_im.channels() > 1) cv::cvtColor(left_im, left_im, cv::COLOR_BGR2GRAY);
        if (right_im.channels() > 1) cv::cvtColor(right_im, right_im, cv::COLOR_BGR2GRAY);
        std::string ts_s = std::to_string(times[i]);
        std::string l_rel = "left/" + ts_s + ".png";
        std::string r_rel = "right/" + ts_s + ".png";
        if (!cv::imwrite((dest / l_rel).string(), left_im) || !cv::imwrite((dest / r_rel).string(), right_im)) {
            throw std::runtime_error("Failed to write prepared image");
        }
        frames_out << ts_s << ',' << l_rel << ',' << r_rel << '\n';
        ++written;
        if (written % 100 == 0) {
            std::cout << "PREPARE " << written << '/' << chosen.size() << std::endl;
        }
    }

    {
        std::ofstream imu_out(dest / "imu.csv", std::ios::binary | std::ios::trunc);
        if (!imu_out) throw std::runtime_error("Cannot write imu.csv");
        imu_out << std::setprecision(15);
        for (size_t k = 0; k < imu.timestamps_ns.size(); ++k) {
            imu_out << (imu.timestamps_ns[k] - offset);
            for (int c = 0; c < 6; ++c) imu_out << ',' << imu.samples[k](c);
            imu_out << '\n';
        }
    }

    JsonValue warn_arr = JsonValue::array();
    for (const auto& w : warnings) warn_arr.push_back(w);
    JsonValue result = JsonValue::object();
    result["adapter"] = legacy ? "umi_ms901m_uniform" : "euroc";
    result["source_frames"] = static_cast<int64_t>(times.size());
    result["selected_frames"] = static_cast<int64_t>(written);
    result["imu_samples"] = static_cast<int64_t>(imu.timestamps_ns.size());
    result["stride"] = static_cast<int64_t>(stride);
    result["camera_to_imu_offset_ns"] = offset;
    result["median_frame_period_s"] = period;
    result["imu_hz_observed"] = hz;
    result["accel_norm_median_m_s2"] = accel_median;
    result["max_imu_gap_s"] = max_imu_gap_s;
    result["first_timestamp_ns"] = times[chosen.front()];
    result["last_timestamp_ns"] = times[chosen.back()];
    result["warnings"] = warn_arr;
    result["calibration_sha256"] = slam_common::sha256_file(settings);
    slam_common::save_json(dest / "input_report.json", result, true);
    return result;
}

JsonValue export_result(const std::filesystem::path& out, const JsonValue& report) {
    struct ExportRow {
        int64_t ts_ns;
        std::array<double, 7> pose;  // x, y, z, qx, qy, qz, qw
    };
    std::vector<ExportRow> rows;
    {
        std::ifstream f(out / "imu_euroc.txt");
        if (!f) throw std::runtime_error("Missing imu_euroc.txt");
        std::string line;
        while (std::getline(f, line)) {
            std::istringstream ss(line);
            std::string ts_tok;
            if (!(ss >> ts_tok)) continue;
            ExportRow row{};
            row.ts_ns = round_decimal_timestamp_ns(ts_tok);
            for (int i = 0; i < 7; ++i) {
                if (!(ss >> row.pose[static_cast<size_t>(i)])) {
                    throw std::invalid_argument("Malformed trajectory output");
                }
            }
            std::string extra;
            if (ss >> extra) throw std::invalid_argument("Malformed trajectory output");
            rows.push_back(row);
        }
    }
    if (rows.empty()) throw std::invalid_argument("Backend exported no trajectory");
    {
        std::vector<int64_t> row_ts;
        row_ts.reserve(rows.size());
        for (const auto& r : rows) row_ts.push_back(r.ts_ns);
        slam_common::strictly_increasing(row_ts, "Trajectory");
    }

    struct TrackState {
        std::string state;
        std::string imu_initialized;
        std::string map_id;
    };
    std::map<int64_t, TrackState> states;
    std::vector<int64_t> stamps;
    std::set<std::string> seen_maps;
    {
        std::ifstream tf(out / "tracking.csv");
        if (!tf) throw std::runtime_error("Missing tracking.csv");
        std::string hdr;
        std::getline(tf, hdr);
        auto cols = split_csv(hdr);
        auto find_col = [&](const std::string& name) -> int {
            for (size_t i = 0; i < cols.size(); ++i) {
                if (trim(cols[i]) == name) return static_cast<int>(i);
            }
            throw std::invalid_argument("Missing column in tracking.csv: " + name);
        };
        int its = find_col("timestamp_ns");
        int ist = find_col("state");
        int iimu = find_col("imu_initialized");
        int imap = find_col("map_id");
        std::string line;
        while (std::getline(tf, line)) {
            if (trim(line).empty()) continue;
            auto r = split_csv(line);
            int64_t ts = std::stoll(trim(r[its]));
            TrackState st{trim(r[ist]), trim(r[iimu]), trim(r[imap])};
            states[ts] = st;
            stamps.push_back(ts);
            seen_maps.insert(st.map_id);
        }
    }
    std::sort(stamps.begin(), stamps.end());

    struct MatchedRow {
        int64_t stamp;
        std::array<double, 7> pose;
        std::string map_id;
    };
    std::vector<MatchedRow> matched;
    for (const auto& r : rows) {
        double qn2 = r.pose[3] * r.pose[3] + r.pose[4] * r.pose[4] + r.pose[5] * r.pose[5] + r.pose[6] * r.pose[6];
        for (double v : r.pose) {
            if (!std::isfinite(v)) throw std::invalid_argument("Invalid pose/quaternion");
        }
        if (std::abs(std::sqrt(qn2) - 1.0) > 0.01) throw std::invalid_argument("Invalid pose/quaternion");

        auto it = std::lower_bound(stamps.begin(), stamps.end(), r.ts_ns);
        int64_t best_stamp = -1;
        int64_t best_diff = std::numeric_limits<int64_t>::max();
        if (it != stamps.end()) {
            best_diff = std::abs(*it - r.ts_ns);
            best_stamp = *it;
        }
        if (it != stamps.begin()) {
            int64_t prev = *(it - 1);
            int64_t diff = std::abs(prev - r.ts_ns);
            if (diff < best_diff) {
                best_diff = diff;
                best_stamp = prev;
            }
        }
        if (best_stamp < 0 || best_diff > 1000) {
            throw std::invalid_argument("Exported pose has no matching image timestamp");
        }
        const auto& st = states[best_stamp];
        if (st.imu_initialized == "1" && st.state == "2") {
            matched.push_back({best_stamp, r.pose, st.map_id});
        }
    }
    if (matched.size() < 2) throw std::invalid_argument("No usable initialized IMU trajectory");
    {
        std::vector<int64_t> mts;
        mts.reserve(matched.size());
        for (const auto& m : matched) mts.push_back(m.stamp);
        slam_common::strictly_increasing(mts, "Matched trajectory");
    }

    int64_t start = report.get("first_timestamp_ns").as_int();
    double med_period_s = report.get("median_frame_period_s").as_double();
    int64_t selected_frames = report.get("selected_frames").as_int();
    int segment = 0;
    bool has_prev = false;
    int64_t prev_stamp = 0;
    std::string prev_map;

    {
        std::ofstream csv_f(out / "imu_trajectory.csv", std::ios::binary | std::ios::trunc);
        std::ofstream tum_f(out / "imu_trajectory.tum", std::ios::binary | std::ios::trunc);
        csv_f << "timestamp_ns,time_from_input_start_s,x_m,y_m,z_m,qx,qy,qz,qw,segment_id\n";
        csv_f << std::setprecision(15);
        for (const auto& m : matched) {
            if (has_prev && (static_cast<double>(m.stamp - prev_stamp) > med_period_s * 1e9 * 2.5 || m.map_id != prev_map)) {
                ++segment;
            }
            csv_f << m.stamp << ',' << (static_cast<double>(m.stamp - start) / 1e9);
            for (double v : m.pose) csv_f << ',' << v;
            csv_f << ',' << segment << '\n';

            tum_f << (m.stamp / 1000000000LL) << '.' << std::setw(9) << std::setfill('0') << (m.stamp % 1000000000LL);
            tum_f << std::fixed << std::setprecision(9);
            for (double v : m.pose) tum_f << ' ' << v;
            tum_f << '\n';

            has_prev = true;
            prev_stamp = m.stamp;
            prev_map = m.map_id;
        }
    }

    double coverage = static_cast<double>(matched.size()) / static_cast<double>(std::max<int64_t>(1, selected_frames));
    JsonValue warnings = JsonValue::array();
    if (report.get("warnings").is_array()) warnings = report.get("warnings");
    if (coverage < 0.99) {
        warnings.push_back("Trajectory does not cover every selected frame; see tracking.csv for initialization/loss.");
    }
    if (seen_maps.size() > 1) {
        warnings.push_back("SLAM changed maps. Export contains only the map selected by ORB-SLAM3, not all motion.");
    }

    JsonValue quality = JsonValue::object();
    quality["status"] = "completed";
    quality["pose"] = "T_world_imu";
    quality["position_unit"] = "m";
    quality["quaternion_order"] = "qx,qy,qz,qw";
    quality["timestamp_unit"] = "ns";
    quality["time_basis"] = "original camera clock (not Unix time)";
    quality["world_frame"] = "ORB-SLAM3 export frame: first keyframe IMU frame of selected map; not robot base";
    quality["source_export"] = "ORB-SLAM3 largest map, finalized after shutdown";
    quality["output_frames"] = static_cast<int64_t>(matched.size());
    quality["selected_frames"] = selected_frames;
    quality["coverage_fraction"] = coverage;
    quality["segments"] = static_cast<int64_t>(segment + 1);
    quality["first_output_timestamp_ns"] = matched.front().stamp;
    quality["last_output_timestamp_ns"] = matched.back().stamp;
    quality["accuracy"] = "not measured; no independent reference supplied";
    quality["warnings"] = warnings;

    auto count_nonempty_lines = [](const std::filesystem::path& p) {
        std::ifstream f(p);
        int cnt = 0;
        std::string ln;
        while (std::getline(f, ln)) {
            if (!trim(ln).empty()) ++cnt;
        }
        return cnt;
    };
    if (std::filesystem::is_regular_file(out / "keyframes_euroc.txt")) {
        quality["keyframes"] = static_cast<int64_t>(count_nonempty_lines(out / "keyframes_euroc.txt"));
    }
    if (std::filesystem::is_regular_file(out / "map_points.csv")) {
        quality["map_points"] = static_cast<int64_t>(std::max(0, count_nonempty_lines(out / "map_points.csv") - 1));
    }
    slam_common::save_json(out / "quality.json", quality, true);

    std::ostringstream projections;
    struct AxisPair { int a, b; const char* label; };
    for (const auto& ap : std::array<AxisPair, 3>{{{0, 1, "XY"}, {0, 2, "XZ"}, {1, 2, "YZ"}}}) {
        double lo_a = matched[0].pose[ap.a], hi_a = lo_a;
        double lo_b = matched[0].pose[ap.b], hi_b = lo_b;
        for (const auto& m : matched) {
            lo_a = std::min(lo_a, m.pose[ap.a]); hi_a = std::max(hi_a, m.pose[ap.a]);
            lo_b = std::min(lo_b, m.pose[ap.b]); hi_b = std::max(hi_b, m.pose[ap.b]);
        }
        double span_a = hi_a - lo_a, span_b = hi_b - lo_b;
        double scale = 340.0 / std::max({span_a, span_b, 1e-6});
        double mid_a = lo_a + 0.5 * span_a, mid_b = lo_b + 0.5 * span_b;
        std::ostringstream lines_svg;
        std::ostringstream cur_pts;
        bool p_prev = false;
        int64_t p_stamp = 0;
        std::string p_map;
        auto flush_group = [&]() {
            std::string pts = cur_pts.str();
            if (!pts.empty()) {
                lines_svg << "<polyline fill=\"none\" stroke=\"#15803d\" stroke-width=\"1.5\" points=\"" << pts << "\"/>";
            }
            cur_pts.str("");
            cur_pts.clear();
        };
        for (const auto& m : matched) {
            if (p_prev && (static_cast<double>(m.stamp - p_stamp) > med_period_s * 1e9 * 2.5 || m.map_id != p_map)) {
                flush_group();
            }
            double sx = (m.pose[ap.a] - mid_a) * scale + 200.0;
            double sy = (m.pose[ap.b] - mid_b) * scale + 200.0;
            if (!cur_pts.str().empty()) cur_pts << ' ';
            cur_pts << std::fixed << std::setprecision(2) << sx << ',' << (400.0 - sy);
            p_prev = true; p_stamp = m.stamp; p_map = m.map_id;
        }
        flush_group();
        projections << std::fixed << std::setprecision(4)
                    << "<section><h2>" << ap.label << " · metres</h2><svg viewBox=\"0 0 400 400\">"
                    << lines_svg.str() << "</svg><p>Range: " << span_a << " × " << span_b << " m</p></section>";
    }
    std::string html_doc =
        "<!doctype html><meta charset=\"utf-8\"><title>UMI IMU trajectory</title>"
        "<style>body{font:16px sans-serif;margin:32px;background:#f6f8fa}main{display:flex}"
        "section{width:32%;background:white;margin:8px;padding:12px}pre{white-space:pre-wrap}svg{width:100%}</style>"
        "<h1>UMI · IMU 原点轨迹</h1><p>位置单位：米；精度尚未通过独立真值验证。断线表示输出间断或地图切换。</p><main>" +
        projections.str() + "</main><pre>" + slam_common::html_escape(quality.dump(2)) + "</pre>";
    write_text_utf8(out / "report.html", html_doc);
    return quality;
}

BackendConfig load_backend_config(const std::filesystem::path& backend_path) {
    std::filesystem::path cfg_path = std::filesystem::weakly_canonical(backend_path);
    JsonValue raw = slam_common::read_json_file(cfg_path);
    std::filesystem::path base = cfg_path.parent_path();
    auto res_path = [&](const std::string& p) -> std::string {
        if (p.empty()) return "";
        std::filesystem::path path = std::filesystem::u8path(p);
        return path.is_absolute() ? path.string() : std::filesystem::weakly_canonical(base / path).string();
    };
    BackendConfig cfg;
    if (raw.get("binary").is_string()) cfg.binary = res_path(raw.get("binary").as_string());
    if (raw.get("vocabulary").is_string()) cfg.vocabulary = res_path(raw.get("vocabulary").as_string());
    if (raw.get("library_paths").is_array()) {
        for (const auto& item : raw.get("library_paths").as_array()) {
            if (item.is_string() && !item.as_string().empty()) {
                cfg.library_paths.push_back(res_path(item.as_string()));
            }
        }
    }
    if (raw.get("source_sha256").is_string()) cfg.source_sha256 = raw.get("source_sha256").as_string();
    return cfg;
}

std::map<std::string, std::string> backend_env(
    const BackendConfig& backend,
    const std::map<std::string, std::string>* base_env) {
    std::map<std::string, std::string> env;
    if (base_env != nullptr) env = *base_env;
#ifdef _WIN32
    const char sep = ';';
    const std::string key = "PATH";
#elif defined(__APPLE__)
    const char sep = ':';
    const std::string key = "DYLD_LIBRARY_PATH";
#else
    const char sep = ':';
    const std::string key = "LD_LIBRARY_PATH";
#endif
    std::string joined;
    for (const auto& p : backend.library_paths) {
        if (!p.empty()) {
            if (!joined.empty()) joined.push_back(sep);
            joined += p;
        }
    }
    auto it = env.find(key);
    if (it != env.end() && !it->second.empty()) {
        if (!joined.empty()) joined.push_back(sep);
        joined += it->second;
    }
    env[key] = joined;
    return env;
}

void patch_orb(const std::filesystem::path& orb_root) {
    std::filesystem::path cmake = orb_root / "CMakeLists.txt";
    std::string cmake_text = slam_common::read_text_utf8(cmake);
    size_t anchor = cmake_text.find("# Build examples");
    if (anchor == std::string::npos) {
        throw std::runtime_error("ORB-SLAM3 patch mismatch (CMakeLists.txt): missing \"# Build examples\" anchor");
    }
    write_text_utf8(cmake, replace_all(cmake_text.substr(0, anchor), "-march=native", ""));

    std::filesystem::path thirdparty = orb_root / "Thirdparty";
    if (std::filesystem::is_directory(thirdparty)) {
        for (const auto& entry : std::filesystem::recursive_directory_iterator(thirdparty)) {
            if (entry.is_regular_file() && entry.path().filename() == "CMakeLists.txt") {
                std::string t = slam_common::read_text_utf8(entry.path());
                write_text_utf8(entry.path(), replace_all(t, "-march=native", ""));
            }
        }
    }

    std::filesystem::path header = orb_root / "include/System.h";
    std::string header_patched = replace_exact(
        slam_common::read_text_utf8(header),
        "    int GetTrackingState();",
        "    int GetTrackingState();\n    bool OfflineImuInitialized();\n    unsigned long OfflineMapId();\n    void OfflineSaveMapPoints(const string &filename);",
        1,
        "System.h GetTrackingState declaration");
    write_text_utf8(header, header_patched);

    std::filesystem::path source = orb_root / "src/System.cc";
    std::string text = slam_common::read_text_utf8(source);
    text = replace_exact(text, "Map* pBiggerMap;", "Map* pBiggerMap = nullptr;", 4, "System.cc pBiggerMap init");

    size_t shutdown_pos = text.find("void System::Shutdown()");
    size_t start = (shutdown_pos != std::string::npos) ? text.find("    /*if(mpViewer)", shutdown_pos) : std::string::npos;
    size_t end = (start != std::string::npos) ? text.find("    if(!mStrSaveAtlasToFile.empty())", start) : std::string::npos;
    if (shutdown_pos == std::string::npos || start == std::string::npos || end == std::string::npos) {
        throw std::runtime_error("ORB-SLAM3 patch mismatch (System.cc Shutdown block)");
    }
    text = text.substr(0, start) +
           "    if(mpViewer) mpViewer->RequestFinish();\n"
           "    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||\n"
           "          mpLoopCloser->isRunningGBA() || (mpViewer && !mpViewer->isFinished()))\n"
           "        std::this_thread::sleep_for(std::chrono::microseconds(5000));\n\n" +
           text.substr(end);

    const std::string needle = "    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();";
    text = replace_exact(
        text,
        needle,
        "    if(!pBiggerMap || pBiggerMap->KeyFramesInMap() == 0) return;\n" + needle,
        4,
        "System.cc empty atlas guard");

    const std::string marker = "int System::GetTrackingState()";
    const std::string helpers =
        "bool System::OfflineImuInitialized() { return mpAtlas->isImuInitialized(); }\n"
        "unsigned long System::OfflineMapId() { return mpAtlas->GetCurrentMap()->GetId(); }\n"
        "void System::OfflineSaveMapPoints(const string &filename)\n"
        "{\n"
        "    vector<Map*> vpMaps = mpAtlas->GetAllMaps();\n"
        "    Map* pBiggerMap = nullptr;\n"
        "    int numMaxKFs = 0;\n"
        "    for(Map* pMap : vpMaps)\n"
        "    {\n"
        "        if(pMap && pMap->GetAllKeyFrames().size() > (size_t)numMaxKFs)\n"
        "        {\n"
        "            numMaxKFs = pMap->GetAllKeyFrames().size();\n"
        "            pBiggerMap = pMap;\n"
        "        }\n"
        "    }\n"
        "    if(!pBiggerMap || pBiggerMap->KeyFramesInMap() == 0) return;\n"
        "    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();\n"
        "    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);\n"
        "    Sophus::SE3f Twb0 = (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)\n"
        "        ? vpKFs[0]->GetImuPose()\n"
        "        : vpKFs[0]->GetPoseInverse();\n"
        "    Sophus::SE3f Tb0w = Twb0.inverse();\n"
        "    ofstream f(filename.c_str());\n"
        "    f << \"x_m,y_m,z_m\\n\" << fixed << setprecision(6);\n"
        "    for(MapPoint* pMP : pBiggerMap->GetAllMapPoints())\n"
        "    {\n"
        "        if(!pMP || pMP->isBad()) continue;\n"
        "        Eigen::Vector3f p = Tb0w * pMP->GetWorldPos();\n"
        "        if(!p.allFinite()) continue;\n"
        "        f << p(0) << ',' << p(1) << ',' << p(2) << '\\n';\n"
        "    }\n"
        "}\n\n" + marker;
    text = replace_exact(text, marker, helpers, 1, "System.cc offline helpers");
    write_text_utf8(source, text);
}

int run_pipeline_main(const PipelineArgs& args) {
    g_cancel_requested.store(false);
    std::signal(SIGTERM, signal_cancel_handler);
    std::signal(SIGINT, signal_cancel_handler);
#ifdef _WIN32
    SetConsoleCtrlHandler(console_ctrl_handler, TRUE);
#endif

    std::filesystem::path source = std::filesystem::weakly_canonical(args.input);
    std::filesystem::path output = std::filesystem::weakly_canonical(args.output);
    std::filesystem::path settings = std::filesystem::weakly_canonical(args.settings);
    if (output == source || is_subpath(source, output) || is_subpath(output, source)) {
        throw std::invalid_argument("Input and output directories must be separate, not nested");
    }
    if (!std::filesystem::is_directory(source)) throw std::invalid_argument("Input directory does not exist");
    if (std::filesystem::exists(output)) throw std::invalid_argument("Output directory already exists");

    JsonValue profile = (!args.profile.empty() && std::filesystem::is_regular_file(args.profile))
        ? slam_common::read_json_file(args.profile)
        : JsonValue::object();
    BackendConfig backend;
    if (!args.prepare_only) {
        backend = load_backend_config(args.backend);
    }

    std::filesystem::create_directories(output);
    auto unix_now = []() -> double {
        return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    };

    JsonValue state = JsonValue::object();
    state["status"] = "preparing";
    state["input"] = source.string();
    state["settings"] = settings.string();
    state["profile"] = profile;
    state["started_unix_s"] = unix_now();
    slam_common::save_json(output / "job.json", state, true);

    try {
        std::filesystem::path prepared;
        JsonValue report;
        if (args.prepared_input) {
            prepared = source;
            report = slam_common::read_json_file(prepared / "input_report.json");
            if (slam_common::sha256_file(settings) != report.get("calibration_sha256").as_string()) {
                throw std::invalid_argument("Prepared input was produced with different calibration");
            }
            settings_info(prepared / "settings.yaml");
            state["prepared_input"] = prepared.string();
        } else {
            prepared = output / "prepared";
            std::filesystem::create_directories(prepared);
            report = prepare(source, prepared, profile, settings, args.stride, args.max_frames);
        }

        if (args.prepare_only) {
            state["status"] = "prepared";
            state["finished_unix_s"] = unix_now();
            slam_common::save_json(output / "job.json", state, true);
            return 0;
        }

        state["status"] = "running";
        slam_common::save_json(output / "job.json", state, true);

        double duration_s = static_cast<double>(report.get("last_timestamp_ns").as_int() -
                                                report.get("first_timestamp_ns").as_int()) / 1e9;
        double timeout_s = (args.timeout > 0.0) ? args.timeout : std::max(180.0, duration_s * 5.0 + 120.0);

        int child_rc = -1;
        auto env_map = backend_env(backend);
#ifdef _WIN32
        auto quote_win = [](const std::string& s) -> std::string {
            std::string q = "\"";
            for (char c : s) {
                if (c == '"') q += "\\\"";
                else q.push_back(c);
            }
            q.push_back('"');
            return q;
        };
        std::string cmd_line = quote_win(backend.binary) + " " +
                               quote_win(backend.vocabulary) + " " +
                               quote_win((prepared / "settings.yaml").string()) + " " +
                               quote_win(prepared.string()) + " " +
                               quote_win(output.string());
        if (args.viewer) cmd_line += " --viewer";

        for (const auto& kv : env_map) {
            SetEnvironmentVariableA(kv.first.c_str(), kv.second.c_str());
        }
        SECURITY_ATTRIBUTES sa{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
        HANDLE log_h = CreateFileA((output / "slam.log").string().c_str(), GENERIC_WRITE,
                                   FILE_SHARE_READ | FILE_SHARE_WRITE, &sa, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
        STARTUPINFOA si{};
        si.cb = sizeof(si);
        si.dwFlags = STARTF_USESTDHANDLES;
        si.hStdOutput = log_h;
        si.hStdError = log_h;
        PROCESS_INFORMATION pi{};
        DWORD flags = CREATE_NEW_PROCESS_GROUP | (args.viewer ? 0u : CREATE_NO_WINDOW);
        if (!CreateProcessA(nullptr, cmd_line.data(), nullptr, nullptr, TRUE, flags, nullptr,
                            output.string().c_str(), &si, &pi)) {
            if (log_h != INVALID_HANDLE_VALUE) CloseHandle(log_h);
            throw std::runtime_error("Failed to start umi_slam_runner");
        }
        if (log_h != INVALID_HANDLE_VALUE) CloseHandle(log_h);

        auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
        while (true) {
            DWORD wait_res = WaitForSingleObject(pi.hProcess, 200);
            if (wait_res == WAIT_OBJECT_0) {
                DWORD ec = 0;
                GetExitCodeProcess(pi.hProcess, &ec);
                child_rc = static_cast<int>(ec);
                break;
            }
            if (g_cancel_requested.load() || std::chrono::steady_clock::now() > deadline) {
                auto desc = windows_descendant_pids(pi.dwProcessId);
                TerminateProcess(pi.hProcess, 1);
                WaitForSingleObject(pi.hProcess, 2000);
                windows_kill_pids(desc);
                CloseHandle(pi.hThread);
                CloseHandle(pi.hProcess);
                if (g_cancel_requested.load()) {
                    state["status"] = "cancelled";
                    state["finished_unix_s"] = unix_now();
                    slam_common::save_json(output / "job.json", state, true);
                    return 130;
                }
                throw std::runtime_error("SLAM exceeded timeout; see slam.log");
            }
        }
        CloseHandle(pi.hThread);
        CloseHandle(pi.hProcess);
#else
        pid_t pid = fork();
        if (pid == 0) {
            setsid();
            for (const auto& kv : env_map) setenv(kv.first.c_str(), kv.second.c_str(), 1);
            std::filesystem::current_path(output);
            FILE* lf = fopen((output / "slam.log").c_str(), "w");
            if (lf) {
                dup2(fileno(lf), STDOUT_FILENO);
                dup2(fileno(lf), STDERR_FILENO);
                fclose(lf);
            }
            std::string s_path = (prepared / "settings.yaml").string();
            std::string p_path = prepared.string();
            std::string o_path = output.string();
            std::vector<char*> argv_exec = {
                const_cast<char*>(backend.binary.c_str()),
                const_cast<char*>(backend.vocabulary.c_str()),
                const_cast<char*>(s_path.c_str()),
                const_cast<char*>(p_path.c_str()),
                const_cast<char*>(o_path.c_str()),
            };
            std::string vw = "--viewer";
            if (args.viewer) argv_exec.push_back(const_cast<char*>(vw.c_str()));
            argv_exec.push_back(nullptr);
            execvp(backend.binary.c_str(), argv_exec.data());
            _exit(127);
        }
        auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
        while (true) {
            int wstatus = 0;
            pid_t wp = waitpid(pid, &wstatus, WNOHANG);
            if (wp == pid) {
                child_rc = WIFEXITED(wstatus) ? WEXITSTATUS(wstatus) : 1;
                break;
            }
            if (g_cancel_requested.load() || std::chrono::steady_clock::now() > deadline) {
                killpg(pid, SIGTERM);
                std::this_thread::sleep_for(std::chrono::seconds(2));
                if (waitpid(pid, &wstatus, WNOHANG) == 0) killpg(pid, SIGKILL);
                waitpid(pid, &wstatus, 0);
                if (g_cancel_requested.load()) {
                    state["status"] = "cancelled";
                    state["finished_unix_s"] = unix_now();
                    slam_common::save_json(output / "job.json", state, true);
                    return 130;
                }
                throw std::runtime_error("SLAM exceeded timeout; see slam.log");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
#endif

        state["backend_exit_code"] = static_cast<int64_t>(child_rc);
        if (child_rc != 0) {
            throw std::runtime_error("SLAM exited with code " + std::to_string(child_rc) + "; see slam.log");
        }
        JsonValue quality = export_result(output, report);
        state["status"] = "completed";
        state["coverage_fraction"] = quality.get("coverage_fraction");
        state["finished_unix_s"] = unix_now();
        slam_common::save_json(output / "job.json", state, true);
        return 0;
    } catch (const std::exception& exc) {
        state["status"] = g_cancel_requested.load() ? "cancelled" : "failed";
        state["error"] = exc.what();
        state["finished_unix_s"] = unix_now();
        slam_common::save_json(output / "job.json", state, true);
        std::cerr << "ERROR: " << exc.what() << std::endl;
        return g_cancel_requested.load() ? 130 : 1;
    }
}

int run_pipeline_cli(int argc, char** argv) {
    try {
        PipelineArgs args;
        bool build_backend_mode = false;
        bool show_help = false;
        std::string checksum_mode;
        BackendBuildOptions backend_opt;
        bool has_source_root = false, has_runner_dir = false, has_build_dir = false;
        std::filesystem::path repo_root_hint;
        for (int i = 1; i < argc; ++i) {
            std::string flag = argv[i];
            auto need_val = [&](const std::string& f) -> std::string {
                if (i + 1 >= argc) throw std::invalid_argument("Missing value for " + f);
                return argv[++i];
            };
            if (flag == "--help" || flag == "-h") show_help = true;
            else if (flag == "--input") args.input = std::filesystem::u8path(need_val(flag));
            else if (flag == "--output") args.output = std::filesystem::u8path(need_val(flag));
            else if (flag == "--settings") args.settings = std::filesystem::u8path(need_val(flag));
            else if (flag == "--profile") args.profile = std::filesystem::u8path(need_val(flag));
            else if (flag == "--backend") args.backend = std::filesystem::u8path(need_val(flag));
            else if (flag == "--stride") args.stride = std::stoi(need_val(flag));
            else if (flag == "--max-frames") args.max_frames = std::stoi(need_val(flag));
            else if (flag == "--timeout") args.timeout = std::stod(need_val(flag));
            else if (flag == "--prepare-only") args.prepare_only = true;
            else if (flag == "--prepared-input") args.prepared_input = true;
            else if (flag == "--viewer") args.viewer = true;
            else if (flag == "--build-backend") build_backend_mode = true;
            else if (flag == "--source-root") { backend_opt.source_root = std::filesystem::u8path(need_val(flag)); has_source_root = true; }
            else if (flag == "--runner-dir") { backend_opt.runner_dir = std::filesystem::u8path(need_val(flag)); has_runner_dir = true; }
            else if (flag == "--build-dir") { backend_opt.build_dir = std::filesystem::u8path(need_val(flag)); has_build_dir = true; }
            else if (flag == "--jobs") backend_opt.jobs = std::stoi(need_val(flag));
            else if (flag == "--cmake") backend_opt.cmake_exe = need_val(flag);
            else if (flag == "--verify-checksums") checksum_mode = "verify";
            else if (flag == "--update-checksums") checksum_mode = "update";
            else if (flag == "--repo-root") repo_root_hint = std::filesystem::u8path(need_val(flag));
        }

        if (show_help) {
            print_pipeline_usage();
            return 0;
        }

        if (checksum_mode == "verify" || checksum_mode == "update") {
            std::filesystem::path root = repo_root_hint.empty()
                ? find_checksum_repo_root()
                : std::filesystem::weakly_canonical(repo_root_hint);
            if (checksum_mode == "update") {
                int n = slam_common::update_repo_checksums(root);
                std::cout << "Updated SHA256SUMS: " << n << " files" << std::endl;
                return 0;
            }
            auto report = slam_common::verify_repo_checksums(root);
            std::cout << "Checked " << report.checked_count << " files" << std::endl;
            for (const auto& m : report.mismatches) {
                std::cout << "MISMATCH " << m.rel_path
                          << " expected=" << m.expected << " actual=" << m.actual << std::endl;
            }
            return report.mismatches.empty() ? 0 : 1;
        }

        if (build_backend_mode) {
            std::optional<std::filesystem::path> repo_root;
            auto repo = [&]() -> const std::filesystem::path& {
                if (!repo_root.has_value()) repo_root = find_backend_repo_root();
                return *repo_root;
            };
            if (!has_source_root) backend_opt.source_root = repo() / "source/stereo_imu_trajectory";
            if (!has_runner_dir) backend_opt.runner_dir = repo() / "software/umi_slam/backend";
            if (!has_build_dir) {
#ifdef _WIN32
                backend_opt.build_dir = repo() / "software/umi_slam/.build-windows";
#else
                backend_opt.build_dir = repo() / "software/umi_slam/.build-linux";
#endif
            }
            auto cfg = build_backend(backend_opt);
            std::cout << "Backend ready: " << cfg.string() << std::endl;
            return 0;
        }

        if (args.input.empty() || args.output.empty() || args.settings.empty()) {
            print_pipeline_usage();
            return 2;
        }
        return run_pipeline_main(args);
    } catch (const std::exception& exc) {
        std::cerr << "ERROR: " << exc.what() << std::endl;
        return 1;
    }
}

}  // namespace umi_slam
