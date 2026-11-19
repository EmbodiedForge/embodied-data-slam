#include "camera_imu_calibration/engine.hpp"

#include "slam_common/geometry.hpp"
#include "slam_common/ms901m.hpp"
#include "slam_common/sha256.hpp"

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace camera_imu_calibration {
namespace {

using slam_common::JsonValue;

std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::string trim_lower(const std::string& s) {
    std::string out = trim(s);
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

std::string strip_yaml_comment(const std::string& line) {
    char in_quote = 0;
    std::string out;
    for (char ch : line) {
        if (ch == '\'' || ch == '"') {
            if (in_quote == 0) in_quote = ch;
            else if (in_quote == ch) in_quote = 0;
        } else if (ch == '#' && in_quote == 0) {
            break;
        }
        out.push_back(ch);
    }
    while (!out.empty() && (out.back() == ' ' || out.back() == '\t' || out.back() == '\r')) {
        out.pop_back();
    }
    return out;
}

JsonValue parse_yaml_scalar(const std::string& token) {
    std::string t = trim(token);
    if (t.empty()) return JsonValue(nullptr);
    if ((t.front() == '"' && t.back() == '"' && t.size() >= 2) ||
        (t.front() == '\'' && t.back() == '\'' && t.size() >= 2)) {
        return JsonValue(t.substr(1, t.size() - 2));
    }
    std::string low = trim_lower(t);
    if (low == "true") return JsonValue(true);
    if (low == "false") return JsonValue(false);
    if (low == "null" || low == "~") return JsonValue(nullptr);
    if (t.front() == '[' && t.back() == ']') {
        std::string inner = trim(t.substr(1, t.size() - 2));
        JsonValue arr = JsonValue::array();
        if (inner.empty()) return arr;
        std::stringstream ss(inner);
        std::string part;
        while (std::getline(ss, part, ',')) {
            arr.push_back(parse_yaml_scalar(part));
        }
        return arr;
    }
    static const std::regex kIntRe(R"(^[+-]?\d+$)");
    if (std::regex_match(t, kIntRe)) {
        try {
            return JsonValue(static_cast<int64_t>(std::stoll(t)));
        } catch (...) {
        }
    }
    try {
        size_t idx = 0;
        double d = std::stod(t, &idx);
        if (idx == t.size() && std::isfinite(d)) return JsonValue(d);
    } catch (...) {
    }
    return JsonValue(t);
}

struct YamlLine {
    int indent = 0;
    std::string content;
};

JsonValue parse_yaml_block(const std::vector<YamlLine>& lines, size_t& idx, int min_indent) {
    if (idx >= lines.size()) return JsonValue::object();
    if (lines[idx].content.rfind("- ", 0) == 0) {
        JsonValue arr = JsonValue::array();
        int list_indent = lines[idx].indent;
        while (idx < lines.size() && lines[idx].indent == list_indent && lines[idx].content.rfind("- ", 0) == 0) {
            arr.push_back(parse_yaml_scalar(lines[idx].content.substr(2)));
            ++idx;
        }
        return arr;
    }
    JsonValue mapping = JsonValue::object();
    int map_indent = lines[idx].indent;
    while (idx < lines.size()) {
        int ind = lines[idx].indent;
        const std::string& content = lines[idx].content;
        if (ind < min_indent || ind < map_indent || ind > map_indent || content.rfind("- ", 0) == 0) {
            break;
        }
        size_t colon = content.find(':');
        if (colon == std::string::npos) {
            ++idx;
            continue;
        }
        std::string k = trim(content.substr(0, colon));
        if (k.size() >= 2 && ((k.front() == '"' && k.back() == '"') || (k.front() == '\'' && k.back() == '\''))) {
            k = k.substr(1, k.size() - 2);
        }
        std::string rest = trim(content.substr(colon + 1));
        ++idx;
        if (!rest.empty()) {
            mapping[k] = parse_yaml_scalar(rest);
        } else if (idx < lines.size() && lines[idx].indent >= ind) {
            if (lines[idx].indent == ind && lines[idx].content.rfind("- ", 0) == 0) {
                mapping[k] = parse_yaml_block(lines, idx, ind);
            } else if (lines[idx].indent > ind) {
                mapping[k] = parse_yaml_block(lines, idx, ind + 1);
            } else {
                mapping[k] = JsonValue(nullptr);
            }
        } else {
            mapping[k] = JsonValue(nullptr);
        }
    }
    return mapping;
}

Eigen::Matrix3d mat_cv_to_eigen(const cv::Mat& m) {
    cv::Mat d;
    m.convertTo(d, CV_64F);
    Eigen::Matrix3d out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) out(r, c) = d.at<double>(r, c);
    }
    return out;
}

cv::Mat mat_eigen_to_cv(const Eigen::Matrix3d& m) {
    cv::Mat out(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) out.at<double>(r, c) = m(r, c);
    }
    return out;
}

cv::Mat dist4_to_cv5(const Eigen::Vector4d& d) {
    cv::Mat out = cv::Mat::zeros(5, 1, CV_64F);
    for (int i = 0; i < 4; ++i) out.at<double>(i, 0) = d(i);
    return out;
}

Eigen::Vector4d cv_dist_to_vec4(const cv::Mat& d_in) {
    cv::Mat d;
    d_in.convertTo(d, CV_64F);
    cv::Mat flat = d.reshape(1, 1);
    Eigen::Vector4d out = Eigen::Vector4d::Zero();
    for (int i = 0; i < std::min(4, flat.cols); ++i) {
        out(i) = flat.at<double>(0, i);
    }
    return out;
}

template <typename T>
std::vector<T> subsample_entries(const std::vector<T>& entries, size_t cap_n) {
    if (entries.size() <= cap_n) return entries;
    std::vector<T> out;
    out.reserve(cap_n);
    for (size_t k = 0; k < cap_n; ++k) {
        size_t idx = static_cast<size_t>(std::llround(
            static_cast<double>(k) * static_cast<double>(entries.size() - 1) / static_cast<double>(cap_n - 1)));
        out.push_back(entries[idx]);
    }
    return out;
}

double median_val(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t mid = v.size() / 2;
    return (v.size() % 2 == 1) ? v[mid] : 0.5 * (v[mid - 1] + v[mid]);
}

double stddev_col(const std::vector<Eigen::Vector3d>& data, int col) {
    if (data.empty()) return 0.0;
    double mean = 0.0;
    for (const auto& v : data) mean += v(col);
    mean /= static_cast<double>(data.size());
    double var = 0.0;
    for (const auto& v : data) {
        double d = v(col) - mean;
        var += d * d;
    }
    return std::sqrt(var / static_cast<double>(data.size()));
}

double median_axis_std(const std::vector<Eigen::Vector3d>& data) {
    return median_val({stddev_col(data, 0), stddev_col(data, 1), stddev_col(data, 2)});
}

std::vector<double> reprojection_errors(
    const std::vector<std::vector<cv::Point3f>>& objs,
    const std::vector<std::vector<cv::Point2f>>& imgs,
    const std::vector<cv::Mat>& rvecs,
    const std::vector<cv::Mat>& tvecs,
    const cv::Mat& K,
    const cv::Mat& D) {
    std::vector<double> errs;
    errs.reserve(objs.size());
    for (size_t i = 0; i < objs.size(); ++i) {
        std::vector<cv::Point2f> proj;
        cv::projectPoints(objs[i], rvecs[i], tvecs[i], K, D, proj);
        double sq = 0.0;
        for (size_t j = 0; j < proj.size(); ++j) {
            cv::Point2f d = proj[j] - imgs[i][j];
            sq += static_cast<double>(d.x * d.x + d.y * d.y);
        }
        errs.push_back(std::sqrt(sq / static_cast<double>(std::max<size_t>(1, proj.size()))));
    }
    return errs;
}

bool match_common_corners(
    const std::vector<cv::Point3f>& o0,
    const std::vector<cv::Point2f>& i0,
    const std::vector<cv::Point3f>& o1,
    const std::vector<cv::Point2f>& i1,
    std::vector<cv::Point3f>& co,
    std::vector<cv::Point2f>& c0,
    std::vector<cv::Point2f>& c1) {
    co.clear(); c0.clear(); c1.clear();
    if (o0.size() == o1.size()) {
        bool same = true;
        for (size_t k = 0; k < o0.size(); ++k) {
            if (std::abs(o0[k].x - o1[k].x) > 1e-5f ||
                std::abs(o0[k].y - o1[k].y) > 1e-5f ||
                std::abs(o0[k].z - o1[k].z) > 1e-5f) {
                same = false;
                break;
            }
        }
        if (same) {
            co = o0; c0 = i0; c1 = i1;
            return co.size() >= 8;
        }
    }
    auto key_of = [](const cv::Point3f& p) {
        return std::pair<long long, long long>{
            std::llround(static_cast<double>(p.x) * 1e6),
            std::llround(static_cast<double>(p.y) * 1e6)
        };
    };
    std::map<std::pair<long long, long long>, cv::Point2f> map1;
    for (size_t k = 0; k < std::min(o1.size(), i1.size()); ++k) {
        map1[key_of(o1[k])] = i1[k];
    }
    for (size_t k = 0; k < std::min(o0.size(), i0.size()); ++k) {
        auto it = map1.find(key_of(o0[k]));
        if (it != map1.end()) {
            co.push_back(o0[k]);
            c0.push_back(i0[k]);
            c1.push_back(it->second);
        }
    }
    return co.size() >= 8;
}

bool solve_pose_4x4(
    const std::vector<cv::Point3f>& obj,
    const std::vector<cv::Point2f>& img,
    const Eigen::Matrix3d& K_eig,
    const Eigen::Vector4d& D_vec,
    Eigen::Matrix4d& T_out) {
    cv::Mat K = mat_eigen_to_cv(K_eig);
    cv::Mat D = dist4_to_cv5(D_vec);
    cv::Mat rvec, tvec;
    if (!cv::solvePnP(obj, img, K, D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) return false;
    std::vector<cv::Point2f> proj;
    cv::projectPoints(obj, rvec, tvec, K, D, proj);
    double sq = 0.0;
    for (size_t i = 0; i < proj.size(); ++i) {
        cv::Point2f d = proj[i] - img[i];
        sq += static_cast<double>(d.x * d.x + d.y * d.y);
    }
    if (std::sqrt(sq / static_cast<double>(proj.size())) > 3.5) return false;
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    T_out = Eigen::Matrix4d::Identity();
    T_out.topLeftCorner<3, 3>() = mat_cv_to_eigen(R_cv);
    cv::Mat tv;
    tvec.convertTo(tv, CV_64F);
    T_out(0, 3) = tv.at<double>(0);
    T_out(1, 3) = tv.at<double>(1);
    T_out(2, 3) = tv.at<double>(2);
    return true;
}

struct FramePose {
    int64_t ts_ns;
    Eigen::Matrix3d R_wc;
    Eigen::Vector3d p_wc;
};

std::vector<FramePose> solve_frame_poses(
    const std::vector<Observation>& obs_list,
    const Eigen::Matrix3d& K0,
    const Eigen::Vector4d& D0,
    int cam_idx) {
    cv::Mat K = mat_eigen_to_cv(K0);
    cv::Mat D = dist4_to_cv5(D0);
    std::vector<FramePose> poses;
    for (const auto& obs : obs_list) {
        auto it_o = obs.obj_pts.find(cam_idx);
        auto it_i = obs.img_pts.find(cam_idx);
        if (it_o == obs.obj_pts.end() || it_i == obs.img_pts.end() || it_o->second.size() < 6) continue;
        cv::Mat rvec, tvec;
        if (!cv::solvePnP(it_o->second, it_i->second, K, D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) continue;
        std::vector<cv::Point2f> proj;
        cv::projectPoints(it_o->second, rvec, tvec, K, D, proj);
        double sq = 0.0;
        for (size_t k = 0; k < proj.size(); ++k) {
            cv::Point2f d = proj[k] - it_i->second[k];
            sq += static_cast<double>(d.x * d.x + d.y * d.y);
        }
        if (std::sqrt(sq / static_cast<double>(proj.size())) > 3.0) continue;
        cv::Mat R_cw_cv, tv;
        cv::Rodrigues(rvec, R_cw_cv);
        tvec.convertTo(tv, CV_64F);
        Eigen::Matrix3d R_wc = mat_cv_to_eigen(R_cw_cv).transpose();
        Eigen::Vector3d t_cw(tv.at<double>(0), tv.at<double>(1), tv.at<double>(2));
        Eigen::Vector3d p_wc = -(R_wc * t_cw);
        poses.push_back({obs.ts_ns, R_wc, p_wc});
    }
    return poses;
}

double interp_1d(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
    if (xs.empty()) return 0.0;
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    auto it = std::lower_bound(xs.begin(), xs.end(), x);
    size_t idx = static_cast<size_t>(std::distance(xs.begin(), it));
    if (idx == 0) return ys[0];
    double x0 = xs[idx - 1], x1 = xs[idx];
    double alpha = (x1 > x0) ? (x - x0) / (x1 - x0) : 0.0;
    return ys[idx - 1] * (1.0 - alpha) + ys[idx] * alpha;
}

Eigen::Matrix4d json_to_matrix4d(const JsonValue& arr) {
    if (!arr.is_array() || arr.as_array().size() != 4) {
        throw std::invalid_argument("Expected 4x4 matrix array in YAML");
    }
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    for (size_t r = 0; r < 4; ++r) {
        const auto& row = arr.as_array()[r].as_array();
        if (row.size() != 4) throw std::invalid_argument("Expected 4 columns in matrix row");
        for (size_t c = 0; c < 4; ++c) m(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = row[c].as_double();
    }
    return m;
}

std::string fmt14(double v) {
    std::ostringstream os;
    os << std::setprecision(14) << v;
    return os.str();
}

void append_opencv_matrix(std::vector<std::string>& lines, const std::string& name, const Eigen::Matrix4d& mat) {
    lines.push_back(name + ": !!opencv-matrix");
    lines.push_back(" rows: 4");
    lines.push_back(" cols: 4");
    lines.push_back(" dt: f");
    std::ostringstream os;
    os << " data: [";
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (r > 0 || c > 0) os << ", ";
            os << fmt14(mat(r, c));
        }
    }
    os << "]";
    lines.push_back(os.str());
}

std::vector<std::string> split_ws_or_csv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) out.push_back(trim(item));
    return out;
}

std::vector<int64_t> read_video_timestamps(
    const std::filesystem::path& source_dir,
    int frame_count,
    double fallback_fps) {
    std::filesystem::path csv_path = source_dir / "umi_video_timestamps.csv";
    if (std::filesystem::is_regular_file(csv_path)) {
        std::ifstream f(csv_path);
        std::string header;
        if (!std::getline(f, header)) throw std::invalid_argument("Empty umi_video_timestamps.csv");
        auto cols = split_ws_or_csv(header);
        int ts_col = -1;
        for (size_t i = 0; i < cols.size(); ++i) {
            if (cols[i] == "driver_ts_ns") ts_col = static_cast<int>(i);
        }
        if (ts_col < 0) throw std::invalid_argument("Missing driver_ts_ns column");
        std::vector<int64_t> ts;
        std::string line;
        while (std::getline(f, line)) {
            if (trim(line).empty()) continue;
            auto row = split_ws_or_csv(line);
            if (ts_col < static_cast<int>(row.size())) ts.push_back(slam_common::integer_ns(row[ts_col]));
        }
        return ts;
    }
    std::filesystem::path txt_path = source_dir / "timestamps.txt";
    if (std::filesystem::is_regular_file(txt_path)) {
        std::ifstream f(txt_path);
        std::vector<int64_t> ts;
        std::string line;
        while (std::getline(f, line)) {
            std::string s = trim(line);
            if (!s.empty() && s.front() != '#') ts.push_back(slam_common::integer_ns(s));
        }
        return ts;
    }
    if (frame_count > 1) {
        int64_t dt_ns = static_cast<int64_t>(std::llround(1e9 / std::max(1.0, fallback_fps)));
        std::vector<int64_t> ts;
        ts.reserve(static_cast<size_t>(frame_count));
        for (int k = 0; k < frame_count; ++k) ts.push_back(static_cast<int64_t>(k) * dt_ns);
        return ts;
    }
    throw std::invalid_argument("目录中缺少 umi_video_timestamps.csv 或 timestamps.txt");
}

bool read_source_imu(
    const std::filesystem::path& source_dir,
    double imu_hz,
    double accel_g,
    double gyro_dps,
    slam_common::ImuSeries& out) {
    for (const char* cand : {"umi_imu_timestamps.bin", "imu.bin"}) {
        if (std::filesystem::is_regular_file(source_dir / cand)) {
            out = slam_common::decode_ms901m_bin(source_dir / cand, imu_hz, accel_g, gyro_dps);
            return true;
        }
    }
    for (const char* cand : {"mav0/imu0/data.csv", "imu0/data.csv", "imu.csv"}) {
        std::filesystem::path p = source_dir / std::filesystem::u8path(cand);
        if (std::filesystem::is_regular_file(p)) {
            out = slam_common::read_imu_csv(p);
            return true;
        }
    }
    return false;
}

template <typename T>
std::vector<T> select_span_samples(const std::vector<T>& seq, int stride, int max_frames) {
    if (seq.empty()) return {};
    if (max_frames <= 0 || seq.size() <= static_cast<size_t>(max_frames) * static_cast<size_t>(stride)) {
        std::vector<T> chosen;
        for (size_t i = 0; i < seq.size(); i += static_cast<size_t>(stride)) {
            chosen.push_back(seq[i]);
            if (max_frames > 0 && chosen.size() >= static_cast<size_t>(max_frames)) break;
        }
        return chosen;
    }
    int n_triplets = std::max(4, max_frames / 3);
    int max_start = std::max(0, static_cast<int>(seq.size()) - 1 - 2 * stride);
    std::set<size_t> picked_idx;
    for (int k = 0; k < n_triplets; ++k) {
        int anchor = (n_triplets > 1)
            ? static_cast<int>(std::llround(static_cast<double>(k) * max_start / static_cast<double>(n_triplets - 1)))
            : 0;
        for (int off : {0, stride, 2 * stride}) {
            size_t idx = static_cast<size_t>(anchor + off);
            if (idx < seq.size()) picked_idx.insert(idx);
        }
    }
    std::vector<T> out;
    for (size_t idx : picked_idx) {
        out.push_back(seq[idx]);
        if (out.size() >= static_cast<size_t>(max_frames)) break;
    }
    return out;
}

bool is_image_ext(const std::filesystem::path& p) {
    std::string ext = trim_lower(p.extension().string());
    return ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || ext == ".tiff" || ext == ".tif";
}

bool is_video_ext(const std::filesystem::path& p) {
    std::string ext = trim_lower(p.extension().string());
    return ext == ".mkv" || ext == ".mp4" || ext == ".avi" || ext == ".mov";
}

std::map<int64_t, std::filesystem::path> discover_image_folder_times(const std::filesystem::path& cam_dir) {
    std::vector<std::filesystem::path> paths;
    for (const auto& entry : std::filesystem::directory_iterator(cam_dir)) {
        if (entry.is_regular_file() && is_image_ext(entry.path())) paths.push_back(entry.path());
    }
    std::sort(paths.begin(), paths.end());
    std::map<int64_t, std::filesystem::path> files;
    for (size_t idx = 0; idx < paths.size(); ++idx) {
        int64_t ts = static_cast<int64_t>(idx) * 33333333LL;
        try {
            ts = slam_common::integer_ns(paths[idx].stem().string());
        } catch (...) {
        }
        files[ts] = paths[idx];
    }
    return files;
}

std::vector<std::filesystem::path> discover_multicam_dirs(
    const std::filesystem::path& src,
    const std::vector<std::filesystem::path>& explicit_dirs) {
    if (!explicit_dirs.empty()) {
        std::vector<std::filesystem::path> resolved;
        for (const auto& p : explicit_dirs) {
            std::filesystem::path cp = std::filesystem::weakly_canonical(p);
            std::filesystem::path data_sub = std::filesystem::is_directory(cp / "data") ? (cp / "data") : cp;
            if (!std::filesystem::is_directory(data_sub)) {
                throw std::invalid_argument("相机图像目录不存在: " + p.string());
            }
            resolved.push_back(data_sub);
        }
        return resolved;
    }
    std::filesystem::path mav = std::filesystem::is_directory(src / "mav0") ? (src / "mav0") : src;
    std::vector<std::pair<int, std::filesystem::path>> cam_dirs;
    static const std::regex kCamRe(R"(^cam(\d+)$)");
    if (std::filesystem::is_directory(mav)) {
        for (const auto& entry : std::filesystem::directory_iterator(mav)) {
            if (!entry.is_directory()) continue;
            std::string name = entry.path().filename().string();
            std::smatch m;
            if (std::regex_match(name, m, kCamRe)) {
                std::filesystem::path d = std::filesystem::is_directory(entry.path() / "data")
                    ? (entry.path() / "data")
                    : entry.path();
                if (std::filesystem::is_directory(d)) {
                    cam_dirs.emplace_back(std::stoi(m[1].str()), d);
                }
            }
        }
    }
    std::sort(cam_dirs.begin(), cam_dirs.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    std::vector<std::filesystem::path> out;
    for (const auto& kv : cam_dirs) out.push_back(kv.second);
    return out;
}

/// Self-contained builtin detector (does NOT import calibration_targets to keep zero cross-package dependency).
DetectCallback make_builtin_detector(
    const std::string& target_type,
    int rows,
    int cols,
    double tag_size_m,
    double tag_spacing,
    const ProgressCallback& log_fn) {
    std::string kind = trim_lower(target_type);
    if (kind == "aprilgrid" || kind == "april" || kind == "apriltag") {
        auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        cv::aruco::DetectorParameters params;
        params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
        cv::aruco::ArucoDetector detector(dict, params);
        struct PermState {
            bool found = false;
            bool bu = false;
            bool rev = false;
            int roll = 2;
            double expand = 7.0 / 6.0;
        };
        auto perm_state = std::make_shared<PermState>();

        auto refine_quads = [](const cv::Mat& gray,
                               const std::vector<std::vector<cv::Point2f>>& raw_q,
                               double expand) {
            float max_x = std::max(1.0f, static_cast<float>(gray.cols - 2));
            float max_y = std::max(1.0f, static_cast<float>(gray.rows - 2));
            std::vector<cv::Point2f> flat;
            flat.reserve(raw_q.size() * 4);
            for (const auto& q : raw_q) {
                cv::Point2f c(0.0f, 0.0f);
                for (int k = 0; k < 4; ++k) c += q[k];
                c *= 0.25f;
                for (int k = 0; k < 4; ++k) {
                    cv::Point2f pt = (std::abs(expand - 1.0) > 1e-6) ? (c + (q[k] - c) * static_cast<float>(expand)) : q[k];
                    pt.x = std::max(1.0f, std::min(max_x, pt.x));
                    pt.y = std::max(1.0f, std::min(max_y, pt.y));
                    flat.push_back(pt);
                }
            }
            cv::TermCriteria term(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 1e-4);
            cv::cornerSubPix(gray, flat, cv::Size(5, 5), cv::Size(-1, -1), term);
            return flat;
        };

        return [detector, perm_state, refine_quads, rows, cols, tag_size_m, tag_spacing, log_fn](
                   const cv::Mat& gray, int) mutable -> DetectedCorners {
            std::vector<std::vector<cv::Point2f>> corners, rejected;
            std::vector<int> ids;
            detector.detectMarkers(gray, corners, ids, rejected);
            if (ids.empty()) return {};
            int max_id = rows * cols;
            if (!perm_state->found) {
                std::vector<std::vector<cv::Point2f>> raw_q;
                std::vector<int> tids;
                for (size_t i = 0; i < std::min(corners.size(), ids.size()); ++i) {
                    if (ids[i] >= 0 && ids[i] < max_id) {
                        raw_q.push_back(corners[i]);
                        tids.push_back(ids[i]);
                    }
                }
                if (tids.size() >= 4) {
                    double best_e = 1e18;
                    PermState best;
                    for (double expand : {7.0 / 6.0, 1.0}) {
                        auto ref = refine_quads(gray, raw_q, expand);
                        for (bool bu : {false, true}) {
                            for (bool rev : {false, true}) {
                                for (int roll = 0; roll < 4; ++roll) {
                                    std::vector<cv::Point2f> objs_2d;
                                    for (int tid : tids) {
                                        auto c3 = slam_common::aprilgrid_tag_corners_3d(
                                            tid, rows, cols, tag_size_m, tag_spacing, bu, rev, roll);
                                        for (const auto& p : c3) objs_2d.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()));
                                    }
                                    cv::Mat H = cv::findHomography(objs_2d, ref, 0);
                                    if (H.empty()) continue;
                                    std::vector<cv::Point2f> proj;
                                    cv::perspectiveTransform(objs_2d, proj, H);
                                    double sq = 0.0;
                                    for (size_t k = 0; k < ref.size(); ++k) {
                                        cv::Point2f d = proj[k] - ref[k];
                                        sq += static_cast<double>(d.x * d.x + d.y * d.y);
                                    }
                                    double err = std::sqrt(sq / static_cast<double>(ref.size()));
                                    if (err < best_e) {
                                        best_e = err;
                                        best = {true, bu, rev, roll, expand};
                                    }
                                }
                            }
                        }
                    }
                    if (best.found && best_e < 25.0) {
                        *perm_state = best;
                        if (log_fn) {
                            std::ostringstream msg;
                            msg << "自动匹配 AprilGrid 角点序: bottom_up=" << (best.bu ? "true" : "false")
                                << ", rev=" << (best.rev ? "true" : "false")
                                << ", roll=" << best.roll
                                << ", expand=" << std::fixed << std::setprecision(3) << best.expand;
                            log_fn(msg.str());
                        }
                    }
                }
            }
            std::vector<std::vector<cv::Point2f>> quads;
            std::vector<int> tids;
            std::set<int> seen;
            for (size_t i = 0; i < std::min(corners.size(), ids.size()); ++i) {
                int tid = ids[i];
                if (tid >= 0 && tid < max_id && !seen.count(tid)) {
                    seen.insert(tid);
                    quads.push_back(corners[i]);
                    tids.push_back(tid);
                }
            }
            if (seen.size() < 2) return {};
            auto ref = refine_quads(gray, quads, perm_state->expand);
            DetectedCorners out;
            out.found = true;
            out.img = std::move(ref);
            for (int tid : tids) {
                auto c3 = slam_common::aprilgrid_tag_corners_3d(
                    tid, rows, cols, tag_size_m, tag_spacing, perm_state->bu, perm_state->rev, perm_state->roll);
                for (const auto& p : c3) {
                    out.obj.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
                }
            }
            return out;
        };
    }

    if (kind == "circles" || kind == "acircles" || kind == "circle" ||
        kind == "symmetric_circles" || kind == "asymmetric_circles") {
        bool asym = (kind == "acircles" || kind == "asymmetric_circles");
        int base_flag = asym ? cv::CALIB_CB_ASYMMETRIC_GRID : cv::CALIB_CB_SYMMETRIC_GRID;
        std::vector<cv::Point3f> obj_template;
        obj_template.reserve(static_cast<size_t>(rows * cols));
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                double x = asym ? ((2.0 * c + (r % 2)) * 0.5 * tag_size_m) : (c * tag_size_m);
                double y = asym ? (r * 0.5 * tag_size_m) : (r * tag_size_m);
                obj_template.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0f);
            }
        }
        return [asym, base_flag, obj_template, rows, cols](const cv::Mat& gray, int) -> DetectedCorners {
            double area = static_cast<double>(gray.rows) * static_cast<double>(gray.cols);
            for (int bcolor : {0, 255}) {
                cv::SimpleBlobDetector::Params p;
                p.filterByColor = true;
                p.blobColor = static_cast<uchar>(bcolor);
                p.filterByCircularity = true;
                p.minCircularity = 0.65f;
                p.filterByArea = true;
                p.minArea = 12.0f;
                p.maxArea = static_cast<float>(std::max(25000.0, area * 0.2));
                p.filterByInertia = true;
                p.minInertiaRatio = 0.2f;
                p.filterByConvexity = true;
                p.minConvexity = 0.55f;
                auto det = cv::SimpleBlobDetector::create(p);
                for (int flag : {base_flag, base_flag | cv::CALIB_CB_CLUSTERING}) {
                    std::vector<cv::Point2f> pts;
                    if (cv::findCirclesGrid(gray, cv::Size(cols, rows), pts, flag, det) && !pts.empty()) {
                        return {true, obj_template, pts};
                    }
                }
            }
            return {};
        };
    }

    return [rows, cols, tag_size_m](const cv::Mat& gray, int) -> DetectedCorners {
        cv::Size pattern(cols, rows);
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCornersSB(
            gray, pattern, corners, cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE);
        if (!found || corners.empty()) {
            found = cv::findChessboardCorners(
                gray, pattern, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
            if (found && !corners.empty()) {
                cv::TermCriteria term(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 1e-3);
                cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), term);
            }
        } else {
            cv::TermCriteria term(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 1e-4);
            cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), term);
        }
        if (!found || corners.empty()) return {};
        std::vector<cv::Point3f> objp;
        objp.reserve(static_cast<size_t>(rows * cols));
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                objp.emplace_back(static_cast<float>(c * tag_size_m), static_cast<float>(r * tag_size_m), 0.0f);
            }
        }
        return {true, objp, corners};
    };
}

}  // namespace

JsonValue parse_simple_yaml(const std::string& text) {
    std::vector<YamlLine> lines;
    std::istringstream ss(text);
    std::string raw;
    while (std::getline(ss, raw)) {
        std::string s = strip_yaml_comment(raw);
        if (trim(s).empty()) continue;
        int indent = 0;
        while (indent < static_cast<int>(s.size()) && s[indent] == ' ') ++indent;
        lines.push_back({indent, trim(s)});
    }
    size_t idx = 0;
    JsonValue res = parse_yaml_block(lines, idx, 0);
    return res.is_object() ? res : JsonValue::object();
}

MonoCalibrationResult calibrate_monocular_from_observations(
    const std::vector<Observation>& obs_list,
    cv::Size image_size,
    int cam_idx) {
    struct Entry {
        size_t idx;
        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> img;
    };
    std::vector<Entry> entries;
    for (size_t idx = 0; idx < obs_list.size(); ++idx) {
        const auto& obs = obs_list[idx];
        auto it_o = obs.obj_pts.find(cam_idx);
        auto it_i = obs.img_pts.find(cam_idx);
        if (it_o != obs.obj_pts.end() && it_i != obs.img_pts.end() && it_o->second.size() >= 4) {
            entries.push_back({idx, it_o->second, it_i->second});
        }
    }
    if (entries.size() < 8) {
        throw std::invalid_argument("相机 cam" + std::to_string(cam_idx) +
                                    " 有效标定帧不足（仅 " + std::to_string(entries.size()) +
                                    " 帧，至少需 8 帧清晰靶标图像）");
    }

    auto sub_obs = subsample_entries(entries, 80);
    std::vector<std::vector<cv::Point3f>> objs;
    std::vector<std::vector<cv::Point2f>> imgs;
    for (const auto& e : sub_obs) {
        objs.push_back(e.obj);
        imgs.push_back(e.img);
    }

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(objs, imgs, image_size, K, D, rvecs, tvecs, cv::CALIB_FIX_K3);

    auto per_err = reprojection_errors(objs, imgs, rvecs, tvecs, K, D);
    double thresh = std::max(2.0, median_val(per_err) * 2.5);
    std::vector<Entry> filtered;
    for (size_t i = 0; i < sub_obs.size(); ++i) {
        if (per_err[i] <= thresh) filtered.push_back(sub_obs[i]);
    }
    if (filtered.size() >= 8 && filtered.size() < sub_obs.size()) {
        objs.clear(); imgs.clear();
        for (const auto& e : filtered) {
            objs.push_back(e.obj);
            imgs.push_back(e.img);
        }
        rvecs.clear(); tvecs.clear();
        rms = cv::calibrateCamera(
            objs, imgs, image_size, K, D, rvecs, tvecs,
            cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_K3);
    } else {
        filtered = sub_obs;
    }

    MonoCalibrationResult res;
    res.cam_idx = cam_idx;
    res.K = mat_cv_to_eigen(K);
    res.D = cv_dist_to_vec4(D);
    res.mono_rms = rms;
    res.frames = static_cast<int>(filtered.size());
    res.image_size = image_size;
    for (const auto& e : filtered) res.kept_indices.push_back(e.idx);
    return res;
}

StereoCalibrationResult calibrate_stereo_from_observations(
    const std::vector<Observation>& obs_list,
    cv::Size image_size) {
    MonoCalibrationResult m0 = calibrate_monocular_from_observations(obs_list, image_size, 0);
    MonoCalibrationResult m1 = calibrate_monocular_from_observations(obs_list, image_size, 1);

    std::set<size_t> kept0(m0.kept_indices.begin(), m0.kept_indices.end());
    std::set<size_t> kept_stereo;
    for (size_t idx : m1.kept_indices) {
        if (kept0.count(idx)) kept_stereo.insert(idx);
    }

    struct StereoEntry {
        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> i0;
        std::vector<cv::Point2f> i1;
    };
    std::vector<StereoEntry> pairs;
    for (size_t idx : kept_stereo) {
        const auto& obs = obs_list[idx];
        auto o0 = obs.obj_pts.find(0), o1 = obs.obj_pts.find(1);
        auto p0 = obs.img_pts.find(0), p1 = obs.img_pts.find(1);
        if (o0 == obs.obj_pts.end() || o1 == obs.obj_pts.end() ||
            p0 == obs.img_pts.end() || p1 == obs.img_pts.end()) {
            continue;
        }
        std::vector<cv::Point3f> co;
        std::vector<cv::Point2f> c0, c1;
        if (match_common_corners(o0->second, p0->second, o1->second, p1->second, co, c0, c1)) {
            pairs.push_back({std::move(co), std::move(c0), std::move(c1)});
        }
    }
    if (pairs.size() < 6) {
        throw std::invalid_argument("双目同时观测到足够共同角点的帧不足（仅 " +
                                    std::to_string(pairs.size()) + " 帧，至少需 6 帧）");
    }
    pairs = subsample_entries(pairs, 80);

    std::vector<std::vector<cv::Point3f>> s_objs;
    std::vector<std::vector<cv::Point2f>> s_img0, s_img1;
    for (const auto& p : pairs) {
        s_objs.push_back(p.obj);
        s_img0.push_back(p.i0);
        s_img1.push_back(p.i1);
    }

    cv::Mat K0 = mat_eigen_to_cv(m0.K), D0 = dist4_to_cv5(m0.D);
    cv::Mat K1 = mat_eigen_to_cv(m1.K), D1 = dist4_to_cv5(m1.D);
    cv::Mat R_21, t_21, E, F;
    double stereo_rms = cv::stereoCalibrate(
        s_objs, s_img0, s_img1, K0, D0, K1, D1, image_size, R_21, t_21, E, F,
        cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_K3,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 1e-6));

    cv::Mat tv;
    t_21.convertTo(tv, CV_64F);
    Eigen::Matrix4d T_c2_c1 = Eigen::Matrix4d::Identity();
    T_c2_c1.topLeftCorner<3, 3>() = slam_common::project_so3(mat_cv_to_eigen(R_21));
    T_c2_c1(0, 3) = tv.at<double>(0);
    T_c2_c1(1, 3) = tv.at<double>(1);
    T_c2_c1(2, 3) = tv.at<double>(2);
    Eigen::Matrix4d T_c1_c2 = T_c2_c1.inverse();

    StereoCalibrationResult res;
    res.K0 = mat_cv_to_eigen(K0);
    res.D0 = cv_dist_to_vec4(D0);
    res.K1 = mat_cv_to_eigen(K1);
    res.D1 = cv_dist_to_vec4(D1);
    res.T_c1_c2 = T_c1_c2;
    res.baseline = T_c1_c2.topRightCorner<3, 1>().norm();
    res.mono_rms[0] = m0.mono_rms;
    res.mono_rms[1] = m1.mono_rms;
    res.stereo_rms = stereo_rms;
    res.stereo_frames = static_cast<int>(pairs.size());
    return res;
}

MultiCamCalibrationResult calibrate_multicam_from_observations(
    const std::vector<Observation>& obs_list,
    const std::vector<cv::Size>& image_sizes) {
    int num_cams = static_cast<int>(image_sizes.size());
    if (num_cams < 2) {
        throw std::invalid_argument("多目标定至少需要 2 路相机且每路需指定分辨率");
    }

    std::vector<MonoCalibrationResult> monos;
    monos.reserve(num_cams);
    for (int i = 0; i < num_cams; ++i) {
        monos.push_back(calibrate_monocular_from_observations(obs_list, image_sizes[i], i));
    }

    std::vector<std::map<int, Eigen::Matrix4d>> adj(num_cams);
    JsonValue pair_stats = JsonValue::object();

    for (int i = 0; i < num_cams; ++i) {
        std::set<size_t> kept_i(monos[i].kept_indices.begin(), monos[i].kept_indices.end());
        for (int j = i + 1; j < num_cams; ++j) {
            struct PairItem {
                std::vector<cv::Point3f> obj;
                std::vector<cv::Point2f> ci;
                std::vector<cv::Point2f> cj;
            };
            std::vector<PairItem> common;
            for (size_t idx : monos[j].kept_indices) {
                if (!kept_i.count(idx)) continue;
                const auto& obs = obs_list[idx];
                auto oi = obs.obj_pts.find(i), oj = obs.obj_pts.find(j);
                auto pi = obs.img_pts.find(i), pj = obs.img_pts.find(j);
                if (oi == obs.obj_pts.end() || oj == obs.obj_pts.end() ||
                    pi == obs.img_pts.end() || pj == obs.img_pts.end()) {
                    continue;
                }
                std::vector<cv::Point3f> co;
                std::vector<cv::Point2f> ci, cj;
                if (match_common_corners(oi->second, pi->second, oj->second, pj->second, co, ci, cj)) {
                    common.push_back({std::move(co), std::move(ci), std::move(cj)});
                }
            }
            if (common.size() < 6) continue;
            common = subsample_entries(common, 60);

            Eigen::Matrix4d T_ci_cj = Eigen::Matrix4d::Identity();
            double rms_ij = 0.0;
            if (image_sizes[i] == image_sizes[j]) {
                std::vector<std::vector<cv::Point3f>> c_objs;
                std::vector<std::vector<cv::Point2f>> c_img_i, c_img_j;
                for (const auto& it : common) {
                    c_objs.push_back(it.obj);
                    c_img_i.push_back(it.ci);
                    c_img_j.push_back(it.cj);
                }
                cv::Mat Ki = mat_eigen_to_cv(monos[i].K), Di = dist4_to_cv5(monos[i].D);
                cv::Mat Kj = mat_eigen_to_cv(monos[j].K), Dj = dist4_to_cv5(monos[j].D);
                cv::Mat R_ji, t_ji, E, F;
                rms_ij = cv::stereoCalibrate(
                    c_objs, c_img_i, c_img_j, Ki, Di, Kj, Dj, image_sizes[i], R_ji, t_ji, E, F,
                    cv::CALIB_FIX_INTRINSIC,
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 1e-6));
                cv::Mat tv;
                t_ji.convertTo(tv, CV_64F);
                Eigen::Matrix4d T_cj_ci = Eigen::Matrix4d::Identity();
                T_cj_ci.topLeftCorner<3, 3>() = slam_common::project_so3(mat_cv_to_eigen(R_ji));
                T_cj_ci(0, 3) = tv.at<double>(0);
                T_cj_ci(1, 3) = tv.at<double>(1);
                T_cj_ci(2, 3) = tv.at<double>(2);
                T_ci_cj = T_cj_ci.inverse();
            } else {
                std::vector<Eigen::Matrix4d> rels;
                for (const auto& it : common) {
                    Eigen::Matrix4d Tiw, Tjw;
                    if (solve_pose_4x4(it.obj, it.ci, monos[i].K, monos[i].D, Tiw) &&
                        solve_pose_4x4(it.obj, it.cj, monos[j].K, monos[j].D, Tjw)) {
                        rels.push_back(Tiw * Tjw.inverse());
                    }
                }
                if (rels.size() < 6) continue;
                Eigen::Matrix3d R_sum = Eigen::Matrix3d::Zero();
                std::vector<double> tx, ty, tz;
                for (const auto& T : rels) {
                    R_sum += T.topLeftCorner<3, 3>();
                    tx.push_back(T(0, 3));
                    ty.push_back(T(1, 3));
                    tz.push_back(T(2, 3));
                }
                T_ci_cj.topLeftCorner<3, 3>() = slam_common::project_so3(R_sum);
                T_ci_cj(0, 3) = median_val(tx);
                T_ci_cj(1, 3) = median_val(ty);
                T_ci_cj(2, 3) = median_val(tz);
                rms_ij = 0.5 * (monos[i].mono_rms + monos[j].mono_rms);
            }

            adj[i][j] = T_ci_cj;
            adj[j][i] = T_ci_cj.inverse();

            JsonValue stat = JsonValue::object();
            stat["frames"] = static_cast<int64_t>(common.size());
            stat["pair_rms_px"] = rms_ij;
            stat["baseline_m"] = T_ci_cj.topRightCorner<3, 1>().norm();
            JsonValue mat_json = JsonValue::array();
            for (int r = 0; r < 4; ++r) {
                JsonValue row = JsonValue::array();
                for (int c = 0; c < 4; ++c) row.push_back(T_ci_cj(r, c));
                mat_json.push_back(row);
            }
            stat["T_ci_cj"] = mat_json;
            pair_stats["cam" + std::to_string(i) + "-cam" + std::to_string(j)] = stat;
        }
    }

    std::map<int, Eigen::Matrix4d> T_c0_map;
    T_c0_map[0] = Eigen::Matrix4d::Identity();
    std::deque<int> q;
    q.push_back(0);
    while (!q.empty()) {
        int u = q.front();
        q.pop_front();
        for (const auto& kv : adj[u]) {
            int v = kv.first;
            if (!T_c0_map.count(v)) {
                T_c0_map[v] = slam_common::rigid_matrix(T_c0_map[u] * kv.second);
                q.push_back(v);
            }
        }
    }

    for (int i = 0; i < num_cams; ++i) {
        if (!T_c0_map.count(i)) {
            throw std::invalid_argument("多目相机拓扑未完全连通：cam" + std::to_string(i) +
                                        " 与 cam0 之间缺乏足够的共同视野帧");
        }
    }

    MultiCamCalibrationResult res;
    res.num_cams = num_cams;
    res.pair_stats = pair_stats;
    for (int i = 0; i < num_cams; ++i) {
        MultiCamEntry entry;
        entry.cam_idx = i;
        entry.K = monos[i].K;
        entry.D = monos[i].D;
        entry.image_size = image_sizes[i];
        entry.mono_rms = monos[i].mono_rms;
        entry.frames = monos[i].frames;
        entry.T_c0_ci = T_c0_map[i];
        entry.baseline_to_c0_m = entry.T_c0_ci.topRightCorner<3, 1>().norm();
        res.cameras.push_back(entry);
    }
    return res;
}

ImuStandaloneStats calibrate_imu_standalone(
    const std::vector<int64_t>& imu_times_ns,
    const std::vector<Eigen::Matrix<double, 6, 1>>& imu_gyro_acc,
    double imu_hz) {
    if (imu_times_ns.size() < 20 || imu_gyro_acc.size() != imu_times_ns.size()) {
        throw std::invalid_argument("IMU 有效样本太少（至少需要 20 条 6 轴测量值）");
    }
    std::vector<double> pos_dts;
    for (size_t i = 1; i < imu_times_ns.size(); ++i) {
        double dt = static_cast<double>(imu_times_ns[i] - imu_times_ns[i - 1]) * 1e-9;
        if (dt > 0.0) pos_dts.push_back(dt);
    }
    if (pos_dts.empty()) {
        throw std::invalid_argument("IMU 时间戳未单调递增");
    }
    double med_dt = median_val(pos_dts);
    double measured_hz = (imu_hz > 0.0) ? imu_hz : std::round((1.0 / med_dt) * 1000.0) / 1000.0;

    std::vector<double> g_norms(imu_gyro_acc.size()), a_norms(imu_gyro_acc.size());
    for (size_t i = 0; i < imu_gyro_acc.size(); ++i) {
        g_norms[i] = imu_gyro_acc[i].head<3>().norm();
        a_norms[i] = imu_gyro_acc[i].tail<3>().norm();
    }
    double a_med = median_val(a_norms);
    std::vector<size_t> static_idx;
    for (size_t i = 0; i < imu_gyro_acc.size(); ++i) {
        if (g_norms[i] < 0.08 && std::abs(a_norms[i] - a_med) < 0.8) {
            static_idx.push_back(i);
        }
    }
    int static_count = static_cast<int>(static_idx.size());
    if (static_idx.size() < 30) {
        std::vector<size_t> all_idx(imu_gyro_acc.size());
        std::iota(all_idx.begin(), all_idx.end(), 0);
        std::sort(all_idx.begin(), all_idx.end(), [&](size_t a, size_t b) { return g_norms[a] < g_norms[b]; });
        size_t n_q = std::max<size_t>(20, imu_gyro_acc.size() / 4);
        all_idx.resize(std::min(n_q, all_idx.size()));
        std::sort(all_idx.begin(), all_idx.end());
        static_idx = std::move(all_idx);
    }

    std::vector<Eigen::Vector3d> g_seg, a_seg;
    g_seg.reserve(static_idx.size());
    a_seg.reserve(static_idx.size());
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
    for (size_t idx : static_idx) {
        Eigen::Vector3d g = imu_gyro_acc[idx].head<3>();
        Eigen::Vector3d a = imu_gyro_acc[idx].tail<3>();
        g_seg.push_back(g);
        a_seg.push_back(a);
        gyro_bias += g;
        acc_mean += a;
    }
    gyro_bias /= static_cast<double>(static_idx.size());
    acc_mean /= static_cast<double>(static_idx.size());

    std::vector<Eigen::Vector3d> dg, da;
    for (size_t i = 1; i < g_seg.size(); ++i) {
        dg.push_back(g_seg[i] - g_seg[i - 1]);
        da.push_back(a_seg[i] - a_seg[i - 1]);
    }
    double sigma_g = (dg.size() > 2) ? (median_axis_std(dg) / std::sqrt(2.0)) : 0.0035;
    double sigma_a = (da.size() > 2) ? (median_axis_std(da) / std::sqrt(2.0)) : 0.02;
    double noise_gyro = std::max(1e-5, sigma_g * std::sqrt(med_dt));
    double noise_acc = std::max(1e-4, sigma_a * std::sqrt(med_dt));

    int k_chunks = std::min(10, std::max(2, static_cast<int>(g_seg.size()) / 25));
    int chunk_len = std::max(1, static_cast<int>(g_seg.size()) / k_chunks);
    double gyro_walk = 0.0002;
    double acc_walk = 0.002;
    if (chunk_len >= 5 && k_chunks >= 2) {
        std::vector<Eigen::Vector3d> g_means(k_chunks), a_means(k_chunks);
        for (int k = 0; k < k_chunks; ++k) {
            Eigen::Vector3d gm = Eigen::Vector3d::Zero(), am = Eigen::Vector3d::Zero();
            for (int i = k * chunk_len; i < (k + 1) * chunk_len; ++i) {
                gm += g_seg[i];
                am += a_seg[i];
            }
            g_means[k] = gm / static_cast<double>(chunk_len);
            a_means[k] = am / static_cast<double>(chunk_len);
        }
        std::vector<Eigen::Vector3d> dgm, dam;
        for (int k = 1; k < k_chunks; ++k) {
            dgm.push_back(g_means[k] - g_means[k - 1]);
            dam.push_back(a_means[k] - a_means[k - 1]);
        }
        double win_dt = std::max(static_cast<double>(chunk_len) * med_dt, 1e-3);
        gyro_walk = std::max(1e-6, median_axis_std(dgm) / std::sqrt(win_dt));
        acc_walk = std::max(1e-5, median_axis_std(dam) / std::sqrt(win_dt));
    }

    ImuStandaloneStats out;
    out.measured_hz = measured_hz;
    out.duration_s = static_cast<double>(imu_times_ns.back() - imu_times_ns.front()) * 1e-9;
    out.sample_count = static_cast<int>(imu_times_ns.size());
    out.static_samples = static_count;
    out.gyro_bias_rad_s = gyro_bias;
    out.acc_mean_mps2 = acc_mean;
    out.gravity_norm_mps2 = acc_mean.norm();
    out.noise_gyro = noise_gyro;
    out.noise_acc = noise_acc;
    out.gyro_walk = gyro_walk;
    out.acc_walk = acc_walk;
    return out;
}

CamImuExtrinsicResult estimate_cam_imu_extrinsic(
    const std::vector<Observation>& obs_list,
    const Eigen::Matrix3d& K0,
    const Eigen::Vector4d& D0,
    const std::vector<int64_t>& imu_times_ns,
    const std::vector<Eigen::Matrix<double, 6, 1>>& imu_gyro_acc,
    int cam_idx,
    double max_timeshift_s) {
    if (imu_times_ns.size() < 10 || imu_gyro_acc.size() != imu_times_ns.size()) {
        throw std::invalid_argument("IMU 样本太少，无法进行相机-IMU 外参标定");
    }
    auto poses = solve_frame_poses(obs_list, K0, D0, cam_idx);
    if (poses.size() < 6) {
        throw std::invalid_argument("可用于求解姿态的 cam" + std::to_string(cam_idx) + " 有效帧太少");
    }

    std::vector<double> cam_t_s(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) cam_t_s[i] = static_cast<double>(poses[i].ts_ns) * 1e-9;

    std::vector<double> imu_t_s(imu_times_ns.size());
    std::vector<double> gx(imu_times_ns.size()), gy(imu_times_ns.size()), gz(imu_times_ns.size());
    std::vector<double> ax(imu_times_ns.size()), ay(imu_times_ns.size()), az(imu_times_ns.size());
    std::vector<double> imu_norm_all(imu_times_ns.size());
    for (size_t i = 0; i < imu_times_ns.size(); ++i) {
        imu_t_s[i] = static_cast<double>(imu_times_ns[i]) * 1e-9;
        gx[i] = imu_gyro_acc[i](0); gy[i] = imu_gyro_acc[i](1); gz[i] = imu_gyro_acc[i](2);
        ax[i] = imu_gyro_acc[i](3); ay[i] = imu_gyro_acc[i](4); az[i] = imu_gyro_acc[i](5);
        imu_norm_all[i] = imu_gyro_acc[i].head<3>().norm();
    }

    std::vector<double> mid_t_s;
    std::vector<Eigen::Vector3d> omega_c;
    for (size_t k = 0; k + 1 < poses.size(); ++k) {
        double dt = cam_t_s[k + 1] - cam_t_s[k];
        if (dt <= 1e-4 || dt > 0.35) continue;
        Eigen::Matrix3d dR = poses[k].R_wc.transpose() * poses[k + 1].R_wc;
        cv::Mat rvec;
        cv::Rodrigues(mat_eigen_to_cv(dR), rvec);
        cv::Mat rv;
        rvec.convertTo(rv, CV_64F);
        Eigen::Vector3d w(rv.at<double>(0) / dt, rv.at<double>(1) / dt, rv.at<double>(2) / dt);
        if (w.norm() < 20.0) {
            mid_t_s.push_back(0.5 * (cam_t_s[k] + cam_t_s[k + 1]));
            omega_c.push_back(w);
        }
    }
    if (omega_c.size() < 5) {
        throw std::invalid_argument("相机连续运动角速度片段不足，请缓慢晃动六自由度采集数据");
    }

    std::vector<double> wc_norm(omega_c.size());
    double wc_mean = 0.0;
    for (size_t i = 0; i < omega_c.size(); ++i) {
        wc_norm[i] = omega_c[i].norm();
        wc_mean += wc_norm[i];
    }
    wc_mean /= static_cast<double>(wc_norm.size());
    double wc_var = 0.0;
    for (double v : wc_norm) wc_var += (v - wc_mean) * (v - wc_mean);
    double wc_std = std::sqrt(wc_var / static_cast<double>(wc_norm.size()));

    double search_span_s = std::max(1e-3, max_timeshift_s);
    double step_s = (2.0 * search_span_s) / static_cast<double>(std::max(1, kTimeshiftGridSteps - 1));
    double best_shift = 0.0;
    double best_corr = 0.0;
    bool corr_evaluated = false;

    if (wc_std > 1e-4) {
        best_corr = -1e18;
        for (int s_idx = 0; s_idx < kTimeshiftGridSteps; ++s_idx) {
            double sh = -search_span_s + static_cast<double>(s_idx) * step_s;
            std::vector<double> wc_sub, wb_sub;
            for (size_t i = 0; i < mid_t_s.size(); ++i) {
                double qt = mid_t_s[i] + sh;
                if (qt >= imu_t_s.front() && qt <= imu_t_s.back()) {
                    wc_sub.push_back(wc_norm[i] - wc_mean);
                    wb_sub.push_back(interp_1d(imu_t_s, imu_norm_all, qt));
                }
            }
            if (wc_sub.size() < 5) continue;
            double wb_mean = std::accumulate(wb_sub.begin(), wb_sub.end(), 0.0) / static_cast<double>(wb_sub.size());
            double dot = 0.0, n_wc = 0.0, n_wb = 0.0;
            for (size_t i = 0; i < wc_sub.size(); ++i) {
                double wb_c = wb_sub[i] - wb_mean;
                dot += wc_sub[i] * wb_c;
                n_wc += wc_sub[i] * wc_sub[i];
                n_wb += wb_c * wb_c;
            }
            double denom = std::sqrt(n_wc) * std::sqrt(n_wb);
            double corr = (denom > 1e-9) ? (dot / denom) : 0.0;
            corr_evaluated = true;
            if (corr > best_corr) {
                best_corr = corr;
                best_shift = sh;
            }
        }
        if (!corr_evaluated) best_corr = 0.0;
    }

    bool timeshift_at_bound = corr_evaluated && (std::abs(std::abs(best_shift) - search_span_s) <= step_s * 0.5 + 1e-9);
    bool timeshift_low_corr = (!corr_evaluated) || (best_corr < kMinTimeshiftCorr);

    std::vector<Eigen::Vector3d> Wc_m, Wb_m;
    int valid_cnt = 0;
    for (size_t i = 0; i < mid_t_s.size(); ++i) {
        double qt = mid_t_s[i] + best_shift;
        if (qt < imu_t_s.front() || qt > imu_t_s.back()) continue;
        ++valid_cnt;
        Eigen::Vector3d wb(interp_1d(imu_t_s, gx, qt), interp_1d(imu_t_s, gy, qt), interp_1d(imu_t_s, gz, qt));
        if (omega_c[i].norm() > kMinMovingGyroRadS && wb.norm() > kMinMovingGyroRadS) {
            Wc_m.push_back(omega_c[i]);
            Wb_m.push_back(wb);
        }
    }
    if (valid_cnt < 5) {
        throw std::invalid_argument("IMU 时间范围未能覆盖相机标定帧时间段");
    }
    if (static_cast<int>(Wc_m.size()) < kMinMovingGyroPairs) {
        throw std::invalid_argument("相机与 IMU 同步旋转激励不足（有效旋转片段 " +
                                    std::to_string(Wc_m.size()) + " < " +
                                    std::to_string(kMinMovingGyroPairs) + "），请充分绕三轴旋转采集标定数据");
    }

    std::vector<double> weights(Wc_m.size(), 1.0);
    Eigen::Matrix3d R_b_c1 = Eigen::Matrix3d::Identity();
    for (int iter = 0; iter < 5; ++iter) {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < Wc_m.size(); ++i) {
            H += weights[i] * (Wb_m[i] * Wc_m[i].transpose());
        }
        R_b_c1 = slam_common::project_so3(H);
        std::vector<double> res(Wc_m.size());
        for (size_t i = 0; i < Wc_m.size(); ++i) {
            res[i] = (Wb_m[i] - R_b_c1 * Wc_m[i]).norm();
        }
        double scale = std::max(0.03, median_val(res) * 1.5);
        for (size_t i = 0; i < Wc_m.size(); ++i) {
            weights[i] = std::min(1.0, scale / std::max(res[i], 1e-6));
        }
    }
    double gyro_sq = 0.0;
    for (size_t i = 0; i < Wc_m.size(); ++i) {
        gyro_sq += (Wb_m[i] - R_b_c1 * Wc_m[i]).squaredNorm();
    }
    double gyro_rmse = std::sqrt(gyro_sq / static_cast<double>(Wc_m.size()));

    std::vector<Eigen::Matrix<double, 3, 6>> A_rows;
    std::vector<Eigen::Vector3d> b_rows;
    for (size_t k = 1; k + 1 < poses.size(); ++k) {
        double dt1 = cam_t_s[k] - cam_t_s[k - 1];
        double dt2 = cam_t_s[k + 1] - cam_t_s[k];
        if (dt1 <= 1e-4 || dt2 <= 1e-4 || std::max(dt1, dt2) > 0.25) continue;
        double t_q = cam_t_s[k] + best_shift;
        if (t_q < imu_t_s.front() || t_q > imu_t_s.back()) continue;
        double dt_mid = 0.5 * (dt1 + dt2);
        Eigen::Vector3d p_dd = ((poses[k + 1].p_wc - poses[k].p_wc) / dt2 - (poses[k].p_wc - poses[k - 1].p_wc) / dt1) / dt_mid;
        Eigen::Matrix3d R_dd = ((poses[k + 1].R_wc - poses[k].R_wc) / dt2 - (poses[k].R_wc - poses[k - 1].R_wc) / dt1) / dt_mid;
        Eigen::Vector3d a_b(interp_1d(imu_t_s, ax, t_q), interp_1d(imu_t_s, ay, t_q), interp_1d(imu_t_s, az, t_q));
        Eigen::Matrix3d R_wb = poses[k].R_wc * R_b_c1.transpose();
        Eigen::Vector3d rhs = (R_wb * a_b) - p_dd;
        Eigen::Matrix<double, 3, 6> A_k;
        A_k.leftCols<3>() = -Eigen::Matrix3d::Identity();
        A_k.rightCols<3>() = R_dd;
        A_rows.push_back(A_k);
        b_rows.push_back(rhs);
    }

    int translation_rows = static_cast<int>(A_rows.size());
    bool translation_valid = false;
    Eigen::Vector3d t_b_c1 = Eigen::Vector3d::Zero();
    if (translation_rows >= kMinTranslationRows) {
        Eigen::Matrix<double, 6, 6> ATA = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> ATb = Eigen::Matrix<double, 6, 1>::Zero();
        for (size_t i = 0; i < A_rows.size(); ++i) {
            ATA += A_rows[i].transpose() * A_rows[i];
            ATb += A_rows[i].transpose() * b_rows[i];
        }
        Eigen::Matrix<double, 6, 1> reg_diag;
        reg_diag << 1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-2;
        ATA += reg_diag.asDiagonal();
        Eigen::Matrix<double, 6, 1> sol = ATA.ldlt().solve(ATb);
        Eigen::Vector3d p_c1_b = sol.tail<3>();
        if (p_c1_b.allFinite() && p_c1_b.norm() < kMaxPlausibleLeverArmM) {
            t_b_c1 = -(R_b_c1 * p_c1_b);
            translation_valid = true;
        }
    }

    std::vector<std::string> ext_warnings;
    auto fmt2 = [](double v, bool sign) {
        std::ostringstream os;
        os << std::fixed << std::setprecision(2);
        if (sign && v >= 0.0) os << '+';
        os << v;
        return os.str();
    };
    if (timeshift_at_bound) {
        ext_warnings.push_back(
            "最优时间偏移 (" + fmt2(best_shift * 1e3, true) + " ms) 触及搜索边界 ±" +
            fmt2(search_span_s * 1e3, false) + " ms，真实时钟偏移可能超出搜索范围。");
    }
    if (timeshift_low_corr) {
        std::ostringstream os;
        os << std::fixed << std::setprecision(3) << best_corr;
        ext_warnings.push_back(
            "相机与 IMU 角速度互相关度偏低 (" + os.str() + " < " + fmt2(kMinTimeshiftCorr, false) +
            ")，时间偏移与外参可靠性较低。");
    }
    if (!translation_valid) {
        ext_warnings.push_back(
            "相机-IMU 平移杆臂未能可靠求解（有效加速度帧 " + std::to_string(translation_rows) +
            "），平移分量已回退为零向量。");
    }

    Eigen::Matrix4d T_b_c1 = Eigen::Matrix4d::Identity();
    T_b_c1.topLeftCorner<3, 3>() = R_b_c1;
    T_b_c1.topRightCorner<3, 1>() = t_b_c1;

    CamImuExtrinsicResult res;
    res.T_b_c1 = T_b_c1;
    res.timeshift_ns = static_cast<int64_t>(std::llround(best_shift * 1e9));
    res.gyro_rmse_rad_s = gyro_rmse;
    res.timeshift_corr = best_corr;
    res.timeshift_at_bound = timeshift_at_bound;
    res.timeshift_low_corr = timeshift_low_corr;
    res.translation_valid = translation_valid;
    res.translation_rows = translation_rows;
    res.imu_pairs = static_cast<int>(Wc_m.size());
    res.warnings = std::move(ext_warnings);
    return res;
}

std::pair<std::filesystem::path, std::filesystem::path> write_orbslam_calibration(
    const WriteOrbYamlParams& p) {
    std::filesystem::create_directories(p.output_dir);
    std::string mode = trim_lower(p.sensor_mode.empty() ? "stereo_imu" : p.sensor_mode);
    bool enable_imu = (mode.find("imu") != std::string::npos);

    std::vector<std::string> lines = {"%YAML:1.0", "File.version: \"1.0\""};

    if (mode != "imu_only") {
        if (p.width <= 0 || p.height <= 0) {
            throw std::invalid_argument("相机标定输出缺少主相机内参或图像分辨率");
        }
        lines.push_back("Camera.type: \"PinHole\"");
        struct CamEntry {
            int idx;
            Eigen::Matrix3d K;
            Eigen::Vector4d D;
        };
        std::vector<CamEntry> cams = {{1, p.cam0_K, p.cam0_D}};
        if (p.has_cam1) cams.push_back({2, p.cam1_K, p.cam1_D});
        for (const auto& c : p.extra_cameras) {
            int idx = c.cam_idx + 1;
            if (idx > 2) cams.push_back({idx, c.K, c.D});
        }
        for (const auto& c : cams) {
            std::string prefix = "Camera" + std::to_string(c.idx) + ".";
            lines.push_back(prefix + "fx: " + fmt14(c.K(0, 0)));
            lines.push_back(prefix + "fy: " + fmt14(c.K(1, 1)));
            lines.push_back(prefix + "cx: " + fmt14(c.K(0, 2)));
            lines.push_back(prefix + "cy: " + fmt14(c.K(1, 2)));
            lines.push_back(prefix + "k1: " + fmt14(c.D(0)));
            lines.push_back(prefix + "k2: " + fmt14(c.D(1)));
            lines.push_back(prefix + "p1: " + fmt14(c.D(2)));
            lines.push_back(prefix + "p2: " + fmt14(c.D(3)));
        }
        std::ostringstream fps_ss;
        fps_ss << std::setprecision(6) << p.fps;
        lines.push_back("Camera.width: " + std::to_string(p.width));
        lines.push_back("Camera.height: " + std::to_string(p.height));
        lines.push_back("Camera.fps: " + fps_ss.str());
        lines.push_back("Camera.RGB: 0");

        if ((mode == "stereo" || mode == "stereo_imu" || mode == "multi" || mode == "multi_imu") && p.has_T_c1_c2) {
            Eigen::Matrix4d T_12 = slam_common::rigid_matrix(p.T_c1_c2);
            double baseline = T_12.topRightCorner<3, 1>().norm();
            if (baseline <= 0.0 || !std::isfinite(baseline)) {
                throw std::invalid_argument("Stereo baseline must be positive");
            }
            lines.push_back("Stereo.ThDepth: 40.0");
            lines.push_back("Stereo.b: " + fmt14(baseline));
            append_opencv_matrix(lines, "Stereo.T_c1_c2", T_12);
        } else if (mode == "stereo" || mode == "stereo_imu") {
            throw std::invalid_argument("Stereo baseline must be positive");
        }

        if (p.extra_cameras.size() >= 2) {
            lines.push_back("Rig.num_cameras: " + std::to_string(p.extra_cameras.size()));
            for (const auto& c : p.extra_cameras) {
                int idx = c.cam_idx + 1;
                if (idx >= 2) {
                    append_opencv_matrix(lines, "Rig.T_c1_c" + std::to_string(idx), slam_common::rigid_matrix(c.T_c0_ci));
                }
            }
        }
    }

    if (enable_imu || mode == "stereo" || mode == "stereo_imu") {
        Eigen::Matrix4d T_imu = p.has_T_b_c1 ? slam_common::rigid_matrix(p.T_b_c1) : Eigen::Matrix4d::Identity();
        if (mode != "imu_only") {
            if (!p.imu_extrinsic_status.empty() && p.imu_extrinsic_status != "ok") {
                std::string warn = p.imu_extrinsic_warning.empty()
                    ? ("IMU.T_b_c1 status=" + p.imu_extrinsic_status)
                    : p.imu_extrinsic_warning;
                std::replace(warn.begin(), warn.end(), '\n', ' ');
                lines.push_back("# WARNING: " + warn);
            }
            append_opencv_matrix(lines, "IMU.T_b_c1", T_imu);
        }
        lines.push_back("IMU.NoiseGyro: " + fmt14(p.noise_gyro));
        lines.push_back("IMU.NoiseAcc: " + fmt14(p.noise_acc));
        lines.push_back("IMU.GyroWalk: " + fmt14(p.gyro_walk));
        lines.push_back("IMU.AccWalk: " + fmt14(p.acc_walk));
        lines.push_back("IMU.Frequency: " + fmt14(p.imu_hz));
        lines.push_back("IMU.fastInit: " + std::to_string(p.fast_init ? 1 : 0));
        if (p.has_imu_stats) {
            std::ostringstream gb;
            gb << std::setprecision(8)
               << "IMU.GyroBias: [" << p.imu_stats.gyro_bias_rad_s(0) << ", "
               << p.imu_stats.gyro_bias_rad_s(1) << ", "
               << p.imu_stats.gyro_bias_rad_s(2) << "]";
            lines.push_back(gb.str());
        }
    }

    if (mode != "imu_only") {
        lines.insert(lines.end(), {
            "ORBextractor.nFeatures: 2000",
            "ORBextractor.scaleFactor: 1.2",
            "ORBextractor.nLevels: 8",
            "ORBextractor.iniThFAST: 15",
            "ORBextractor.minThFAST: 5",
            "Viewer.KeyFrameSize: 0.05",
            "Viewer.KeyFrameLineWidth: 1.0",
            "Viewer.GraphLineWidth: 0.9",
            "Viewer.PointSize: 2.0",
            "Viewer.CameraSize: 0.08",
            "Viewer.CameraLineWidth: 3.0",
            "Viewer.ViewpointX: 0.0",
            "Viewer.ViewpointY: -0.7",
            "Viewer.ViewpointZ: -3.5",
            "Viewer.ViewpointF: 500.0",
            "System.thFarPoints: 20.0",
        });
    }

    std::string yaml_text;
    for (const auto& ln : lines) {
        yaml_text += ln;
        yaml_text.push_back('\n');
    }
    std::filesystem::path yaml_path = p.output_dir / "orbslam_calibration.yaml";
    {
        std::ofstream yf(yaml_path, std::ios::binary | std::ios::trunc);
        if (!yf) throw std::runtime_error("Cannot write " + yaml_path.string());
        yf.write(yaml_text.data(), static_cast<std::streamsize>(yaml_text.size()));
    }
    if (mode == "imu_only") {
        std::ofstream iuf(p.output_dir / "imu_calibration.yaml", std::ios::binary | std::ios::trunc);
        iuf.write(yaml_text.data(), static_cast<std::streamsize>(yaml_text.size()));
    }

    int num_cams = !p.extra_cameras.empty()
        ? static_cast<int>(p.extra_cameras.size())
        : (mode == "imu_only" ? 0 : (mode.rfind("mono", 0) == 0 ? 1 : 2));
    JsonValue profile = JsonValue::object();
    profile["schema_version"] = static_cast<int64_t>(1);
    profile["id"] = p.profile_id;
    profile["name"] = p.profile_name;
    profile["family"] = (p.adapter == "umi_ms901m_uniform") ? "UMI" : "EGO";
    profile["adapter"] = p.adapter;
    profile["sensor_mode"] = mode;
    profile["num_cameras"] = static_cast<int64_t>(num_cams);
    profile["enable_imu"] = enable_imu;
    profile["timestamp_unit"] = "ns";
    profile["timestamp_clock"] = "camera_hardware_clock";
    profile["gyro_unit"] = "rad/s";
    profile["accel_unit"] = "m/s^2";
    profile["stereo_sync"] = "exact";
    profile["camera_to_imu_offset_ns"] = p.timeshift_ns;
    profile["target_frame"] = "imu";
    profile["T_imu_target"] = JsonValue(nullptr);
    profile["auxiliary_streams"] = JsonValue::array();
    profile["calibration_source"] = p.calibration_source;
    profile["notes"] = "Generated by " + p.calibration_source + " (" + mode + ") without ROS.";
    if (p.adapter == "umi_ms901m_uniform") {
        profile["imu_hz"] = p.imu_hz;
        profile["accel_range_g"] = p.accel_range_g;
        profile["gyro_range_dps"] = p.gyro_range_dps;
    }
    if (!p.imu_extrinsic_status.empty()) {
        profile["imu_extrinsic_status"] = p.imu_extrinsic_status;
        if (p.imu_extrinsic_status != "ok" && !p.imu_extrinsic_warning.empty()) {
            profile["imu_extrinsic_warning"] = p.imu_extrinsic_warning;
        }
    }

    std::filesystem::path json_path = p.output_dir / "device_profile.json";
    slam_common::save_json(json_path, profile, true);

    if (p.extra_cameras.size() >= 2) {
        JsonValue rig = JsonValue::object();
        rig["num_cameras"] = static_cast<int64_t>(p.extra_cameras.size());
        JsonValue cam_arr = JsonValue::array();
        for (const auto& c : p.extra_cameras) {
            JsonValue cj = JsonValue::object();
            cj["cam_idx"] = static_cast<int64_t>(c.cam_idx);
            cj["image_size"] = JsonValue(JsonValue::Array{
                static_cast<int64_t>(c.image_size.width),
                static_cast<int64_t>(c.image_size.height)});
            JsonValue K_arr = JsonValue::array();
            for (int r = 0; r < 3; ++r) {
                K_arr.push_back(JsonValue(JsonValue::Array{c.K(r, 0), c.K(r, 1), c.K(r, 2)}));
            }
            cj["K"] = K_arr;
            cj["D"] = JsonValue(JsonValue::Array{c.D(0), c.D(1), c.D(2), c.D(3)});
            JsonValue T_arr = JsonValue::array();
            for (int r = 0; r < 4; ++r) {
                T_arr.push_back(JsonValue(JsonValue::Array{c.T_c0_ci(r, 0), c.T_c0_ci(r, 1), c.T_c0_ci(r, 2), c.T_c0_ci(r, 3)}));
            }
            cj["T_c0_ci"] = T_arr;
            cam_arr.push_back(cj);
        }
        rig["cameras"] = cam_arr;
        if (enable_imu) {
            Eigen::Matrix4d Tb = p.has_T_b_c1 ? slam_common::rigid_matrix(p.T_b_c1) : Eigen::Matrix4d::Identity();
            JsonValue Tb_arr = JsonValue::array();
            for (int r = 0; r < 4; ++r) {
                Tb_arr.push_back(JsonValue(JsonValue::Array{Tb(r, 0), Tb(r, 1), Tb(r, 2), Tb(r, 3)}));
            }
            rig["T_b_c0"] = Tb_arr;
        } else {
            rig["T_b_c0"] = JsonValue(nullptr);
        }
        rig["camera_to_imu_offset_ns"] = p.timeshift_ns;
        slam_common::save_json(p.output_dir / "multicam_calibration.json", rig, true);
    }

    return {yaml_path, json_path};
}

std::pair<std::filesystem::path, std::filesystem::path> convert_kalibr_files(
    const std::filesystem::path& camchain_yaml,
    const std::filesystem::path& imu_yaml,
    const std::filesystem::path& output_dir,
    double fps,
    const std::string& adapter,
    double accel_range_g,
    double gyro_range_dps) {
    JsonValue cam_doc = parse_simple_yaml(slam_common::read_text_utf8(camchain_yaml));
    JsonValue imu_doc = (!imu_yaml.empty() && std::filesystem::is_regular_file(imu_yaml))
        ? parse_simple_yaml(slam_common::read_text_utf8(imu_yaml))
        : JsonValue::object();
    const JsonValue& imu_cfg = imu_doc.contains("imu0") ? imu_doc.get("imu0") : imu_doc;

    static const std::regex kCamRe(R"(^cam(\d+)$)");
    std::vector<std::pair<int, std::string>> cam_names;
    for (const auto& kv : cam_doc.as_object()) {
        std::smatch m;
        if (std::regex_match(kv.first, m, kCamRe)) {
            cam_names.emplace_back(std::stoi(m[1].str()), kv.first);
        }
    }
    if (cam_names.empty()) throw std::invalid_argument("camchain YAML 中未找到 cam0 / cam1 等相机字段");
    std::sort(cam_names.begin(), cam_names.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

    auto make_K = [](const JsonValue& intr) -> Eigen::Matrix3d {
        const auto& a = intr.as_array();
        Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
        K(0, 0) = a[0].as_double(); K(1, 1) = a[1].as_double();
        K(0, 2) = a[2].as_double(); K(1, 2) = a[3].as_double();
        return K;
    };
    auto make_D = [](const JsonValue& dist) -> Eigen::Vector4d {
        const auto& a = dist.as_array();
        Eigen::Vector4d D = Eigen::Vector4d::Zero();
        for (size_t i = 0; i < std::min<size_t>(4, a.size()); ++i) D(static_cast<Eigen::Index>(i)) = a[i].as_double();
        return D;
    };

    const JsonValue& c0 = cam_doc.get("cam0");
    if (c0.get("camera_model").as_string() != "pinhole" || c0.get("distortion_model").as_string() != "radtan") {
        throw std::invalid_argument("Only pinhole + radtan camera model is supported");
    }
    int w = static_cast<int>(c0.get("resolution").as_array()[0].as_int());
    int h = static_cast<int>(c0.get("resolution").as_array()[1].as_int());
    bool has_imu = imu_cfg.is_object() && imu_cfg.contains("update_rate");

    WriteOrbYamlParams wp;
    wp.output_dir = output_dir;
    wp.cam0_K = make_K(c0.get("intrinsics"));
    wp.cam0_D = make_D(c0.get("distortion_coeffs"));
    wp.width = w;
    wp.height = h;
    wp.has_T_b_c1 = true;
    wp.T_b_c1 = c0.contains("T_cam_imu")
        ? Eigen::Matrix4d(slam_common::rigid_matrix(json_to_matrix4d(c0.get("T_cam_imu"))).inverse())
        : Eigen::Matrix4d::Identity().eval();
    wp.timeshift_ns = c0.contains("timeshift_cam_imu")
        ? static_cast<int64_t>(std::llround(c0.get("timeshift_cam_imu").as_double() * 1e9))
        : 0;

    if (cam_names.size() >= 2) {
        std::vector<Eigen::Matrix4d> T_c0_chain = {Eigen::Matrix4d::Identity()};
        for (size_t k = 1; k < cam_names.size(); ++k) {
            const JsonValue& ci = cam_doc.get(cam_names[k].second);
            if (ci.get("camera_model").as_string() != "pinhole" || ci.get("distortion_model").as_string() != "radtan") {
                throw std::invalid_argument(cam_names[k].second + ": Only pinhole camera model and radtan distortion are supported");
            }
            Eigen::Matrix4d T_prev_curr = slam_common::rigid_matrix(json_to_matrix4d(ci.get("T_cn_cnm1"))).inverse();
            T_c0_chain.push_back(slam_common::rigid_matrix(T_c0_chain.back() * T_prev_curr));
        }
        const JsonValue& c1 = cam_doc.get("cam1");
        int w1 = static_cast<int>(c1.get("resolution").as_array()[0].as_int());
        int h1 = static_cast<int>(c1.get("resolution").as_array()[1].as_int());
        if (cam_names.size() == 2 && (w1 != w || h1 != h)) {
            throw std::invalid_argument("Stereo cam0/cam1 resolutions do not match");
        }
        wp.has_cam1 = true;
        wp.cam1_K = make_K(c1.get("intrinsics"));
        wp.cam1_D = make_D(c1.get("distortion_coeffs"));
        wp.has_T_c1_c2 = true;
        wp.T_c1_c2 = T_c0_chain[1];
        if (cam_names.size() > 2) {
            for (size_t k = 0; k < cam_names.size(); ++k) {
                const JsonValue& ci = cam_doc.get(cam_names[k].second);
                MultiCamEntry entry;
                entry.cam_idx = cam_names[k].first;
                entry.K = make_K(ci.get("intrinsics"));
                entry.D = make_D(ci.get("distortion_coeffs"));
                entry.image_size = cv::Size(
                    static_cast<int>(ci.get("resolution").as_array()[0].as_int()),
                    static_cast<int>(ci.get("resolution").as_array()[1].as_int()));
                entry.T_c0_ci = T_c0_chain[k];
                wp.extra_cameras.push_back(entry);
            }
        }
    }

    if (cam_names.size() == 1) wp.sensor_mode = has_imu ? "mono_imu" : "mono";
    else if (cam_names.size() == 2) wp.sensor_mode = has_imu ? "stereo_imu" : "stereo";
    else wp.sensor_mode = has_imu ? "multi_imu" : "multi";

    wp.fps = fps;
    wp.imu_hz = imu_cfg.contains("update_rate") ? imu_cfg.get("update_rate").as_double() : 250.0;
    wp.noise_gyro = imu_cfg.contains("gyroscope_noise_density") ? imu_cfg.get("gyroscope_noise_density").as_double() : 0.0035;
    wp.noise_acc = imu_cfg.contains("accelerometer_noise_density") ? imu_cfg.get("accelerometer_noise_density").as_double() : 0.02;
    wp.gyro_walk = imu_cfg.contains("gyroscope_random_walk") ? imu_cfg.get("gyroscope_random_walk").as_double() : 0.0002;
    wp.acc_walk = imu_cfg.contains("accelerometer_random_walk") ? imu_cfg.get("accelerometer_random_walk").as_double() : 0.002;
    wp.adapter = adapter;
    wp.accel_range_g = accel_range_g;
    wp.gyro_range_dps = gyro_range_dps;
    wp.calibration_source = camchain_yaml.filename().string();
    wp.profile_name = "Kalibr 导入配置";

    return write_orbslam_calibration(wp);
}

SuiteCalibrationOutput calibrate_sensor_suite(const SuiteCalibrationParams& params) {
    std::filesystem::path src = std::filesystem::weakly_canonical(params.source_dir);
    std::string cam_mode = trim_lower(params.camera_mode.empty() ? "stereo" : params.camera_mode);
    if (cam_mode != "none" && cam_mode != "mono" && cam_mode != "stereo" && cam_mode != "multi") {
        throw std::invalid_argument("不支持的相机模式: " + params.camera_mode + "（可选 none / mono / stereo / multi）");
    }
    if (cam_mode == "none" && !params.enable_imu) {
        throw std::invalid_argument("至少需要选择一种相机模式或启用 IMU 标定");
    }

    auto log_msg = [&](const std::string& msg) {
        if (params.progress_cb) params.progress_cb(msg);
    };

    std::filesystem::path out_dir = std::filesystem::weakly_canonical(params.output_dir);
    std::filesystem::create_directories(out_dir);

    auto imu_stats_to_json = [](const ImuStandaloneStats& st) {
        JsonValue j = JsonValue::object();
        j["measured_hz"] = st.measured_hz;
        j["duration_s"] = st.duration_s;
        j["sample_count"] = static_cast<int64_t>(st.sample_count);
        j["static_samples"] = static_cast<int64_t>(st.static_samples);
        j["gyro_bias_rad_s"] = JsonValue(JsonValue::Array{st.gyro_bias_rad_s(0), st.gyro_bias_rad_s(1), st.gyro_bias_rad_s(2)});
        j["acc_mean_mps2"] = JsonValue(JsonValue::Array{st.acc_mean_mps2(0), st.acc_mean_mps2(1), st.acc_mean_mps2(2)});
        j["gravity_norm_mps2"] = st.gravity_norm_mps2;
        j["noise_gyro"] = st.noise_gyro;
        j["noise_acc"] = st.noise_acc;
        j["gyro_walk"] = st.gyro_walk;
        j["acc_walk"] = st.acc_walk;
        return j;
    };

    if (cam_mode == "none") {
        slam_common::ImuSeries imu;
        if (!read_source_imu(src, params.imu_hz, params.accel_range_g, params.gyro_range_dps, imu)) {
            throw std::invalid_argument("未在目录中找到 IMU 数据（需要 imu.csv / mav0/imu0/data.csv / umi_imu_timestamps.bin）");
        }
        ImuStandaloneStats st = calibrate_imu_standalone(imu.timestamps_ns, imu.samples, params.imu_hz);
        std::string adapter = std::filesystem::is_regular_file(src / "umi_imu_timestamps.bin")
            ? "umi_ms901m_uniform"
            : "euroc";
        WriteOrbYamlParams wp;
        wp.output_dir = out_dir;
        wp.imu_hz = st.measured_hz;
        wp.noise_gyro = st.noise_gyro;
        wp.noise_acc = st.noise_acc;
        wp.gyro_walk = st.gyro_walk;
        wp.acc_walk = st.acc_walk;
        wp.adapter = adapter;
        wp.accel_range_g = params.accel_range_g;
        wp.gyro_range_dps = params.gyro_range_dps;
        wp.calibration_source = "SLAMCalibrator-IMU";
        wp.profile_name = "仅 IMU 自标定 (" + src.filename().string() + ")";
        wp.sensor_mode = "imu_only";
        wp.has_imu_stats = true;
        wp.imu_stats = st;
        auto [yaml_path, json_path] = write_orbslam_calibration(wp);

        JsonValue report = JsonValue::object();
        report["source_dir"] = src.string();
        report["sensor_mode"] = "imu_only";
        report["camera_mode"] = "none";
        report["enable_imu"] = true;
        report["imu_stats"] = imu_stats_to_json(st);
        report["orbslam_yaml"] = yaml_path.string();
        report["device_profile"] = json_path.string();
        std::filesystem::path report_path = out_dir / "calibration_report.json";
        slam_common::save_json(report_path, report, true);
        return {yaml_path, json_path, report_path, report};
    }

    if (!std::filesystem::exists(src)) throw std::invalid_argument("标定采集路径不存在");
    if (params.stride < 1 || params.rows < 2 || params.cols < 2 || params.tag_size_m <= 0.0) {
        throw std::invalid_argument("标定参数无效，请检查行列数、物理尺寸与抽帧间隔");
    }

    DetectCallback detect_fn = params.detect_callback
        ? params.detect_callback
        : make_builtin_detector(params.target_type, params.rows, params.cols, params.tag_size_m, params.tag_spacing, log_msg);

    std::vector<Observation> observations;
    std::vector<cv::Size> image_sizes;
    std::string adapter = "euroc";
    std::vector<int64_t> times_ref;
    std::filesystem::path imu_root = std::filesystem::is_directory(src) ? src : src.parent_path();

    auto check_cancel = [&]() {
        if (params.cancel_flag && params.cancel_flag->load()) {
            throw std::runtime_error("标定已取消");
        }
    };

    if (cam_mode == "mono") {
        std::filesystem::path video_path;
        bool split_left = false;
        if (std::filesystem::is_regular_file(src) && is_video_ext(src)) {
            video_path = src;
            split_left = (src.filename() == "stereo.mkv");
            imu_root = src.parent_path();
        } else if (std::filesystem::is_regular_file(src / "stereo.mkv")) {
            video_path = src / "stereo.mkv";
            split_left = true;
            imu_root = src;
        } else if (std::filesystem::is_directory(src)) {
            std::vector<std::filesystem::path> vids;
            for (const auto& entry : std::filesystem::directory_iterator(src)) {
                if (entry.is_regular_file() && is_video_ext(entry.path())) vids.push_back(entry.path());
            }
            std::sort(vids.begin(), vids.end());
            if (!vids.empty()) {
                video_path = vids.front();
                imu_root = src;
            }
        }

        cv::Size img_sz(0, 0);
        if (!video_path.empty()) {
            cv::VideoCapture cap(video_path.string());
            if (!cap.isOpened()) throw std::invalid_argument("无法打开视频文件: " + video_path.string());
            int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
            double vid_fps = cap.get(cv::CAP_PROP_FPS);
            if (vid_fps <= 0.0) vid_fps = (params.fps > 0.0 ? params.fps : 30.0);
            times_ref = read_video_timestamps(imu_root, total_frames, vid_fps);
            int n_avail = (total_frames > 0) ? std::min<int>(static_cast<int>(times_ref.size()), total_frames) : static_cast<int>(times_ref.size());
            std::vector<int> idx_seq(n_avail);
            std::iota(idx_seq.begin(), idx_seq.end(), 0);
            auto chosen = select_span_samples(idx_seq, params.stride, params.max_frames);
            if (chosen.size() < 8) throw std::invalid_argument("可用单目图像帧不足（至少需要 8 帧）");
            adapter = split_left ? "umi_ms901m_uniform" : "euroc";
            std::set<int> wanted(chosen.begin(), chosen.end());
            for (int idx = 0; idx <= chosen.back(); ++idx) {
                check_cancel();
                if (!cap.grab()) break;
                if (!wanted.count(idx)) continue;
                cv::Mat frame;
                if (!cap.retrieve(frame) || frame.empty()) continue;
                int half = split_left ? (frame.cols / 2) : frame.cols;
                cv::Mat crop = frame.colRange(0, half);
                cv::Mat gray;
                if (crop.channels() > 1) cv::cvtColor(crop, gray, cv::COLOR_BGR2GRAY);
                else gray = crop;
                if (img_sz.width == 0) img_sz = cv::Size(gray.cols, gray.rows);
                auto det = detect_fn(gray, 0);
                if (det.found) {
                    Observation obs;
                    obs.ts_ns = times_ref[idx];
                    obs.obj_pts[0] = std::move(det.obj);
                    obs.img_pts[0] = std::move(det.img);
                    observations.push_back(std::move(obs));
                    if (params.preview_cb && (observations.size() == 1 || observations.size() % 10 == 0)) {
                        params.preview_cb(gray, gray, observations.back().img_pts[0], {}, static_cast<int>(observations.size()));
                    }
                }
            }
        } else {
            auto cdirs = discover_multicam_dirs(src, {});
            std::filesystem::path img_dir = cdirs.empty() ? src : cdirs[0];
            auto img_map = discover_image_folder_times(img_dir);
            for (const auto& kv : img_map) times_ref.push_back(kv.first);
            auto chosen = select_span_samples(times_ref, params.stride, params.max_frames);
            if (chosen.size() < 8) throw std::invalid_argument("可用单目图像帧不足（至少需要 8 帧）");
            for (size_t k = 0; k < chosen.size(); ++k) {
                check_cancel();
                cv::Mat gray = cv::imread(img_map[chosen[k]].string(), cv::IMREAD_GRAYSCALE);
                if (gray.empty()) continue;
                if (img_sz.width == 0) img_sz = cv::Size(gray.cols, gray.rows);
                auto det = detect_fn(gray, 0);
                if (det.found) {
                    Observation obs;
                    obs.ts_ns = chosen[k];
                    obs.obj_pts[0] = std::move(det.obj);
                    obs.img_pts[0] = std::move(det.img);
                    observations.push_back(std::move(obs));
                    if (params.preview_cb && (observations.size() == 1 || observations.size() % 10 == 0)) {
                        params.preview_cb(gray, gray, observations.back().img_pts[0], {}, static_cast<int>(observations.size()));
                    }
                }
                if ((k + 1) % 15 == 0) log_msg("单目扫描 " + std::to_string(k + 1) + "/" + std::to_string(chosen.size()) + "，有效帧 " + std::to_string(observations.size()));
            }
        }
        image_sizes.push_back(img_sz);
    } else if (cam_mode == "stereo") {
        bool legacy = std::filesystem::is_regular_file(src / "stereo.mkv");
        cv::Size img_sz(0, 0);
        if (legacy) {
            times_ref = read_video_timestamps(src, 0, 30.0);
            adapter = "umi_ms901m_uniform";
            cv::VideoCapture cap((src / "stereo.mkv").string());
            if (!cap.isOpened()) throw std::invalid_argument("无法打开 stereo.mkv");
            std::vector<int> idx_seq(times_ref.size());
            std::iota(idx_seq.begin(), idx_seq.end(), 0);
            auto chosen = select_span_samples(idx_seq, params.stride, params.max_frames);
            if (chosen.size() < 8) throw std::invalid_argument("可用图像帧不足（至少需要 8 对帧）");
            std::set<int> wanted(chosen.begin(), chosen.end());
            for (int idx = 0; idx <= chosen.back(); ++idx) {
                check_cancel();
                if (!cap.grab()) break;
                if (!wanted.count(idx)) continue;
                cv::Mat frame;
                if (!cap.retrieve(frame) || frame.empty()) continue;
                int half = frame.cols / 2;
                cv::Mat left_raw = frame.colRange(0, half);
                cv::Mat right_raw = frame.colRange(half, frame.cols);
                cv::Mat left, right;
                if (left_raw.channels() > 1) {
                    cv::cvtColor(left_raw, left, cv::COLOR_BGR2GRAY);
                    cv::cvtColor(right_raw, right, cv::COLOR_BGR2GRAY);
                } else {
                    left = left_raw; right = right_raw;
                }
                if (img_sz.width == 0) img_sz = cv::Size(left.cols, left.rows);
                auto d0 = detect_fn(left, 0);
                auto d1 = detect_fn(right, 1);
                if (d0.found || d1.found) {
                    Observation obs;
                    obs.ts_ns = times_ref[idx];
                    if (d0.found) { obs.obj_pts[0] = std::move(d0.obj); obs.img_pts[0] = std::move(d0.img); }
                    if (d1.found) { obs.obj_pts[1] = std::move(d1.obj); obs.img_pts[1] = std::move(d1.img); }
                    observations.push_back(std::move(obs));
                    if (params.preview_cb && (observations.size() == 1 || observations.size() % 10 == 0)) {
                        params.preview_cb(
                            left, right,
                            observations.back().img_pts[0], observations.back().img_pts[1],
                            static_cast<int>(observations.size()));
                    }
                }
            }
        } else {
            auto cdirs = discover_multicam_dirs(src, params.camera_dirs);
            if (cdirs.size() < 2) throw std::invalid_argument("双目标定需要 stereo.mkv 或包含 cam0 + cam1 的采集目录");
            auto m0 = discover_image_folder_times(cdirs[0]);
            auto m1 = discover_image_folder_times(cdirs[1]);
            for (const auto& kv : m0) {
                if (m1.count(kv.first)) times_ref.push_back(kv.first);
            }
            auto chosen = select_span_samples(times_ref, params.stride, params.max_frames);
            if (chosen.size() < 8) throw std::invalid_argument("可用图像帧不足（至少需要 8 对帧）");
            for (size_t k = 0; k < chosen.size(); ++k) {
                check_cancel();
                cv::Mat left = cv::imread(m0[chosen[k]].string(), cv::IMREAD_GRAYSCALE);
                cv::Mat right = cv::imread(m1[chosen[k]].string(), cv::IMREAD_GRAYSCALE);
                if (left.empty() || right.empty()) continue;
                if (img_sz.width == 0) img_sz = cv::Size(left.cols, left.rows);
                auto d0 = detect_fn(left, 0);
                auto d1 = detect_fn(right, 1);
                if (d0.found || d1.found) {
                    Observation obs;
                    obs.ts_ns = chosen[k];
                    if (d0.found) { obs.obj_pts[0] = std::move(d0.obj); obs.img_pts[0] = std::move(d0.img); }
                    if (d1.found) { obs.obj_pts[1] = std::move(d1.obj); obs.img_pts[1] = std::move(d1.img); }
                    observations.push_back(std::move(obs));
                }
                if ((k + 1) % 15 == 0) log_msg("扫描 " + std::to_string(k + 1) + "/" + std::to_string(chosen.size()) + "，有效靶标帧 " + std::to_string(observations.size()));
            }
        }
        image_sizes.push_back(img_sz);
    } else {
        auto cdirs = discover_multicam_dirs(src, params.camera_dirs);
        if (cdirs.size() < 2) throw std::invalid_argument("多目标定需要至少 2 个相机目录");
        std::vector<std::map<int64_t, std::filesystem::path>> cam_maps;
        for (const auto& d : cdirs) cam_maps.push_back(discover_image_folder_times(d));
        std::set<int64_t> common;
        for (const auto& kv : cam_maps[0]) common.insert(kv.first);
        for (size_t i = 1; i < cam_maps.size(); ++i) {
            std::set<int64_t> next_common;
            for (int64_t t : common) {
                if (cam_maps[i].count(t)) next_common.insert(t);
            }
            common = std::move(next_common);
        }
        if (common.size() >= 8) {
            times_ref.assign(common.begin(), common.end());
        } else {
            std::set<int64_t> all_ts;
            for (const auto& m : cam_maps) {
                for (const auto& kv : m) all_ts.insert(kv.first);
            }
            times_ref.assign(all_ts.begin(), all_ts.end());
        }
        auto chosen = select_span_samples(times_ref, params.stride, params.max_frames);
        if (chosen.size() < 8) throw std::invalid_argument("多目可用时间帧不足");
        image_sizes.assign(cam_maps.size(), cv::Size(0, 0));
        for (size_t k = 0; k < chosen.size(); ++k) {
            check_cancel();
            Observation obs;
            obs.ts_ns = chosen[k];
            bool any_det = false;
            for (size_t ci = 0; ci < cam_maps.size(); ++ci) {
                auto it = cam_maps[ci].find(chosen[k]);
                if (it == cam_maps[ci].end()) continue;
                cv::Mat gray = cv::imread(it->second.string(), cv::IMREAD_GRAYSCALE);
                if (gray.empty()) continue;
                if (image_sizes[ci].width == 0) image_sizes[ci] = cv::Size(gray.cols, gray.rows);
                auto det = detect_fn(gray, static_cast<int>(ci));
                if (det.found) {
                    obs.obj_pts[static_cast<int>(ci)] = std::move(det.obj);
                    obs.img_pts[static_cast<int>(ci)] = std::move(det.img);
                    any_det = true;
                }
            }
            if (any_det) observations.push_back(std::move(obs));
        }
    }

    if (observations.empty() || std::any_of(image_sizes.begin(), image_sizes.end(), [](const cv::Size& s) { return s.width <= 0 || s.height <= 0; })) {
        throw std::invalid_argument("未检测到有效标定板角点，请检查标定板类型、行列数、物理尺寸与图像清晰度");
    }

    double fps = params.fps;
    if (fps <= 0.0) {
        if (times_ref.size() >= 2) {
            std::vector<double> pos_dts;
            for (size_t i = 1; i < times_ref.size(); ++i) {
                double dt = static_cast<double>(times_ref[i] - times_ref[i - 1]) * 1e-9;
                if (dt > 0.0) pos_dts.push_back(dt);
            }
            double med = pos_dts.empty() ? 0.0 : median_val(pos_dts);
            fps = (med > 1e-3 && med < 1.0) ? (std::round(100.0 / med) / 100.0) : 125.0;
        } else {
            fps = 125.0;
        }
    }

    std::string sensor_mode = params.enable_imu ? (cam_mode + "_imu") : cam_mode;
    WriteOrbYamlParams wp;
    wp.output_dir = out_dir;
    wp.width = image_sizes[0].width;
    wp.height = image_sizes[0].height;
    wp.fps = fps;
    wp.imu_hz = params.imu_hz;
    wp.noise_gyro = params.noise_gyro;
    wp.noise_acc = params.noise_acc;
    wp.gyro_walk = params.gyro_walk;
    wp.acc_walk = params.acc_walk;
    wp.adapter = adapter;
    wp.accel_range_g = params.accel_range_g;
    wp.gyro_range_dps = params.gyro_range_dps;
    wp.calibration_source = "SLAMCalibrator-OpenCV";
    wp.profile_name = "多模态自标定 (" + src.filename().string() + ")";
    wp.sensor_mode = sensor_mode;

    JsonValue mono_rms_json = JsonValue::object();
    JsonValue stereo_rms_json(nullptr);
    JsonValue baseline_json(nullptr);
    JsonValue pair_stats_json(nullptr);

    if (cam_mode == "mono") {
        auto m0 = calibrate_monocular_from_observations(observations, image_sizes[0], 0);
        wp.cam0_K = m0.K; wp.cam0_D = m0.D;
        mono_rms_json["0"] = m0.mono_rms;
        log_msg("单目标定完成：RMS=" + std::to_string(m0.mono_rms) + " px（有效帧 " + std::to_string(m0.frames) + "）");
    } else if (cam_mode == "stereo") {
        auto st = calibrate_stereo_from_observations(observations, image_sizes[0]);
        wp.cam0_K = st.K0; wp.cam0_D = st.D0;
        wp.has_cam1 = true; wp.cam1_K = st.K1; wp.cam1_D = st.D1;
        wp.has_T_c1_c2 = true; wp.T_c1_c2 = st.T_c1_c2;
        mono_rms_json["0"] = st.mono_rms[0];
        mono_rms_json["1"] = st.mono_rms[1];
        stereo_rms_json = JsonValue(st.stereo_rms);
        baseline_json = JsonValue(st.baseline);
        log_msg("双目标定完成：联合 RMS=" + std::to_string(st.stereo_rms) + " px");
    } else {
        auto mc = calibrate_multicam_from_observations(observations, image_sizes);
        wp.extra_cameras = mc.cameras;
        wp.cam0_K = mc.cameras[0].K; wp.cam0_D = mc.cameras[0].D;
        wp.has_cam1 = true; wp.cam1_K = mc.cameras[1].K; wp.cam1_D = mc.cameras[1].D;
        wp.has_T_c1_c2 = true; wp.T_c1_c2 = mc.cameras[1].T_c0_ci;
        for (const auto& c : mc.cameras) mono_rms_json[std::to_string(c.cam_idx)] = c.mono_rms;
        pair_stats_json = mc.pair_stats;
        baseline_json = JsonValue(wp.T_c1_c2.topRightCorner<3, 1>().norm());
    }

    std::vector<std::string> ext_warnings;
    bool has_imu_res = false;
    CamImuExtrinsicResult imu_res;
    if (params.enable_imu) {
        slam_common::ImuSeries imu;
        if (read_source_imu(imu_root, params.imu_hz, params.accel_range_g, params.gyro_range_dps, imu)) {
            try {
                wp.imu_stats = calibrate_imu_standalone(imu.timestamps_ns, imu.samples, params.imu_hz);
                wp.has_imu_stats = true;
                imu_res = estimate_cam_imu_extrinsic(observations, wp.cam0_K, wp.cam0_D, imu.timestamps_ns, imu.samples, 0);
                has_imu_res = true;
                wp.has_T_b_c1 = true;
                wp.T_b_c1 = imu_res.T_b_c1;
                wp.timeshift_ns = imu_res.timeshift_ns;
                ext_warnings = imu_res.warnings;
                if (!imu_res.translation_valid) wp.imu_extrinsic_status = "rotation_only";
                else if (imu_res.timeshift_at_bound || imu_res.timeshift_low_corr) wp.imu_extrinsic_status = "timeshift_warning";
                else wp.imu_extrinsic_status = "ok";
                for (const auto& w : ext_warnings) log_msg("[警告] " + w);
            } catch (const std::exception& exc) {
                wp.imu_extrinsic_status = "failed";
                std::string w = std::string("IMU 外参自动估计失败（") + exc.what() +
                                "），导出的 IMU.T_b_c1 为占位单位阵，不可直接当作已标定外参用于 SLAM。";
                ext_warnings.push_back(w);
                log_msg("[警告] " + w);
            }
        } else {
            wp.imu_extrinsic_status = "no_imu_data";
            std::string w = "未检测到 IMU 数据文件，导出的 IMU.T_b_c1 为占位单位阵，不可直接当作已标定外参用于 SLAM。";
            ext_warnings.push_back(w);
            log_msg("[警告] " + w);
        }
    }

    if (!ext_warnings.empty()) {
        std::string joined;
        for (size_t i = 0; i < ext_warnings.size(); ++i) {
            if (i > 0) joined += "；";
            joined += ext_warnings[i];
        }
        wp.imu_extrinsic_warning = joined;
    }

    auto [yaml_path, json_path] = write_orbslam_calibration(wp);

    JsonValue report = JsonValue::object();
    report["source_dir"] = src.string();
    report["sensor_mode"] = sensor_mode;
    report["camera_mode"] = cam_mode;
    report["enable_imu"] = params.enable_imu;
    report["target_type"] = params.target_type;
    report["rows"] = static_cast<int64_t>(params.rows);
    report["cols"] = static_cast<int64_t>(params.cols);
    report["tag_size_m"] = params.tag_size_m;
    JsonValue sizes_arr = JsonValue::array();
    for (const auto& sz : image_sizes) {
        sizes_arr.push_back(JsonValue(JsonValue::Array{static_cast<int64_t>(sz.width), static_cast<int64_t>(sz.height)}));
    }
    report["image_sizes"] = sizes_arr;
    report["image_size"] = sizes_arr.as_array()[0];
    report["valid_frames"] = static_cast<int64_t>(observations.size());
    report["mono_rms_px"] = mono_rms_json;
    report["stereo_rms_px"] = stereo_rms_json;
    report["baseline_m"] = baseline_json;
    report["pair_stats"] = pair_stats_json;
    report["imu_extrinsic_status"] = wp.imu_extrinsic_status.empty() ? JsonValue(nullptr) : JsonValue(wp.imu_extrinsic_status);
    JsonValue warn_arr = JsonValue::array();
    for (const auto& w : ext_warnings) warn_arr.push_back(w);
    report["imu_extrinsic_warnings"] = warn_arr;
    if (has_imu_res) {
        JsonValue ir = JsonValue::object();
        ir["timeshift_ns"] = imu_res.timeshift_ns;
        ir["gyro_rmse_rad_s"] = imu_res.gyro_rmse_rad_s;
        ir["timeshift_corr"] = imu_res.timeshift_corr;
        ir["timeshift_at_bound"] = imu_res.timeshift_at_bound;
        ir["timeshift_low_corr"] = imu_res.timeshift_low_corr;
        ir["translation_valid"] = imu_res.translation_valid;
        ir["translation_rows"] = static_cast<int64_t>(imu_res.translation_rows);
        ir["imu_pairs"] = static_cast<int64_t>(imu_res.imu_pairs);
        report["imu_result"] = ir;
    } else {
        report["imu_result"] = JsonValue(nullptr);
    }
    report["imu_stats"] = wp.has_imu_stats ? imu_stats_to_json(wp.imu_stats) : JsonValue(nullptr);
    report["orbslam_yaml"] = yaml_path.string();
    report["device_profile"] = json_path.string();

    std::filesystem::path report_path = out_dir / "calibration_report.json";
    slam_common::save_json(report_path, report, true);
    log_msg("标定文件已生成：\n  YAML: " + yaml_path.string() + "\n  JSON: " + json_path.string() + "\n  报告: " + report_path.string());
    return {yaml_path, json_path, report_path, report};
}

}  // namespace camera_imu_calibration
