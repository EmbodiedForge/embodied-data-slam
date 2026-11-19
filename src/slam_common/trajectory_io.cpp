#include "slam_common/trajectory_io.hpp"

#include "slam_common/geometry.hpp"

#include <Eigen/LU>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace slam_common {
namespace {

bool is_nonempty_string(const JsonValue& v) {
    if (!v.is_string()) return false;
    const std::string& s = v.as_string();
    return s.find_first_not_of(" \t\r\n") != std::string::npos;
}

bool is_relative_subpath(const std::filesystem::path& child, const std::filesystem::path& parent) {
    std::error_code ec;
    auto c = std::filesystem::weakly_canonical(child, ec);
    auto p = std::filesystem::weakly_canonical(parent, ec);
    auto it_c = c.begin();
    auto it_p = p.begin();
    for (; it_p != p.end(); ++it_p, ++it_c) {
        if (it_c == c.end() || *it_c != *it_p) return false;
    }
    return true;
}

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::string cur;
    bool in_quotes = false;
    for (size_t i = 0; i < line.size(); ++i) {
        char c = line[i];
        if (in_quotes) {
            if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
                cur.push_back('"');
                ++i;
            } else if (c == '"') {
                in_quotes = false;
            } else {
                cur.push_back(c);
            }
        } else {
            if (c == '"') {
                in_quotes = true;
            } else if (c == ',') {
                fields.push_back(cur);
                cur.clear();
            } else if (c != '\r') {
                cur.push_back(c);
            }
        }
    }
    fields.push_back(cur);
    return fields;
}

std::string format_double(double v) {
    std::ostringstream os;
    os << std::setprecision(15) << v;
    return os.str();
}

}  // namespace

Eigen::Matrix4d validate_transform(const JsonValue& matrix) {
    if (!matrix.is_array() || matrix.as_array().size() != 4) {
        throw std::invalid_argument("T_imu_target 必须是 4×4 矩阵");
    }
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    for (size_t r = 0; r < 4; ++r) {
        const JsonValue& row_val = matrix.as_array()[r];
        if (!row_val.is_array() || row_val.as_array().size() != 4) {
            throw std::invalid_argument("T_imu_target 必须是 4×4 矩阵");
        }
        for (size_t c = 0; c < 4; ++c) {
            const JsonValue& cell = row_val.as_array()[c];
            if (!cell.is_number()) {
                throw std::invalid_argument("外参必须是有限数值");
            }
            double v = cell.as_double();
            if (!std::isfinite(v)) {
                throw std::invalid_argument("外参必须是有限数值");
            }
            m(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = v;
        }
    }
    Eigen::RowVector4d last_expected(0.0, 0.0, 0.0, 1.0);
    if ((m.row(3) - last_expected).cwiseAbs().maxCoeff() > 1e-8) {
        throw std::invalid_argument("外参最后一行必须为 [0,0,0,1]");
    }
    Eigen::Matrix3d R = m.topLeftCorner<3, 3>();
    if ((R.transpose() * R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1e-5) {
        throw std::invalid_argument("外参旋转矩阵必须正交");
    }
    if (std::abs(R.determinant() - 1.0) > 1e-5) {
        throw std::invalid_argument("外参旋转行列式必须为 +1");
    }
    return m;
}

JsonValue load_device(const std::filesystem::path& path) {
    static const std::set<std::string> kValidSensorModes = {
        "imu_only", "mono", "mono_imu", "stereo", "stereo_imu", "multi", "multi_imu"
    };
    JsonValue d = read_json_file(path);
    if (!d.is_object() || !d.get("schema_version").is_int() || d.get("schema_version").as_int() != 1) {
        throw std::invalid_argument("不支持的设备配置版本");
    }
    for (const char* key : {"id", "name", "family", "adapter", "timestamp_clock"}) {
        if (!is_nonempty_string(d.get(key))) {
            throw std::invalid_argument(std::string("缺少设备字段 ") + key);
        }
    }
    const std::string& family = d.get("family").as_string();
    if (family != "UMI" && family != "EGO" && family != "generic") {
        throw std::invalid_argument("family 应为 UMI / EGO / generic");
    }
    const std::string& adapter = d.get("adapter").as_string();
    if (adapter != "euroc" && adapter != "umi_ms901m_uniform") {
        throw std::invalid_argument("尚未实现该原始协议；请通过 DatasetAdapter 转为 EuRoC 后接入");
    }
    for (const auto& kv : std::vector<std::pair<const char*, const char*>>{
             {"timestamp_unit", "ns"},
             {"gyro_unit", "rad/s"},
             {"accel_unit", "m/s^2"},
             {"stereo_sync", "exact"}}) {
        const auto& val = d.get(kv.first);
        if (!val.is_string() || val.as_string() != kv.second) {
            throw std::invalid_argument(std::string(kv.first) + " 必须为 " + kv.second + "；请先显式转换数据");
        }
    }
    if (!d.get("camera_to_imu_offset_ns").is_int()) {
        throw std::invalid_argument("时间偏移必须为整数纳秒");
    }
    if (adapter == "umi_ms901m_uniform") {
        for (const char* k : {"imu_hz", "accel_range_g", "gyro_range_dps"}) {
            const auto& val = d.get(k);
            if (!val.is_number() || !std::isfinite(val.as_double()) || val.as_double() <= 0.0) {
                throw std::invalid_argument(std::string(k) + " 必须为正数");
            }
        }
    }
    if (!is_nonempty_string(d.get("target_frame"))) {
        throw std::invalid_argument("需要 target_frame 名称");
    }
    const auto& matrix = d.get("T_imu_target");
    if (!matrix.is_null()) {
        validate_transform(matrix);
        if (d.get("target_frame").as_string() == "imu") {
            throw std::invalid_argument("输出新坐标外参时请选择不同于 imu 的目标名称");
        }
    }
    const auto& aux = d.get("auxiliary_streams");
    if (!aux.is_null()) {
        if (!aux.is_array()) throw std::invalid_argument("auxiliary_streams 必须为数组");
        for (const auto& stream : aux.as_array()) {
            if (!stream.is_object() ||
                !is_nonempty_string(stream.get("name")) ||
                !is_nonempty_string(stream.get("kind")) ||
                !is_nonempty_string(stream.get("path")) ||
                !is_nonempty_string(stream.get("timestamp_clock"))) {
                throw std::invalid_argument("辅助流需要 name/kind/path/timestamp_clock");
            }
            const std::string& raw_p = stream.get("path").as_string();
            std::filesystem::path rel = std::filesystem::u8path(raw_p);
            bool has_parent_ref = false;
            for (const auto& part : rel) {
                if (part == "..") has_parent_ref = true;
            }
            if (rel.is_absolute() || has_parent_ref ||
                raw_p.find(':') != std::string::npos ||
                raw_p.find('\\') != std::string::npos) {
                throw std::invalid_argument("辅助流 path 必须为采集目录内的相对路径，使用 /");
            }
        }
    }
    const auto& sm = d.get("sensor_mode");
    if (!sm.is_null()) {
        if (!sm.is_string() || !kValidSensorModes.count(sm.as_string())) {
            throw std::invalid_argument("未知传感器模式 sensor_mode");
        }
    }
    return d;
}

void validate_slam_sensor_mode(const JsonValue& device) {
    const auto& mode = device.get("sensor_mode");
    if (!mode.is_null()) {
        if (!mode.is_string() || mode.as_string() != "stereo_imu") {
            throw std::invalid_argument("当前 SLAM 后端仅支持双目+IMU (stereo_imu)，不支持传感器模式：" +
                                        (mode.is_string() ? mode.as_string() : std::string("unknown")));
        }
    }
    const auto& en_imu = device.get("enable_imu");
    if (en_imu.is_bool() && !en_imu.as_bool()) {
        throw std::invalid_argument("当前 SLAM 后端需要启用 IMU (enable_imu=true)，暂不支持纯视觉建图");
    }
    const auto& num_cams = device.get("num_cameras");
    if (!num_cams.is_null() && num_cams.is_int() && num_cams.as_int() != 2) {
        throw std::invalid_argument("当前 SLAM 后端仅支持双目相机 (num_cameras=2)，当前配置为 " +
                                    std::to_string(num_cams.as_int()));
    }
}

JsonValue pipeline_profile(const JsonValue& device) {
    JsonValue out = JsonValue::object();
    for (const char* k : {"adapter", "imu_hz", "accel_range_g", "gyro_range_dps", "camera_to_imu_offset_ns"}) {
        if (device.contains(k)) {
            out[k] = device.get(k);
        }
    }
    return out;
}

void check_dataset(const std::filesystem::path& source_in, const JsonValue& device) {
    validate_slam_sensor_mode(device);
    std::filesystem::path source = std::filesystem::weakly_canonical(source_in);
    if (!std::filesystem::is_directory(source)) {
        throw std::invalid_argument("采集目录不存在");
    }
    std::vector<std::string> required;
    if (device.get("adapter").as_string() == "umi_ms901m_uniform") {
        required = {"stereo.mkv", "umi_video_timestamps.csv", "umi_imu_timestamps.bin"};
    } else {
        if (std::filesystem::exists(source / "stereo.mkv")) {
            throw std::invalid_argument("原始 UMI 视频不能选择 EuRoC 适配器");
        }
        required = {"mav0/imu0/data.csv"};
        if (!std::filesystem::is_regular_file(source / "timestamps.txt")) {
            required.push_back("mav0/cam0/data.csv");
            required.push_back("mav0/cam1/data.csv");
        }
        for (const char* cam : {"cam0", "cam1"}) {
            if (!std::filesystem::is_directory(source / "mav0" / cam / "data")) {
                throw std::invalid_argument(std::string("缺少 ") + cam + " 图像目录");
            }
        }
    }
    for (const auto& f : required) {
        if (!std::filesystem::is_regular_file(source / std::filesystem::u8path(f))) {
            throw std::invalid_argument("缺少输入文件：" + f);
        }
    }
    const auto& aux = device.get("auxiliary_streams");
    if (aux.is_array()) {
        for (const auto& stream : aux.as_array()) {
            std::filesystem::path p = source / std::filesystem::u8path(stream.get("path").as_string());
            if (!is_relative_subpath(p, source) || !std::filesystem::is_regular_file(p)) {
                throw std::invalid_argument("辅助流文件缺失或超出采集目录");
            }
        }
    }
}

std::optional<std::filesystem::path> export_target(
    const std::filesystem::path& result_dir,
    const JsonValue& device) {
    const auto& mat_val = device.get("T_imu_target");
    if (mat_val.is_null()) return std::nullopt;

    Eigen::Matrix4d T_imu_target = validate_transform(mat_val);
    std::filesystem::path csv_in = result_dir / "imu_trajectory.csv";
    std::ifstream f(csv_in);
    if (!f) throw std::runtime_error("Missing imu_trajectory.csv in " + result_dir.string());

    std::string header_line;
    if (!std::getline(f, header_line)) throw std::invalid_argument("没有可导出的轨迹");
    std::vector<std::string> fields = split_csv_line(header_line);
    auto col_idx = [&](const std::string& name) -> int {
        for (size_t i = 0; i < fields.size(); ++i) {
            if (fields[i] == name) return static_cast<int>(i);
        }
        throw std::invalid_argument("Missing column in imu_trajectory.csv: " + name);
    };
    int ix = col_idx("x_m"), iy = col_idx("y_m"), iz = col_idx("z_m");
    int iqx = col_idx("qx"), iqy = col_idx("qy"), iqz = col_idx("qz"), iqw = col_idx("qw");

    std::vector<std::vector<std::string>> rows;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::vector<std::string> row = split_csv_line(line);
        if (row.size() < fields.size()) continue;
        Eigen::Vector3d p(std::stod(row[ix]), std::stod(row[iy]), std::stod(row[iz]));
        Eigen::Vector4d q(std::stod(row[iqx]), std::stod(row[iqy]), std::stod(row[iqz]), std::stod(row[iqw]));
        if (!p.allFinite() || !q.allFinite()) {
            throw std::invalid_argument("轨迹包含非有限数值");
        }
        Eigen::Matrix3d R_wi = quaternion_to_rot_strict(q);
        Eigen::Vector3d p_wt = p + R_wi * T_imu_target.topRightCorner<3, 1>();
        Eigen::Matrix3d R_wt = R_wi * T_imu_target.topLeftCorner<3, 3>();
        Eigen::Vector4d q_wt = rot_to_quat_xyzw(R_wt);

        row[ix] = format_double(p_wt.x());
        row[iy] = format_double(p_wt.y());
        row[iz] = format_double(p_wt.z());
        row[iqx] = format_double(q_wt.x());
        row[iqy] = format_double(q_wt.y());
        row[iqz] = format_double(q_wt.z());
        row[iqw] = format_double(q_wt.w());
        rows.push_back(std::move(row));
    }
    if (rows.empty()) throw std::invalid_argument("没有可导出的轨迹");

    std::filesystem::path target_csv = result_dir / "target_trajectory.csv";
    std::filesystem::path target_json = result_dir / "target_frame.json";
    if (std::filesystem::exists(target_csv) || std::filesystem::exists(target_json)) {
        throw std::runtime_error("Target trajectory export already exists; refusing to overwrite");
    }

    {
        std::ofstream out(target_csv, std::ios::binary);
        if (!out) throw std::runtime_error("Cannot create " + target_csv.string());
        for (size_t i = 0; i < fields.size(); ++i) {
            if (i > 0) out << ',';
            out << fields[i];
        }
        out << '\n';
        for (const auto& row : rows) {
            for (size_t i = 0; i < row.size(); ++i) {
                if (i > 0) out << ',';
                out << row[i];
            }
            out << '\n';
        }
    }

    JsonValue meta = JsonValue::object();
    meta["target_frame"] = device.get("target_frame");
    meta["T_imu_target"] = mat_val;
    meta["composition"] = "T_world_target = T_world_imu @ T_imu_target";
    meta["position_unit"] = "m";
    meta["quaternion_order"] = "xyzw";
    meta["timestamp_clock"] = device.get("timestamp_clock");
    meta["source"] = "imu_trajectory.csv";
    meta["note"] = "Same SLAM world frame, timestamps and segment IDs; no interpolation.";
    std::ofstream jout(target_json, std::ios::binary);
    if (!jout) throw std::runtime_error("Cannot create " + target_json.string());
    jout << meta.dump(2);

    return target_csv;
}

std::vector<TrajectoryPoint> read_csv_points(const std::filesystem::path& path) {
    std::ifstream f(path);
    if (!f) return {};
    std::string header_line;
    if (!std::getline(f, header_line)) return {};
    auto headers = split_csv_line(header_line);
    int ix = -1, iy = -1, iz = -1, iseg = -1;
    for (size_t i = 0; i < headers.size(); ++i) {
        if (headers[i] == "x_m") ix = static_cast<int>(i);
        else if (headers[i] == "y_m") iy = static_cast<int>(i);
        else if (headers[i] == "z_m") iz = static_cast<int>(i);
        else if (headers[i] == "segment_id") iseg = static_cast<int>(i);
    }
    if (ix < 0 || iy < 0 || iz < 0) return {};

    std::vector<TrajectoryPoint> pts;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        auto row = split_csv_line(line);
        if (static_cast<int>(row.size()) <= std::max({ix, iy, iz})) continue;
        try {
            Eigen::Vector3d p(std::stod(row[ix]), std::stod(row[iy]), std::stod(row[iz]));
            if (p.allFinite()) {
                std::string seg = (iseg >= 0 && iseg < static_cast<int>(row.size())) ? row[iseg] : "0";
                pts.push_back({p, seg});
            }
        } catch (...) {
        }
    }
    return pts;
}

std::vector<TrajectoryPoint> read_tum_points(const std::filesystem::path& path) {
    std::ifstream f(path);
    if (!f) return {};
    std::vector<TrajectoryPoint> pts;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::string ts;
        double x = 0.0, y = 0.0, z = 0.0;
        if (!(ss >> ts) || ts.empty() || ts[0] == '#') continue;
        if (!(ss >> x >> y >> z)) continue;
        Eigen::Vector3d p(x, y, z);
        if (p.allFinite()) pts.push_back({p, "0"});
    }
    return pts;
}

std::pair<std::vector<Eigen::Vector3d>, size_t> read_map_points(
    const std::filesystem::path& path,
    size_t max_points) {
    std::ifstream f(path);
    if (!f) return {{}, 0};
    std::vector<Eigen::Vector3d> all_pts;
    std::string line;
    bool first = true;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        auto row = split_csv_line(line);
        if (first) {
            first = false;
            if (!row.empty()) {
                std::string h0 = row[0];
                std::transform(h0.begin(), h0.end(), h0.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (h0.find('x') != std::string::npos || h0.find('#') != std::string::npos) {
                    continue;
                }
            }
        }
        if (row.size() >= 3) {
            try {
                Eigen::Vector3d p(std::stod(row[0]), std::stod(row[1]), std::stod(row[2]));
                if (p.allFinite()) all_pts.push_back(p);
            } catch (...) {
            }
        }
    }
    size_t total = all_pts.size();
    if (max_points > 0 && total > max_points) {
        size_t step = std::max<size_t>(1, total / max_points);
        std::vector<Eigen::Vector3d> decimated;
        decimated.reserve(max_points);
        for (size_t i = 0; i < total && decimated.size() < max_points; i += step) {
            decimated.push_back(all_pts[i]);
        }
        all_pts = std::move(decimated);
    }
    return {std::move(all_pts), total};
}

PreviewData load_preview_data(const std::filesystem::path& path, size_t max_map_points) {
    std::filesystem::path root = std::filesystem::weakly_canonical(path);
    if (!std::filesystem::is_directory(root)) {
        throw std::invalid_argument("所选目录不存在");
    }
    const std::array<const char*, 4> traj_names = {
        "imu_trajectory.csv", "imu_trajectory.tum", "imu_euroc.txt", "target_trajectory.csv"
    };
    bool has_result_traj = false;
    if (std::filesystem::is_directory(root / "result")) {
        for (const char* n : traj_names) {
            if (std::filesystem::is_regular_file(root / "result" / n)) {
                has_result_traj = true;
                break;
            }
        }
    }
    std::filesystem::path data_dir = has_result_traj ? (root / "result") : root;
    std::filesystem::path job_dir = has_result_traj
        ? root
        : (std::filesystem::is_regular_file(root.parent_path() / "station_job.json") ? root.parent_path() : root);

    PreviewData out;
    out.dir = data_dir.string();
    out.job_dir = job_dir.string();

    if (std::filesystem::is_regular_file(data_dir / "imu_trajectory.csv")) {
        out.points = read_csv_points(data_dir / "imu_trajectory.csv");
        out.traj_source = "imu_trajectory.csv";
    } else if (std::filesystem::is_regular_file(data_dir / "imu_trajectory.tum")) {
        out.points = read_tum_points(data_dir / "imu_trajectory.tum");
        out.traj_source = "imu_trajectory.tum";
    } else if (std::filesystem::is_regular_file(data_dir / "imu_euroc.txt")) {
        out.points = read_tum_points(data_dir / "imu_euroc.txt");
        out.traj_source = "imu_euroc.txt";
    }

    if (std::filesystem::is_regular_file(data_dir / "target_trajectory.csv")) {
        out.target_points = read_csv_points(data_dir / "target_trajectory.csv");
        if (out.points.empty()) {
            out.points = out.target_points;
            out.traj_source = "target_trajectory.csv";
        }
    }

    if (out.points.empty()) {
        throw std::invalid_argument("目录中未找到有效轨迹数据（需要 imu_trajectory.csv / imu_trajectory.tum / imu_euroc.txt）");
    }

    if (std::filesystem::is_regular_file(data_dir / "keyframes_euroc.txt")) {
        auto kfs = read_tum_points(data_dir / "keyframes_euroc.txt");
        out.keyframes.reserve(kfs.size());
        for (const auto& tp : kfs) out.keyframes.push_back(tp.pos);
    }
    if (std::filesystem::is_regular_file(data_dir / "map_points.csv")) {
        auto mp = read_map_points(data_dir / "map_points.csv", max_map_points);
        out.map_points = std::move(mp.first);
        out.total_map_points = mp.second;
    }

    out.quality = read_json_safe(data_dir / "quality.json");
    out.station_job = read_json_safe(job_dir / "station_job.json");
    JsonValue target_meta = read_json_safe(data_dir / "target_frame.json");
    if (target_meta.get("target_frame").is_string()) {
        out.target_frame = target_meta.get("target_frame").as_string();
    }
    return out;
}

}  // namespace slam_common
