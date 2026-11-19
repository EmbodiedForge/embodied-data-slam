#include "slam_common/ms901m.hpp"

#include "slam_common/sha256.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace slam_common {
namespace {

constexpr double kPi = 3.14159265358979323846;

uint64_t read_le_u64(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= static_cast<uint64_t>(p[i]) << (8 * i);
    return v;
}

uint32_t read_le_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

int16_t read_le_i16(const uint8_t* p) {
    uint16_t u = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    int16_t s = 0;
    std::memcpy(&s, &u, sizeof(s));
    return s;
}

}  // namespace

ImuSeries decode_ms901m_bin(
    const std::filesystem::path& path,
    double hz,
    double accel_g,
    double gyro_dps) {
    if (!(std::isfinite(hz) && hz > 0.0 &&
          std::isfinite(accel_g) && accel_g > 0.0 &&
          std::isfinite(gyro_dps) && gyro_dps > 0.0)) {
        throw std::invalid_argument("IMU frequency and ranges must be positive finite numbers");
    }

    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open IMU binary file: " + path.string());

    std::vector<uint8_t> raw;
    std::vector<std::pair<uint64_t, size_t>> blocks;
    uint8_t hdr[12];
    while (true) {
        f.read(reinterpret_cast<char*>(hdr), 12);
        std::streamsize got = f.gcount();
        if (got == 0) break;
        if (got != 12) throw std::invalid_argument("Truncated IMU block header");
        uint64_t ts = read_le_u64(hdr);
        uint32_t size = read_le_u32(hdr + 8);
        if (size > kMaxUartBlockBytes) throw std::invalid_argument("Invalid IMU block length");
        size_t old_sz = raw.size();
        raw.resize(old_sz + size);
        if (size > 0) {
            f.read(reinterpret_cast<char*>(raw.data() + old_sz), static_cast<std::streamsize>(size));
            if (f.gcount() != static_cast<std::streamsize>(size)) {
                throw std::invalid_argument("Truncated IMU block payload");
            }
        }
        blocks.emplace_back(ts, raw.size());
    }
    if (blocks.empty()) throw std::invalid_argument("Empty IMU stream");

    std::vector<std::array<int16_t, 6>> raw_samples;
    std::vector<size_t> ends;
    size_t invalid = 0;
    size_t i = 0;
    while (i + 5 <= raw.size()) {
        if (raw[i] != 0x55 || raw[i + 1] != 0x55) {
            ++i;
            continue;
        }
        size_t end = i + 5 + static_cast<size_t>(raw[i + 3]);
        if (end > raw.size()) break;
        uint32_t csum = 0;
        for (size_t k = i; k < end - 1; ++k) csum += raw[k];
        if ((csum & 0xFFu) != raw[end - 1]) {
            ++invalid;
            ++i;
            continue;
        }
        if (raw[i + 2] == 3 && raw[i + 3] == 12) {
            std::array<int16_t, 6> rec{};
            for (int c = 0; c < 6; ++c) {
                rec[c] = read_le_i16(&raw[i + 4 + c * 2]);
            }
            raw_samples.push_back(rec);
            ends.push_back(end);
        }
        i = end;
    }

    if (raw_samples.size() < 2) {
        throw std::invalid_argument("No usable MS901M samples");
    }
    if (invalid > 0) {
        throw std::invalid_argument(std::to_string(invalid) + " corrupt IMU packets; cannot safely reconstruct a uniform time axis");
    }

    std::pair<uint64_t, size_t> first{0, 0};
    bool found_first = false;
    for (const auto& blk : blocks) {
        if (blk.second >= ends[0]) {
            first = blk;
            found_first = true;
            break;
        }
    }
    if (!found_first) throw std::invalid_argument("No usable MS901M samples");

    size_t n_in_first = 0;
    for (size_t e : ends) {
        if (e <= first.second) ++n_in_first;
    }
    int64_t t0 = static_cast<int64_t>(first.first) -
                 static_cast<int64_t>(std::llround(static_cast<double>(n_in_first - 1) * 1e9 / hz));

    const double acc_scale = accel_g * kStandardGravityMS2 / kInt16FullScale;
    const double gyro_scale = gyro_dps * kPi / 180.0 / kInt16FullScale;

    ImuSeries out;
    out.timestamps_ns.resize(raw_samples.size());
    out.samples.resize(raw_samples.size());
    for (size_t k = 0; k < raw_samples.size(); ++k) {
        out.timestamps_ns[k] = t0 + static_cast<int64_t>(std::llround(static_cast<double>(k) * 1e9 / hz));
        Eigen::Matrix<double, 6, 1> v;
        v(0) = static_cast<double>(raw_samples[k][3]) * gyro_scale;
        v(1) = static_cast<double>(raw_samples[k][4]) * gyro_scale;
        v(2) = static_cast<double>(raw_samples[k][5]) * gyro_scale;
        v(3) = static_cast<double>(raw_samples[k][0]) * acc_scale;
        v(4) = static_cast<double>(raw_samples[k][1]) * acc_scale;
        v(5) = static_cast<double>(raw_samples[k][2]) * acc_scale;
        out.samples[k] = v;
    }
    return out;
}

ImuSeries read_imu_csv(const std::filesystem::path& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open IMU CSV: " + path.string());
    ImuSeries out;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        size_t first = line.find_first_not_of(" \t");
        if (first == std::string::npos || line[first] == '#') continue;
        std::vector<std::string> cols;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) cols.push_back(cell);
        if (cols.size() != 7) throw std::invalid_argument("IMU CSV needs timestamp_ns,gx,gy,gz,ax,ay,az");
        int64_t ts = integer_ns(cols[0]);
        Eigen::Matrix<double, 6, 1> row;
        for (int j = 0; j < 6; ++j) {
            row(j) = std::stod(cols[1 + j]);
        }
        out.timestamps_ns.push_back(ts);
        out.samples.push_back(row);
    }
    return out;
}

}  // namespace slam_common
