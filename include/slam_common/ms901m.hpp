#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace slam_common {

constexpr size_t kMaxUartBlockBytes = 16u * 1024u * 1024u;
constexpr double kStandardGravityMS2 = 9.80665;
constexpr double kInt16FullScale = 32768.0;

struct ImuSeries {
    std::vector<int64_t> timestamps_ns;
    /// Each row stores [gx, gy, gz, ax, ay, az] in rad/s and m/s^2.
    std::vector<Eigen::Matrix<double, 6, 1>> samples;
};

/// Decode MS901M 0x5555 Type-03 UART arrival blocks and reconstruct a uniform nanosecond time axis.
ImuSeries decode_ms901m_bin(
    const std::filesystem::path& path,
    double hz = 250.0,
    double accel_g = 4.0,
    double gyro_dps = 2000.0);

/// Read EuRoC / prepared IMU CSV (#timestamp_ns, gx, gy, gz, ax, ay, az).
ImuSeries read_imu_csv(const std::filesystem::path& path);

}  // namespace slam_common
