# 项目概览（`project.md`）

## 背景与目标
`embodied-data-slam-cpp` 是 `embodied-data-slam`（SLAM Station）的纯 C++17 / Qt6 Widgets 移植版，面向 UMI 手持夹爪与 EGO 第一视角采集设备，提供无需宿主机 Python/ROS 的离线双目视觉惯性（Stereo-Inertial）轨迹恢复与全功能标定桌面工作站。

## 技术栈
- **语言与构建**：C++17、CMake 3.20+、Ninja（支持 `EDS_BUILD_STATION=ON/OFF` 开关）。
- **核心库**：Eigen3、OpenCV（Windows MSYS2 CLANG64 下为 5.0.0，Ubuntu 下为 4.5.4/4.10.0）、Qt6 Widgets（仅 `preview_3d` 与 `src/app`）、Boost.Serialization + OpenSSL + Pangolin（ORB-SLAM3 后端）。
- **双后端运行**：
  1. 本地原生后端：`umi_slam_pipeline --build-backend` 隔离编译 `ORB_SLAM3` + `Pangolin` + `umi_slam_runner`，生成可重定位 `backend.json`。
  2. Docker 后端：纯 C++ Ubuntu 22.04 镜像 `umi-slam:0.1`（`software/umi_slam/Dockerfile`）。
- **Kalibr 兼容**：保留 `calibration/kalibr.sh` 与容器内 ROS1 Python 辅助脚本，同时在 C++ `camera_imu_calibration` 中提供免 ROS Kalibr YAML 直接转换。
