# UMI SLAM 后处理工具包（UMI_Toolkit_20260921）

对 UMI 采集的双目 + IMU 数据做离线轨迹恢复（基于打过补丁的 ORB-SLAM3），并附带 Kalibr 标定流程的封装脚本。输出为 IMU 原点位姿；夹爪中心的外参尚未加入。

详细使用说明见 [README_先读我.md](README_先读我.md)（Ubuntu）和 [WINDOWS_上位机说明.md](WINDOWS_上位机说明.md)（Windows 上位机）。

## 目录

| 路径 | 内容 |
|---|---|
| `software/umi_slam` | 轨迹恢复主流程（`pipeline.py`）、C++ 后端（`backend/runner.cc`）、Docker 入口、桌面界面、测试 |
| `software/slam_station` | SLAM Station 0.2 上位机（任务管理、设备模板、报告） |
| `software/ubuntu_viewer` | Ubuntu 结果查看界面 |
| `calibration/` | Kalibr 封装脚本、原始数据转 bag、Kalibr 结果转 ORB-SLAM3 配置、标定板与 2026-09-08 参考标定参数 |
| `source/stereo_imu_trajectory/ORB_SLAM3` | 打过补丁的 ORB-SLAM3 源码（改动在 `src/Tracking.cc`、`src/LocalMapping.cc`、`src/Settings.cc`、`include/Settings.h`、`CMakeLists.txt`） |
| `source/stereo_imu_trajectory/Pangolin` | ORB-SLAM3 依赖的 Pangolin 源码 |
| `source/portable_toolkit/assemble.py` | 原工具包的组装脚本 |

## 本仓库没有包含的内容

原工具包约 16 GB，这里只保留代码和小体积配置。以下内容没有上传，需要时找郑佳桐拷贝：

- `images/`：离线 Docker 镜像（`umi-slam:0.1`、`kalibr:portable-20260921`，共约 2.3 GB）
- `examples/`：Lissajous 示例采集数据、参考结果和标定 bag（约 4.2 GB）
- `results/`、`validation_station/`：历史运行结果
- `releases/`：SLAM Station Windows 离线包
- `calibration/reference_20260908/stereo_imu-report-imucam.pdf`：Kalibr 标定报告（4.6 MB）
- ORB-SLAM3 自带的 EuRoC 评测真值（`evaluation/Ground_truth`）

因此 `load_images.sh` 和文档里引用 `examples/` 的命令，需要先拿到上述文件才能运行；`SHA256SUMS` 和 `MANIFEST.json` 描述的是完整工具包。

## 从源码重建 SLAM 镜像

`software/umi_slam/Dockerfile` 要求构建上下文里 `umi_slam` 和 `stereo_imu_trajectory` 并列：

```bash
mkdir build_ctx
cp -r software/umi_slam build_ctx/umi_slam
cp -r source/stereo_imu_trajectory build_ctx/stereo_imu_trajectory
docker build -f build_ctx/umi_slam/Dockerfile -t umi-slam:0.1 build_ctx
```

此重建流程在整理本仓库时没有重新验证。

## 第三方代码

ORB-SLAM3 为 GPLv3，Pangolin 为 MIT，各自的许可证文件保留在对应目录内。
