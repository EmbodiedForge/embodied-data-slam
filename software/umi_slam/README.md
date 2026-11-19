# UMI 双目 IMU 轨迹后处理后端（C++17 原生与 Docker 管线）

`umi_slam_pipeline`（由 `src/umi_slam/` 编译生成）与 `software/umi_slam/backend/runner.cc` 共同构成离线双目惯性 SLAM 数据预处理、ORB-SLAM3 执行与轨迹导出管线。现阶段默认输出 **IMU 原点位姿**；当上层传入合法 `T_imu_target` 外参时由上位机导出目标坐标系轨迹。

## 当前支持

- **现有 UMI**：`stereo.mkv`、`umi_video_timestamps.csv`、`umi_imu_timestamps.bin`（MS901M 串口协议）。
- **EuRoC 风格目录**：左右图像、严格对应纳秒时间戳、IMU CSV（PNG/JPG）。
- **数据校验与安全边界**：时间严格单调性检查、标定分辨率一致性、IMU 频率/量程/丢包缺口校验、输入输出目录隔离与防覆盖、未完成惯性初始化或崩溃严禁标为成功。
- **隔离补丁编译**：`umi_slam_pipeline --build-backend` 将 `source/stereo_imu_trajectory/` 下的 `ORB_SLAM3` 与 `Pangolin` 拷贝到独立 `.build-windows` / `.build-linux` 目录后再注入 `patch_orb()` 离线导出补丁并编译，绝不污染原源码树。

## 隔离构建后端

```bash
# Windows（在 MSYS2 CLANG64 环境或执行根目录 build_windows.ps1）
umi_slam_pipeline.exe --build-backend --jobs 8

# Linux / macOS / WSL
./umi_slam_pipeline --build-backend --jobs 4
```

生成产物位于 `software/umi_slam/.build-windows/backend.json`（或 `.build-linux/backend.json`），其内所有路径均以相对 `backend.json` 所在目录的形式存储，支持整个后端目录重定位分发。

## 命令行运行

```bash
umi_slam_pipeline \
  --input examples/lissajous_20260915/input \
  --output outputs/umi_imu_run_001 \
  --settings software/umi_slam/profiles/umi_20260908.yaml \
  --profile software/umi_slam/profiles/umi_20260908.json \
  --backend software/umi_slam/.build-linux/backend.json \
  --stride 5
```

- `--prepare-only`：仅执行数据检查与时间对齐预处理。
- `--prepared-input`：复用先前生成的 `prepared/` 目录直接重跑后端（校验 `calibration_sha256` 一致性）。
- `--viewer`：仅本地原生模式下开启 Pangolin 实时建图窗口。
- `--verify-checksums` / `--update-checksums`：校验或刷新仓库根目录 `SHA256SUMS`。

## 纯 C++ Docker 镜像

`software/umi_slam/Dockerfile` 基于 Ubuntu 22.04，在构建阶段使用 `-DEDS_BUILD_STATION=OFF` 编译 `umi_slam_pipeline` 并调用 `--build-backend` 生成内置 `ORB_SLAM3 + umi_slam_runner`：

```bash
# 从仓库根目录执行构建
BUILD_JOBS=4 bash software/umi_slam/build_docker.sh

# 运行示例（无网络、只读挂载输入与配置）
mkdir -p outputs/docker_umi
docker run --rm --init --network none \
  --user "$(id -u):$(id -g)" \
  -v "$PWD/examples/lissajous_20260915/input:/input:ro" \
  -v "$PWD/outputs/docker_umi:/output" \
  -v "$PWD/software/umi_slam/profiles:/config:ro" \
  umi-slam:0.1 \
  --input /input --output /output/run001 \
  --settings /config/umi_20260908.yaml \
  --profile /config/umi_20260908.json --stride 5
```

## 输出与坐标约定

| 文件 | 内容 |
|---|---|
| `imu_trajectory.csv` | 原图像纳秒时间戳、相对输入起点秒数、IMU 位置（米）、四元数 `qx,qy,qz,qw`、`segment_id` |
| `imu_trajectory.tum` | 时间戳换算为秒的同一组 IMU 位姿 |
| `imu_euroc.txt` | ORB-SLAM3 原始导出（主要使用过滤后的 CSV/TUM） |
| `tracking.csv` | 逐帧跟踪状态、当时惯性初始化状态、地图 ID |
| `quality.json` / `report.html` | 覆盖率、时间范围、分段与告警信息（非精度认证） |
| `job.json` / `slam.log` | 运行参数、状态与后端日志 |

## 校验与测试

```bash
ctest --test-dir build-windows --output-on-failure
```

ORB-SLAM3（GPLv3）与 Pangolin（MIT）许可证文件保留在对应源码目录中。
