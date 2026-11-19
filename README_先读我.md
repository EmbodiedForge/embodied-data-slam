# UMI 轨迹恢复与 Kalibr 离线工具包（C++17 / Qt6 版）

面向 Windows 10/11（MSYS2 CLANG64）与 Ubuntu 22.04+（x86_64）的离线双目视觉惯性（Stereo-Inertial）轨迹恢复与免 ROS 标定工作站。软件默认输出 **IMU 原点位姿**；在设备配置中提供有效刚体旋转平移外参 `T_imu_target` 时，会同步导出目标坐标系轨迹 `target_trajectory.csv`。

---

## 一、原生编译、独立打包与启动

### 1. Windows（MSYS2 CLANG64 一键构建）

在仓库根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\build_windows.ps1 --check-only
powershell -ExecutionPolicy Bypass -File .\build_windows.ps1
```

构建完成后生成独立免安装目录 `dist\SLAMStation\`（内含 `slam_station.exe`、`umi_slam_pipeline.exe`、Qt6 运行时、`backend\umi_slam_runner.exe`、`ORBvoc.txt` 与模板配置），直接双击 `launch_windows.cmd` 或 `launch_calibrator.cmd` 启动。

### 2. Ubuntu Linux 原生构建

```bash
bash build_linux.sh
bash launch_station.sh
```

在 WSL2 的 `/mnt/*` 路径下执行 `build_linux.sh` 时，构建缓存自动放入 `~/.cache/eds-cpp-*` 且独立包输出至 `dist-linux/SLAMStation/`，不会覆盖 Windows 产物。

### 3. 纯后端（无 Qt6）构建与命令行使用

适用于无桌面环境或 Docker 容器：

```bash
cmake -S . -B build-cli -G Ninja -DCMAKE_BUILD_TYPE=Release -DEDS_BUILD_STATION=OFF
cmake --build build-cli -j 4
./build-cli/umi_slam_pipeline --build-backend --jobs 4
./build-cli/umi_slam_pipeline \
  --input examples/lissajous_20260915/input \
  --output results/lissajous_run001 \
  --settings software/umi_slam/profiles/umi_20260908.yaml \
  --profile software/umi_slam/profiles/umi_20260908.json \
  --backend software/umi_slam/.build-linux/backend.json \
  --stride 5
```

完整上位机 CLI 用法：

```bash
slam_station --cli --check --mode auto
slam_station --cli \
  --input examples/lissajous_20260915/input \
  --output results/run001 \
  --settings software/umi_slam/profiles/umi_20260908.yaml \
  --device software/slam_station/devices/umi_20260908.json \
  --stride 5
```

---

## 二、纯 C++ Docker 后端（`umi-slam:0.1`）

本版本已将 `software/umi_slam/Dockerfile` 升级为纯 C++ 构建与运行镜像（不再依赖镜像内 Python）：

```bash
# 从仓库根目录构建
BUILD_JOBS=4 bash software/umi_slam/build_docker.sh

# 若使用离线镜像归档（需自备 images/*.tar.gz）
bash load_images.sh
```

---

## 三、Kalibr 容器标定流程（保留 ROS1 辅助脚本）

`calibration/` 保留了在 `kalibr:portable-20260921` 容器内运行的 ROS1 `.bag` 转换与标定封装脚本（`calibration/tools/kalibr_to_bag.py`、`calibration/tools/kalibr_to_orb.py`）。若需与免 ROS 标定工作台交叉验证：

1. 生成 ROS1 `.bag`：
   ```bash
   bash calibration/kalibr.sh "$HOME/calibration_work" bag \
     --input /work/session --output /work/dataset.bag --stride 5 \
     --imu-hz 250 --accel-range-g 4 --gyro-range-dps 2000
   ```
2. 标定相机内参与双目外参：
   ```bash
   bash calibration/kalibr.sh "$HOME/calibration_work" stereo \
     --bag /work/dataset.bag --target /work/target.yaml \
     --models pinhole-radtan pinhole-radtan \
     --topics /cam0/image_raw /cam1/image_raw
   ```
3. 联合标定相机—IMU 外参与时延：
   ```bash
   bash calibration/kalibr.sh "$HOME/calibration_work" imu \
     --bag /work/dataset.bag --target /work/target.yaml \
     --cams /work/dataset-camchain.yaml --imu /work/imu.yaml
   ```
4. 转换为 ORB-SLAM3 配置（既可在容器内执行 `convert`，也可直接在 `slam_station` 标定工作台点击“从 Kalibr YAML 免 ROS 转换”）：
   ```bash
   bash calibration/kalibr.sh "$HOME/calibration_work" convert \
     --cam /work/dataset-camchain-imucam.yaml \
     --imu /work/imu.yaml --output-dir /work/slam_config --fps 25 \
     --capture-profile /toolkit/software/umi_slam/profiles/umi_20260908.json
   ```

---

## 四、标定板尺寸与校验和说明

- 用户已确认当前实物标签边长为 **34.5 mm**，对应配置文件 `calibration/targets/aprilgrid_6x6_measured34p5mm.yaml`（`tagSize=0.0345`）。`aprilgrid_6x6_nominal40mm_A3.pdf` 文件名中的 `40mm` 仅作历史参考，请以实体黑框实测边长为准。
- 校验仓库文件哈希或刷新 `SHA256SUMS`：
  ```bash
  umi_slam_pipeline --verify-checksums
  umi_slam_pipeline --update-checksums
  ```
- 本工具主要使用 CPU 计算；轨迹覆盖率指标仅表示已初始化且跟踪正常的帧占比，不构成绝对定位精度认证。
