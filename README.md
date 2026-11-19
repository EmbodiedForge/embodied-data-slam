# UMI & EGO SLAM Station（C++17 / Dear ImGui / Conan 2 跨平台原生版）

面向 UMI 手持夹爪与 EGO 第一视角设备的 **纯 C++17 + Dear ImGui + Conan 2** 跨平台（Windows MSVC / Linux / macOS）离线双目视觉惯性（Stereo-Inertial）轨迹恢复与全功能免 ROS 标定工作站。

- Windows 独立包为**全静态单文件分发**（0 第三方 DLL，脱离 MSYS2、Qt6 与 Docker）。
- WSL2 / Ubuntu Linux 构建与测试请先阅读 [WSL_LINUX_BUILD.md](WSL_LINUX_BUILD.md)；更多背景说明见 [README_先读我.md](README_先读我.md) 与 [WINDOWS_上位机说明.md](WINDOWS_上位机说明.md)。

## 目录结构

| 路径 | 内容 |
|---|---|
| `conanfile.py`、`CMakeLists.txt`、`cmake/` | Conan 2 跨平台依赖声明与 CMake 根工程（支持 `EDS_BUILD_STATION=ON/OFF` 与 Windows UTF-8 清单合并） |
| `include/`、`src/slam_common/` | 叶子基础库：SHA-256 与校验和、JSON、刚体几何与 2D 拟合、MS901M 串口解码、轨迹 I/O（仅依赖标准库 + Eigen3） |
| `src/calibration_targets/` | 标定板角点/圆阵列检测与 1:1 物理尺寸 PDF / SVG / PNG 生成 |
| `src/camera_imu_calibration/` | 免 ROS 单目/双目/多目 + IMU 联合标定与 Kalibr YAML 转换 |
| `src/handeye_calibration/` | VisionMaster 风格 2D N 点标定/旋转中心/对位与 3D `AX=XB` / `AX=ZB` 手眼标定 |
| `src/vision_calibration/` | 2D 平面标定、像素↔物理单位换算、径向/透视畸变校正与双相机映射 |
| `src/preview_3d/` | Dear ImGui `ImDrawList` 交互式 3D 轨道点云/轨迹视图与 XY/XZ/YZ 三正交投影视图 |
| `src/umi_slam/` | 纯原生 SLAM 预处理/后处理管线、进程树取消与隔离式 `--build-backend`（`umi_slam_pipeline`） |
| `src/app/` | Dear ImGui（Docking + GLFW + OpenGL3）主界面、三端原生文件弹窗、CLI（`--cli` / `--run-pipeline` / `--calibrator`）与标定工作台（`slam_station`） |
| `tests/` | CTest 原生测试套件（`test_pipeline_cpp`、`test_station_cpp`） |
| `software/umi_slam/` | ORB-SLAM3 C++ 运行器（`backend/`）、设备配置（`profiles/`）及历史 Docker 参考文件（保留未删除） |
| `software/slam_station/` | 设备接口模板（`devices/`）、接口文档（`DEVICE_API.md`）与验证记录（`VALIDATION.md`） |
| `calibration/` | 传统 ROS1 Kalibr 参考脚本（`kalibr.sh`）、标定板模板与 `reference_20260908` 基准（保留作参考） |
| `source/stereo_imu_trajectory/` | `ORB_SLAM3`（GPLv3）与 `Pangolin`（MIT）源码树（补丁在隔离构建目录注入，不改原树逻辑） |

## 快速构建与启动

### Windows（MSVC + Conan 2 + Ninja + NSIS）

前置要求：Visual Studio 2022/2026 Build Tools（含 C++ 桌面开发）、`conan`（2.x）、`cmake`、`ninja`；如需生成商业友好的单文件安装程序，安装 `nsis`（`scoop install extras/nsis`）。

```powershell
# 检查 MSVC + Conan + CMake + Ninja 环境
powershell -ExecutionPolicy Bypass -File .\build_windows.ps1 -CheckOnly

# 清理旧编译与 dist 产物（保留 ~/.conan2 缓存）
powershell -ExecutionPolicy Bypass -File .\build_windows.ps1 -CleanOnly

# 一键拉取/构建 Conan 依赖（含纯 LGPL FFmpeg 内置解码）、MSVC 编译、刷新校验和、运行 CTest、构建 ORB-SLAM3 后端并打包
powershell -ExecutionPolicy Bypass -File .\build_windows.ps1
```

产物位于 `dist\SLAMStation\`（仅包含 `slam_station.exe`、`umi_slam_pipeline.exe`、`backend\umi_slam_runner.exe`、`ORBvoc.txt` 及配置，**0 第三方 DLL**）；检测到 NSIS 时会自动生成可对外商用分发的单文件安装程序 `dist\SLAMStation-Setup-2.0.0-x64.exe`（内置并静默安装 `vc_redist.x64.exe`，安装完成即可直接使用）。

### Linux / WSL2（GCC/Clang + Conan 2 + Ninja）

详见 [WSL_LINUX_BUILD.md](WSL_LINUX_BUILD.md)：

```bash
# 全量构建（含 Dear ImGui 桌面端、CTest 与 ORB-SLAM3 原生后端），产物输出至 dist-linux/SLAMStation/
bash build_linux.sh

# 清理旧构建与 dist-linux 产物 / 清理后重编
bash build_linux.sh --clean-only
bash build_linux.sh --clean

# 无头模式（仅编译 umi_slam_pipeline 与 ORB-SLAM3 后端，不编 GUI）
EDS_BUILD_STATION=OFF bash build_linux.sh
```

### macOS（AppleClang + Conan 2 + Ninja）

```bash
bash build_macos.sh
bash build_macos.sh --clean-only
```

## 第三方许可证

- 上位机 GUI（`Dear ImGui`、`GLFW`）采用 **MIT / Zlib** 许可证，允许静态链接商业闭源。
- 独立子进程 `umi_slam_runner` 静态链接 **GPLv3** 的 `ORB_SLAM3`（与上位机 `slam_station` 进程隔离）；对外分发 `umi_slam_runner` 时，请遵循 GPLv3 源码提供义务。
