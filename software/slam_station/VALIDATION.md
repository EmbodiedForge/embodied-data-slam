# SLAM Station（C++17 / Qt6 版）验证记录

本文件记录当前 C++17 / Qt6 移植版本的实测构建与测试覆盖范围。**严禁将测试通过或覆盖率指标等同于真实场景绝对定位精度认证。**

## 1. Windows（MSYS2 CLANG64 · Clang 23.1.5 / OpenCV 5.0.0 / Qt6 / Eigen）

- **主工程全量编译**：`cmake --build build-windows --clean-first` 零编译警告通过（涵盖 `slam_common`、5 个功能库、`umi_slam_core`、`umi_slam_pipeline`、`slam_station`、`test_pipeline_cpp`、`test_station_cpp`）。
- **CTest 全部测试通过（2/2 可执行测试集）**：
  - `test_pipeline_cpp`（17 项测试）：MS901M 串口分片/截断校验、整数纳秒时间戳精度、EuRoC 导出与断段保留、未初始化状态拒绝、浮点纳秒回配、相机-IMU 时延预处理、`backend.json` 相对路径重定位、`patch_orb` 补丁命中与重复打补丁拒绝、中文/空格/`&` 路径命令构造与 CLI 实测、嵌套输出拒绝、流式 SHA-256 与 CRLF 规范化、`update_repo_checksums` 规则、后端源码指纹、`stage_backend_sources` 隔离与防篡改、中文 CLI 路径进程实测返回码 1 + `job.json` 正确记录 `failed`。
  - `test_station_cpp`（18 项测试）：Windows 盘符/中文/逗号 Docker 挂载编码、Linux UID 与只读挂载、嵌套/已有输出拒绝、未知协议/单位/外参拒绝、EuRoC 拒绝旧协议、末端坐标变换与 180° 旋转往返、启动前与容器创建竞态取消、配置快照防误报、原生后端定位与孙进程树强杀 + `job.json` 状态补写、3D 预览数据加载与回退、Kalibr 2026-09-08 参考结果转换一致性、合成双目+IMU 外参标定（含旋转激励不足、±50 ms 时延边界告警、稀疏观测平移回退）、功能模块零交叉依赖检查、MS901M 解码一致性、`SHA256SUMS` 全仓校验。
- **原生后端隔离编译**：`umi_slam_pipeline.exe --build-backend --jobs 8` 成功隔离拷贝 `ORB_SLAM3` 与 `Pangolin`、注入 `patch_orb`、解压 `ORBvoc.txt`、编译 `Pangolin / DBoW2 / g2o / ORB_SLAM3 / umi_slam_runner.exe`、收集 122 个运行时 DLL 并生成可重定位 `software/umi_slam/.build-windows/backend.json`；`slam_station.exe --cli --check --mode native` 返回退出码 0。
- **独立发行打包**：`build_windows.ps1` 成功产出自包含目录 `dist/SLAMStation/`（含 `windeployqt6` 部署、DLL 收集、`backend/` 与标定/设备模板），在脱离 MSYS2 `PATH` 环境下可独立启动并完成 `--cli --check --mode native` 自检。

## 2. WSL2 / Ubuntu（纯后端无 Qt6 构建 & Docker 镜像）

- **Linux 无 Qt6 构建（`-DEDS_BUILD_STATION=OFF`）**：GCC 15.2 / OpenCV 4.10.0 / Eigen 3.4.0 编译零警告通过，`test_pipeline_cpp` 全部通过（注：因 WSL2 环境未安装 Qt6 开发包，`build_linux.sh` 完整 Qt6 桌面打包需在带 `qt6-base-dev` 的 Ubuntu 桌面/构建机执行）。
- **纯 C++ Docker 镜像（`umi-slam:0.1`）**：`BUILD_JOBS=8 bash software/umi_slam/build_docker.sh` 在 Ubuntu 22.04 容器内完整编译 `umi_slam_pipeline`、`Pangolin`、`DBoW2`、`g2o`、`ORB_SLAM3` 与 `umi_slam_runner` 并打包为 `umi-slam:0.1`；`docker run --rm umi-slam:0.1` 正常打印用法并退出 0。（注：Windows 宿主机未启动 Docker Desktop 守护进程，宿主机侧 `slam_station.exe --cli --check --mode docker` 按预期报错提示 Docker 未启动。）

## 3. 待实机验收项

1. 完整采集序列（如 `examples/lissajous_20260915/input`）在 C++ 原生后端与 Docker 后端上的端到端运行覆盖率与轨迹导出对比。
2. 带 Qt6 开发环境的原生 Ubuntu 桌面机执行 `bash build_linux.sh` 与 `bash launch_station.sh` 实测。
3. `calibration/kalibr.sh` 配合 `kalibr:portable-20260921` 容器镜像的真实包转换/标定联合复验。

## 常用验证命令

```powershell
# Windows MSYS2 CLANG64 构建与测试
cmake --build build-windows
ctest --test-dir build-windows --output-on-failure
.\build-windows\umi_slam_pipeline.exe --verify-checksums
```
