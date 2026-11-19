# SLAM Station · C++17 / Qt6 跨平台上位机全功能说明

`slam_station` 是基于 **C++17 + Qt6 Widgets + OpenCV + Eigen3** 构建的跨平台桌面工作站（支持 **Windows 10/11 原生离线运行**、**Ubuntu Linux 原生运行** 以及 **Docker 容器后端**），完全摆脱对宿主机 Python 或 ROS 的运行时依赖，并内置 **免 ROS 多模态相机/IMU 标定、动态标定板生成打印、VisionMaster N 点/平面/畸变/相机映射、以及 3D 手眼标定**。

---

## 一、核心 GUI 功能与模块结构

### 主界面 5 大分页（`src/app/main_window.cpp`）

1. **处理任务**（`umi_slam::Job` + `runtime.cpp` + `pipeline.cpp`）：
   - 选择采集目录（支持 `stereo.mkv` + `umi_imu_timestamps.bin` 与标准 EuRoC `mav0/` 目录）、结果父目录、标定 YAML、设备配置 JSON。
   - 运行后端支持 **`自动（优先本地）` / `本地原生 (ORB-SLAM3 C++)` / `Docker 容器`**；勾选 **`实时 3D 建图窗口 (Pangolin · 本地原生)`** 可在重建期间实时观察特征跟踪与建图状态。
   - 支持 **仅检查并准备数据**、抽帧间隔 (`stride`)、最大帧数 (`max_frames`)、随时停止并安全清理整棵子进程树（Windows 下通过进程树强杀与状态回写，POSIX 下通过进程组信号），不会遗留孤儿 `umi_slam_runner` 进程。
2. **轨迹与建图预览**（独立组件 `src/preview_3d/preview_widget.cpp`）：
   - 基于 Qt `QPainter` 的交互式 **3D 点云与轨迹轨道视图**（左键旋转、滚轮缩放、右键平移，支持关键帧、地图点 Z 轴高度色谱、起始/终止标记、目标坐标系切换）+ **XY / XZ / YZ 三正交投影视图**。
   - 任务完成后自动刷新，也可手动加载历史目录、弹出独立大窗口或一键打开离线 HTML 报告。
3. **多模态标定 / 手眼工作台**（`src/app/calibration_workbench.cpp`，支持主窗口内嵌、独立弹窗或 `launch_calibrator.cmd` / `slam_station --calibrator` 独立启动）：
   - **子页 1 · 多模态相机 / IMU 标定**（`camera_imu_calibration` + `calibration_targets`）：
     - 支持 `仅 IMU` / `单目` / `双目` / `多目` 与 `启用 IMU 联合标定` 组合；支持 **AprilGrid 36h11**、**棋盘格**、**对称/非对称圆阵列**。
     - 自动检测同步旋转激励不足、时延触及搜索边界、互相关度偏低或平移杆臂不可解，显式写入 `imu_extrinsic_status` 与警告注释；在点击 **应用到当前 SLAM 任务** 时拦截非 `stereo_imu` 模式并在外参状态异常时弹窗确认。
     - 支持 **Kalibr YAML 免 ROS 直接转换** 生成 `orbslam.yaml` 与 `device_profile.json`。
   - **子页 2 · 标定板生成与打印**（`calibration_targets/generator.cpp`）：
     - 动态导出 1:1 真实物理毫米比例矢量 **PDF / SVG / 带 `pHYs` DPI 块 PNG**，保存后可直接打开打印。
   - **子页 3 · N 点标定 & 3D 手眼**（`handeye_calibration`）：
     - **2D N 点标定**：9+3 S 形示教轨迹生成、4/6/8 自由度（相似/仿射/透视）与 `最小二乘 / Huber / Tukey / RANSAC` 鲁棒权重、旋转轴中心拟合、`.iwcal` 读写与 $(\Delta X, \Delta Y, \Delta\theta)$ 对位计算。
     - **3D 机械臂手眼标定**：纯 C++/Eigen 实现 `Tsai`、`Park`、`Horaud`、`Andreff`、`Daniilidis` 五种算法，支持 `Eye-in-Hand (AX=XB)` 与 `Eye-to-Hand (AX=ZB)`。
   - **子页 4 · 2D 平面 / 畸变 / 相机映射**（`vision_calibration`）：
     - 实现 2D 平面标定（`mm/px`）、像素↔物理坐标/长度转换、径向/透视/复合畸变校正与原图对比预览、双相机 2D 映射与 `.iwcal` 存取。
4. **设备与扩展**：查看、另存及切换 `UMI / EGO / generic` 设备 JSON 模板。
5. **历史结果**：浏览 `run_*` 历史任务状态、日志、一键跳转 3D 预览与报告。

---

## 二、零交叉依赖架构

与早期原型一致，5 个核心功能库严格禁止互相 `#include` 或反向依赖上层 `umi_slam` / `app`，公共纯函数、SHA-256、JSON、几何拟合、MS901M 解码与轨迹 I/O 均收敛在叶子库 `slam_common`（仅依赖 C++17 标准库与 Eigen3，无 Qt/OpenCV 依赖）：

```text
include/ & src/
├── slam_common/             # 叶子公共库（仅 C++17 + Eigen3；无 OpenCV / Qt6）
├── calibration_targets/     # 标定板识别 + PDF/SVG/PNG 动态生成
├── camera_imu_calibration/  # 仅IMU / 单目 / 双目 / 多目 + IMU 联合标定与 Kalibr 转换
├── handeye_calibration/     # 2D N点标定/旋转中心/对位 + 3D AX=XB / AX=ZB
├── vision_calibration/      # 2D 平面标定、单位转换、畸变校正、双相机映射
├── preview_3d/              # Qt6 3D 点云轨迹轨道交互预览 & 三正交视图
├── umi_slam/                # 数据管线、进程树管理、隔离式 --build-backend 构建器
└── app/                     # Qt6 主界面、标定工作台与 CLI 入口
```

以上零交叉依赖约束由单元测试 `test_zero_cross_dependencies_between_functional_packages` 在 CTest 中自动强制检查。

---

## 三、Windows 构建、独立打包与启动

### 一键检查、编译与打包

在仓库根目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\build_windows.ps1
```

或直接运行 `build_windows.cmd`。脚本会自动：
1. 查找（或在缺失时引导安装）MSYS2 CLANG64 工具链；
2. 校验或通过 `pacman` 安装 Clang、LLVM (`llvm-objdump`)、CMake、Ninja、Qt6、OpenCV、Eigen3、Boost、OpenSSL、GLEW；
3. 使用 CMake + Ninja 编译全部 C++ 库、可执行程序与测试，并运行 `ctest`；
4. 调用 `umi_slam_pipeline.exe --build-backend` 隔离编译 `Pangolin + DBoW2 + g2o + ORB_SLAM3 + umi_slam_runner.exe`；
5. 调用 `windeployqt6.exe` 与 `llvm-objdump.exe` 收集 Qt6 插件、OpenCV/CLANG64 运行时 DLL 及可重定位 `backend\backend.json`，组装完整独立目录 `dist\SLAMStation\`。

常用参数：

```powershell
.\build_windows.ps1 --check-only                      # 仅检测编译环境
.\build_windows.ps1 --msys2-root D:\msys64 --install-deps --rebuild-backend
.\build_windows.ps1 --jobs 8
```

打包完成后，将整个 `dist\SLAMStation\` 目录复制到任意 Windows 10/11 电脑即可双击 `slam_station.exe` 或 `SLAMCalibrator.cmd` 离线运行（目标电脑无需安装 MSYS2、Python、ROS 或 Docker）。所有可执行文件均已嵌入 UTF-8 `activeCodePage` 清单，原生支持含中文、空格与 `&` 的路径。

### Docker 容器后端（可选）

如果希望在 Windows 上通过 Docker Desktop Linux 容器运行 SLAM 后端：
1. 启动 Docker Desktop（Linux containers 模式）；
2. 运行 `load_images_windows.cmd` 导入离线 `umi-slam:0.1` 镜像；
3. 启动 `launch_windows.cmd`，将运行后端选择为 **Docker 容器**。

---

## 四、任务输出目录结构

```text
SLAM_Results/run_日期_随机ID/
  station_job.json          # 上位机状态、参数、后端类型/镜像 ID、设备信息
  host.log
  config/settings.yaml      # 标定 YAML 快照
  config/device.json        # 完整设备接口配置
  config/profile.json       # 传给后端管线的数据配置
  result/
    job.json / slam.log / prepared/
    imu_trajectory.csv / imu_trajectory.tum / keyframes_euroc.txt / map_points.csv
    quality.json / report.html / tracking.csv
    target_trajectory.csv / target_frame.json  # 当提供有效 T_imu_target 时自动导出
```

详见 [DEVICE_API.md](software/slam_station/DEVICE_API.md) 与 [VALIDATION.md](software/slam_station/VALIDATION.md)。
