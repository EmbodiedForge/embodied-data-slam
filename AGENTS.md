# AGENTS.md — SLAM Station (`embodied-data-slam-cpp`) 项目规范

本文件为 C++17 / Dear ImGui / Conan 2 跨平台原生仓库的项目级 AI / 工程师协作规范，优先级高于全局默认规则。

## 1. 项目定位与目录边界

- **`conanfile.py` & `CMakeLists.txt`**：使用 **Conan 2** 统一管理跨平台依赖（Windows MSVC / Linux / macOS），不依赖 MSYS2 或 Docker，支持全静态原生构建。
- **`include/slam_common/` & `src/slam_common/`**：叶子公共库，**仅允许依赖 C++17 标准库与 `Eigen3::Eigen`**，严禁引入 `OpenCV`、`ImGui`、`GLFW`、`NFD` 或任何上层模块。涵盖 SHA-256 与仓库校验和、JSON 读写、刚体几何/2D 拟合、MS901M 串口解码及轨迹 I/O。
- **5 个隔离功能库（严格零交叉依赖）**：
  - `calibration_targets`
  - `camera_imu_calibration`
  - `handeye_calibration`
  - `vision_calibration`
  - `preview_3d`（唯一允许链接 `imgui::imgui` 的功能库）
  - **红线约束**：上述 5 个库的头文件与源文件严禁互相 `#include`，也严禁反向 `#include` 上层 `umi_slam` 或 `app`。该约束由 `test_zero_cross_dependencies_between_functional_packages` 在 `ctest` 中强制检查。
- **`include/umi_slam/` & `src/umi_slam/`（`umi_slam_core` + `umi_slam_pipeline`）**：纯原生数据预处理/后处理管线、进程树管理与隔离式后端构建器（`--build-backend`，自动透传 Conan 工具链）。支持 `CMake -DEDS_BUILD_STATION=OFF` 无 GUI 独立编译。
- **`src/app/`（`slam_station`）**：Dear ImGui（Docking + GLFW + OpenGL3 + NFD）桌面主程序、标定工作台与上位机 CLI（`--cli`、`--run-pipeline`、`--calibrator`）。
- **`calibration/`**：传统 ROS1 Kalibr 参考脚本与 2026-09-08 实测参考参数（保留作离线参考）。
- **`source/stereo_imu_trajectory/`**：`ORB_SLAM3`（GPLv3）与 `Pangolin`（MIT）基础源码树；仅保留跨平台 C++17 / Clang / MSYS2 编译兼容性修复，离线状态与地图点导出补丁由 `umi_slam::patch_orb()` 在隔离 `.build*` 目录注入。

## 2. 关键数学、坐标、编码与工程约定

1. **刚体变换约定**：`T_A_B` 将 `B` 系坐标变换到 `A` 系（例如 `T_world_target = T_world_imu @ T_imu_target`、`Stereo.T_c1_c2 = inv(Kalibr cam1.T_cn_cnm1)`、`IMU.T_b_c1 = inv(Kalibr cam0.T_cam_imu)`）。四元数导出顺序统一为 `qx, qy, qz, qw`。
2. **时间戳与时延符号**：所有传感器时间戳使用整数纳秒 `int64_t`（十进制精确舍入解析，禁止 `double` 截断）；正的 `camera_to_imu_offset_ns` 定义为 `t_imu = t_camera + offset`。
3. **Windows UTF-8 路径契约**：MSVC 可执行目标直接合并 `cmake/windows_utf8.manifest`（MinGW 链接 `cmake/windows_utf8.rc`），确保窄字符串与 `std::filesystem::u8path` 在中文/空格/`&` 路径下与 UTF-8 一致；跨进程子进程启动统一使用 UTF-16 `CreateProcessW`。
4. **Eigen / OpenCV / FFmpeg 跨版本兼容性**：通过 `conanfile.py` 统一管理依赖（Eigen 3.4、OpenCV 4.10 + LGPL 内置解码 FFmpeg、Boost Serialization、OpenSSL、GLEW、GLFW、Dear ImGui Docking）；统一使用 `Eigen::JacobiSVD<Eigen::MatrixXd>(M, ...)` 构造函数形式，避免使用 OpenCV 5 弃用的 `Mat_ << ...`。
5. **数据防呆红线**：
   - 结果输出目录必须为新建目录，绝不覆盖既有结果或允许输入输出互相嵌套。
   - 未完成惯性初始化或失跟区间绝不线性插补；外参估计异常（旋转激励不足、平移杆臂不可解、时延触界）必须显式标记 `translation_valid` / `imu_extrinsic_status` 并由 UI 弹窗拦截确认。
   - 当前 SLAM 后端仅支持 `stereo_imu`，非双目惯性模式由设备校验拦截。
6. **文件校验清单**：修改仓库文件后，必须运行 `umi_slam_pipeline --update-checksums` 刷新 `SHA256SUMS` 并运行 `ctest` 确保 `test_sha256sums_matches_existing_repo_files` 通过。

## 3. 常用构建与验证命令

### Windows（MSVC + Conan 2 + Ninja + NSIS）
```powershell
# 环境检查 / 清理旧产物 / 一键编译、测试、构建后端并打包 dist\SLAMStation\ 与 NSIS 安装包
powershell -ExecutionPolicy Bypass -File .\build_windows.ps1 -CheckOnly
powershell -ExecutionPolicy Bypass -File .\build_windows.ps1 -CleanOnly
powershell -ExecutionPolicy Bypass -File .\build_windows.ps1

# 手动刷新校验和并运行 CTest
.\build\Release\umi_slam_pipeline.exe --update-checksums
ctest --test-dir build\Release --output-on-failure
```

### Linux / WSL2（Conan 2 + Ninja，详见 `WSL_LINUX_BUILD.md`）
```bash
# 完整桌面构建与测试（WSL2 下自动在 ~/.cache/eds-cpp-linux-build 构建）
BUILD_JOBS=8 bash build_linux.sh

# 仅清理 / 清理后重编 / 无头纯后端模式
bash build_linux.sh --clean-only
bash build_linux.sh --clean
EDS_BUILD_STATION=OFF BUILD_JOBS=8 bash build_linux.sh
```

### macOS（AppleClang + Conan 2 + Ninja）
```bash
bash build_macos.sh
bash build_macos.sh --clean-only
```

## 4. 注意事项（Git Worktree）

当前目录 `C:\code\embodied-data-slam-cpp` 为 Git worktree（分支 `feat/cpp-port`），在 WSL2 下 Linux `git` 无法直接解析 Windows worktree 路径，执行任何 `git` 命令请使用 Windows PowerShell。

## 5. 项目记忆索引

长期架构决策、技术栈背景与阶段进展维护在 `.agent/memory/` 目录，详见 `.agent/memory/README.md`。
