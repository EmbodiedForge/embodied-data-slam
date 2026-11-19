# 项目交接与工程记忆文档（`HANDOFF.md`）

> 本文档面向后续接手的 AI Agent / 工程师，汇总当前项目背景、关键记忆与红线约束、已完成实现、待实现/待验收事项，以及后续优化与重点测试清单。

---

## 1. 仓库拓扑与环境记忆

### 1.1 双目录 Worktree 结构（重要）
- **Python 原型主仓**：`C:\code\embodied-data-slam`（分支 `main`）
  - 保留早期完整 Python + Tkinter 版本与优化规划成果（仅作对照参考；当前开发重心已切换至 C++ 版本）。
- **C++17 / Qt6 移植仓**：`C:\code\embodied-data-slam-cpp`（Git worktree，分支 `feat/cpp-port`）
  - 当前所有 C++ 实现、编译脚本、Docker 改造与独立打包均在此目录完成。
  - **两仓目前均保留工作区未提交改动（用户此前明确暂不自动 `git commit`）**。
  - **Git Worktree 工具链注意**：`C:\code\embodied-data-slam-cpp` 是 Windows Git worktree，WSL2 下的 Linux `git` 无法解析 Windows 盘符 `.git` 指针文件；**所有 `git` 命令必须在 Windows PowerShell 中执行**。

### 1.2 开发与验证环境
- **Windows 编译环境（最新主线）**：**Visual Studio 2026 Build Tools（MSVC 195）+ Conan 2.34.0 + CMake 4.4.5 + Ninja 1.13.2**；已全面替换 MSYS2 与 Qt6，所有第三方库通过 `conanfile.py` 全静态管理（`opencv/*:with_ffmpeg=True` 已开启以兼容旧版 `stereo.mkv`）。
- **WSL2 Linux 环境**：Ubuntu 26.04（GCC 15.2、Ninja、Conan 2）；参见 [WSL_LINUX_BUILD.md](WSL_LINUX_BUILD.md)，`build_linux.sh` 在 `/mnt/*` 下自动使用 `~/.cache/eds-cpp-linux-build` 构建以避免 DrvFs 慢 IO 与 Windows 构建目录冲突。
- **旧 Docker / Kalibr 资产**：按用户指令保留原有 `software/umi_slam/Dockerfile*`、`build_docker.sh`、`load_images*` 与 `calibration/kalibr.sh` 作参考；上位机与管线运行时已精简为纯原生模式。

### 1.3 不可违背的架构与工程红线
1. **依赖分层红线**：
   - `slam_common`（`include/slam_common/`、`src/slam_common/`）为纯 C++17 + `Eigen3::Eigen` 叶子库，**严禁引入 OpenCV、ImGui、GLFW、Qt 或任何上层头文件**。
   - 5 个业务功能库（`calibration_targets`、`camera_imu_calibration`、`handeye_calibration`、`vision_calibration`、`preview_3d`）**严禁互相 `#include`，也严禁反向 `#include` `umi_slam` 或 `app`**（其中仅 `preview_3d` 允许链接 `imgui::imgui`）。由 `test_zero_cross_dependencies_between_functional_packages` 强制检查。
2. **数学与时间戳约定**：
   - `T_A_B` 表示将 `B` 系点变换到 `A` 系（`T_world_target = T_world_imu @ T_imu_target`；`Stereo.T_c1_c2 = inv(Kalibr cam1.T_cn_cnm1)`；`IMU.T_b_c1 = inv(Kalibr cam0.T_cam_imu)`）。
   - 四元数导出顺序统一为 `qx, qy, qz, qw`。
   - 时间戳一律使用整数纳秒 `int64_t`（十进制精确舍入解析，禁止 `float/double` 截断）；正 `camera_to_imu_offset_ns` 定义为 `t_imu = t_camera + offset`。
   - 实物 AprilGrid 标定板边长已由用户确认为 **34.5 mm**（`calibration/targets/aprilgrid_6x6_measured34p5mm.yaml`），`aprilgrid_6x6_nominal40mm_A3.pdf` 文件名中的 `40mm` 仅作历史参考。
3. **数据防呆红线**：
   - 输出目录必须新建，禁止覆盖既有结果，禁止输入输出目录互相嵌套。
   - 未完成惯性初始化、失跟区间禁止线性插补；外参求解退化（旋转激励不足、时延触界、互相关低、平移杆臂不可解）必须显式标记 `translation_valid` / `imu_extrinsic_status` 与警告并在 GUI 应用前拦截确认。
   - 当前 `software/umi_slam/backend/runner.cc` 仅支持 `stereo_imu`，非双目惯性模式必须在任务创建及应用前显式拒绝。
4. **跨平台兼容性经验（已踩坑总结）**：
   - **Windows 中文路径（ACP 936 / GBK）**：所有 Windows `.exe`（含独立编译的 `umi_slam_runner.exe`）必须链接 `cmake/windows_utf8.rc`（**必须包含 `#include <winuser.h>`**，否则 `windres` 将 `RT_MANIFEST` 识别为字符串资源而非类型 `24`，清单不生效导致 `GetACP()` 仍为 936 且 `std::filesystem::u8path` 抛出 `locale not supported`），并配合 `SetConsoleOutputCP(CP_UTF8)` 与 UTF-16 `CreateProcessW`。
   - **Eigen 3.4 与 5.x 共存**：`A.bdcSvd<...>()` 在 Ubuntu Eigen 3.4 下编译失败，`A.bdcSvd(...)` 在 Eigen 5.x 下报弃用警告；统一使用 `Eigen::JacobiSVD<Eigen::MatrixXd>(A, Eigen::ComputeThinU | Eigen::ComputeThinV)` 构造函数写法；OpenCV 5 禁止使用 `cv::Mat_<double>(...) << ...`。
   - **Lambda 返回值陷阱**：在测试或业务代码中，涉及 Eigen 矩阵表达式的 lambda **必须显式声明返回类型**（如 `-> Eigen::Vector3d`），避免返回引用局部临时对象的表达式模板引发 `NaN` 未定义行为。
   - **行尾与 `SHA256SUMS`**：`.gitattributes` 锁定 `*.sh` 为 LF、`*.cmd/*.ps1` 为 CRLF。修改仓库内代码/脚本/根目录 `.md`/配置后，必须运行 `umi_slam_pipeline --update-checksums` 刷新 `SHA256SUMS`。

---

## 2. 已实现内容（`C:\code\embodied-data-slam-cpp`）

### 2.1 C++17 / Qt6 核心代码与构建体系
- **根工程与无头开关**：`CMakeLists.txt` 支持 `EDS_BUILD_STATION=ON/OFF` 与 `BUILD_TESTING`；`cmake/windows_utf8.manifest` + `cmake/windows_utf8.rc` 为全部可执行目标嵌入 UTF-8 进程代码页清单。
- **全部核心库移植完成**：
  - `slam_common`：流式 `Sha256`、`compute_canonical_sha256`、`verify_repo_checksums`、`update_repo_checksums`、JSON 原子写入、2D/3D 几何拟合、MS901M 串口解码、轨迹变换与 SVG/HTML 报告导出。
  - 5 个独立功能库：`calibration_targets`（检测 + 1:1 矢量 PDF/SVG/PNG 生成）、`camera_imu_calibration`（单/双/多目 + IMU 免 ROS 标定与 Kalibr 转换）、`handeye_calibration`（VisionMaster 2D N 点/旋转中心/对位 + 5 种 3D 手眼）、`vision_calibration`（2D 平面/畸变/双目映射）、`preview_3d`（Qt `QPainter` 3D 轨道点云/轨迹与三正交视图）。
  - `umi_slam_core` & `umi_slam_pipeline`：预处理/导出、原生进程树与 Docker 容器生命周期管理、`--verify-checksums` / `--update-checksums`、以及 `--build-backend` 隔离后端构建器（源码指纹 `backend_source_digest`、隔离拷贝、`patch_orb()`、`cmake -E tar` 解压词典、编译 `Pangolin/DBoW2/g2o/ORB_SLAM3/umi_slam_runner`、Windows `llvm-objdump` 递归收集 122 个 DLL、输出相对路径可重定位 `backend.json`）。
  - `src/app`（`slam_station`）：Qt6 Widgets 5 分页主界面、4 分页标定工作台、`--cli`、`--run-pipeline`、`--calibrator` 独立入口。

### 2.2 Kalibr 容器辅助脚本、Docker 与跨平台脚本/文档
- **Kalibr 兼容**：保留并从 git 恢复容器内 ROS1 辅助脚本（`calibration/tools/kalibr_to_bag.py`、`calibration/tools/kalibr_to_orb.py`、`calibration/build/PlotCollection_headless.py`、`calibration/build/prepare_stereo_bag.py`），维持 `calibration/kalibr.sh` 不变。
- **纯 C++ Docker 镜像**：重写 `software/umi_slam/Dockerfile` 与 `software/umi_slam/Dockerfile.dockerignore`，新增 `software/umi_slam/build_docker.sh`，基于 Ubuntu 22.04 编译无 Qt6 版 `umi_slam_pipeline` 与 `ORB_SLAM3` 后端，构建出 `umi-slam:0.1`。
- **一键构建、打包与启动脚本**：新增/更新 `build_windows.ps1`、`build_windows.cmd`、`software/slam_station/build_windows.ps1`、`build_linux.sh`、`launch_windows.cmd`、`launch_calibrator.cmd`、`launch_station.sh`、`launch_calibrator.sh`、`launch_software.sh`、`software/umi_slam/launch.sh`、`load_images_windows.cmd`、`load_images.sh`。
- **文档与规范**：更新 `MANIFEST.json`、`SHA256SUMS`、`README.md`、`README_先读我.md`、`WINDOWS_上位机说明.md`、`software/slam_station/VALIDATION.md`、`software/umi_slam/README.md`、`AGENTS.md` 与 `.agent/memory/*`。

### 2.3 已通过的实测验证
- **Windows MSVC 195 + Conan 2 + Dear ImGui 全量构建**：主程序 0 警告编译通过；`slam_station.exe`（9.2 MB）、`umi_slam_pipeline.exe`（3.9 MB）、`backend/umi_slam_runner.exe`（7.5 MB）均为**全静态链接、0 第三方 DLL**（仅动态链接 `/MD` VC++ 运行库）。
- **FFmpeg 解码与 MSVC 195 避坑**：`conanfile.py` 开启 `opencv/*:with_ffmpeg=True` 并禁用 FFmpeg 外部/GPL 编码器（`with_libx265=False`、`with_libx264=False`、`postproc=False` 等），既规避了 `libx265/3.6` 在 MSVC 195 下的编译失败，又保留了全部内置 MKV/MP4/H.264/HEVC/MJPEG/FFV1 解码且保持纯 LGPL。
- **商用友好 Windows 安装包（NSIS 3.13）**：新增 `installer/windows/slam_station.nsi`（zlib/libpng 许可证），集成至 `build_windows.ps1`，内置匹配版本的 `vc_redist.x64.exe`（静默安装 `/install /quiet /norestart`），已验证生成 `dist/SLAMStation-Setup-2.0.0-x64.exe`（56.7 MB），安装完成即可直接运行。
- **三端清理脚本**：`build_windows.ps1` 支持 `-Clean` / `-CleanOnly`，`build_linux.sh` 与 `build_macos.sh` 支持 `--clean` / `--clean-only`（保留 `~/.conan2` 缓存）。
- **Windows CTest**：`test_pipeline_cpp` 与 `test_station_cpp` **2/2 全部通过**（含零交叉依赖、`SHA256SUMS`、合成双目+IMU 标定、进程树强杀与中文路径测试）。
- **干净环境自检**：`PATH=C:\Windows\system32;C:\Windows` 下实测 `dist/SLAMStation/slam_station.exe --cli --check` 返回 `mode: native` 且就绪。
- **待 WSL2 / Linux 与 macOS 实测**：参见 [WSL_LINUX_BUILD.md](WSL_LINUX_BUILD.md)，在 WSL2 Ubuntu 中验证 `build_linux.sh`。

---

## 3. 待实现与待外部环境验收内容

### 3.1 待外部环境实机验收（高优先，当前环境缺少对应硬件/镜像/Qt6 包）
1. **真实采集序列端到端跑批与对比回归**：使用 `examples/lissajous_20260915/input`（当前仓库未随带大体积 `examples/`）分别运行 C++ 原生后端与 `umi-slam:0.1` Docker 后端，核对覆盖率、分段、`imu_trajectory.csv/tum`、`map_points.csv`、`report.html` 与历史参考结果（`examples/lissajous_20260915/reference_result`）的一致性。
2. **原生 Ubuntu 桌面（含 `qt6-base-dev`）完整打包验证**：在带 Qt6 开发包的 Ubuntu 22.04/24.04 主机上运行 `bash build_linux.sh`，验证 `ldd + patchelf` 动态库收集、`qt.conf` 插件加载、`dist/SLAMStation/slam_station` 跨目录运行与中文显示（`fonts-wqy-microhei`）。
3. **Windows Docker Desktop 联调**：在启用 Docker Desktop（WSL2 Linux containers）的 Windows 机器上实测 `slam_station.exe --cli --check --mode docker`、`load_images_windows.cmd` 以及含中文/空格/盘符路径的容器挂载跑批。
4. **Kalibr 容器联调**：加载 `kalibr:portable-20260921` 后实测 `calibration/kalibr.sh` 的 `bag` -> `stereo` -> `imu` -> `convert` 全流程，并与 C++ 免 ROS 标定结果交叉对比。

### 3.2 待实现功能演进（对应原规划阶段 5 与工程收尾）
1. **SLAM 传感器模式扩展**：当前 `software/umi_slam/backend/runner.cc` 与 `umi_slam::settings_info()` 写死 `IMU_STEREO`；若后续支持纯双目（`STEREO`）、单目惯性（`IMU_MONOCULAR`）或多目设备，需同步扩展 `runner.cc` 分支、预处理校验及 `validate_slam_sensor_mode`。
2. **离线处理加速（需真实数据集回归后改动）**：
   - `runner.cc` 目前按帧间采集时间戳间隔 `sleep_until`（1× 实时倍速）喂帧以保障后台 LocalMapping/LoopClosing；可增加 `--speed` 参数或基于 `LocalMapping` 空闲状态的自适应喂帧。
   - `umi_slam::prepare()` 对被 `stride` 跳过的旧式视频帧目前仍完整解码（`cap.read` / `retrieve`），可改为 `grab()` 跳过并评估 PNG 压缩级别（`IMWRITE_PNG_COMPRESSION`）或多线程写盘提速。
3. **标定算法增强**：
   - 免 ROS 相机-IMU 标定目前为“网格互相关时延 + 加权 Procrustes 旋转 + 二阶差分线性回归平移”；后续可在闭式初值之上增加小规模非线性联合优化（BA / 连续时间样条），并输出重投影/残差统计。
   - `calibrate_imu_standalone` 目前使用静态段标准差与下限值粗估噪声密度与随机游走，后续可实现 Allan 方差辨识并要求足够时长的静置数据。
4. **Git 历史整理与 CI（按用户指令暂缓）**：
   - 当前 `main` 与 `feat/cpp-port` 的大量工作区改动尚未提交；审阅后可按“构建与基础设施 / slam_common 与算法库 / 管线与后端构建 / Qt6 上位机 / 文档与校验和”分批提交。
   - CI（GitHub Actions Windows CLANG64 + Ubuntu 构建与 CTest）按用户指示“先不加 CI”，后续需要时再引入。
5. **GPLv3 合规分发打包**：`umi_slam_runner` 静态链接 GPLv3 的 `ORB_SLAM3`；对外分发 `dist/SLAMStation/` 时建议在 `build_windows.ps1` / `build_linux.sh` 中增加可选 `--with-source-archive` 自动附带对应源码或书面源码提供声明。

---

## 4. 需要优化的点与重点测试点（Optimization & Test Backlog）

### 4.1 已知可优化点
1. **相机-IMU 平移杆臂数值稳定性**：`estimate_cam_imu_extrinsic` 依赖二阶中心差分 `p_dd` 与 `R_dd`，在低动态、低帧率或平面靶标 PnP 深度噪声较大时，正规方程 `ATA` 条件数可能恶化（目前已通过 `kMaxPlausibleLeverArmM = 0.5` 安全拦截并回退为零平移）。建议增加矩阵条件数/最小奇异值检查与多帧平滑窗口。
2. **标定 UI 长耗时操作异步化评估**：目前 `calibration_workbench.cpp` 中多模态标定走后台线程，但大图（>4K）畸变校正、大批量角点检测或 N 点 RANSAC 仍在界面线程或同步回调执行，后续可对大图去畸变/批量检测统一加进度回调与异步化。
3. **独立包体积瘦身**：Windows `build_windows.ps1` 通过 `llvm-objdump` 递归收集了 199 个应用 DLL 与 122 个后端 DLL（包含完整 OpenCV 5 / FFmpeg / Abseil 依赖）；若需缩小离线发布包体积，可排查非必要图像/视频编解码插件并按需裁剪。
4. **非 Windows 英文/中文路径打包测试补充**：当前仅在 Windows 验证了 `test_pipeline_cli_chinese_path_fails_cleanly`，后续可把含中文/空格/`&` 路径测试也覆盖到 `slam_station --cli` 与真实 `prepare()` 成功路径（构造最小合法 EuRoC 合成数据集）。

### 4.2 后续 Agent 必跑回归与测试清单

每次修改代码后，建议按以下顺序验证：

```powershell
# 1. Windows MSVC + Conan 2 + Ninja 一键编译、刷新校验和、CTest、后端构建与 NSIS 安装包
powershell -NoProfile -ExecutionPolicy Bypass -File .\build_windows.ps1 -CheckOnly
powershell -NoProfile -ExecutionPolicy Bypass -File .\build_windows.ps1

# 或快速增量刷新校验和与 CTest
.\build\Release\umi_slam_pipeline.exe --update-checksums
ctest --test-dir build\Release --output-on-failure
```

```bash
# 2. WSL2 / Linux 原生编译与测试（详见 WSL_LINUX_BUILD.md）
BUILD_JOBS=8 bash build_linux.sh
```

重点手工/集成测试点：
1. **无环境迁移测试**：将 `dist/SLAMStation/` 复制到不含 MSYS2/Qt/Python 的目录或干净 Windows 虚拟机，且路径包含中文、空格和 `&`（例如 `C:\测试 目录&1\SLAMStation\`），验证双击启动、`--calibrator` 启动、打开 A3 标定板 PDF、3D 预览加载历史结果及原生后端自检。
2. **原生任务中途取消与进程树清理**：运行长序列原生任务过程中点击停止或关闭主窗口，核查任务管理器中无残留 `umi_slam_runner.exe` / `umi_slam_pipeline.exe`，且 `result/job.json` 与 `station_job.json` 状态均收敛为 `cancelled`。
3. **外参退化防呆链路**：使用缺 IMU、零旋转激励、大时延（>50 ms）或稀疏平移的数据运行工作台标定，验证输出 YAML 含 `# WARNING`、点击“应用到当前 SLAM 任务”弹出默认拒绝确认框，以及单目/纯 IMU 配置被禁止流入双目惯性任务。
4. **构建脚本与校验和闭环**：任何增删根目录文档、脚本、`src/`、`include/`、`tests/`、`calibration/` 文件的操作完成后，务必运行 `umi_slam_pipeline --update-checksums` 再跑 `ctest`，确保 `test_sha256sums_matches_existing_repo_files` 与 `test_zero_cross_dependencies_between_functional_packages` 通过。
