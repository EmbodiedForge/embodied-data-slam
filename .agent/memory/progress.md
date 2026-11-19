# 当前进度与待验证项（`progress.md`）

## 已完成
- [x] **编译警告与外参数值测试修复**：消除 Eigen 5 / OpenCV 5 弃用警告及 Eigen 3.4 跨版本编译错误；修复 `test_synthetic_stereo_and_imu_extrinsic_calibration` 中 `pw_b_at` lambda 悬垂 Eigen 表达式模板导致的 `NaN` 问题。
- [x] **Windows UTF-8 非 ASCII 路径支持与校验和 CLI**：补全 `cmake/windows_utf8.rc`（`#include <winuser.h>`）、`SetConsoleOutputCP(CP_UTF8)`、`CreateProcessW`、`--verify-checksums` / `--update-checksums` 及对应回归测试。
- [x] **C++ `--build-backend` 实现与实测**：实现 `backend_source_digest`、`stage_backend_sources`（隔离拷贝 + `patch_orb` + `cmake -E tar` 解压词典）、`build_backend`、Windows DLL 递归收集与可重定位 `backend.json`；在 Windows 上实测成功编出 `umi_slam_runner.exe` 与 122 个依赖 DLL。
- [x] **Kalibr 辅助脚本与纯 C++ Docker 镜像**：恢复容器内 4 个 Kalibr Python 脚本；`CMakeLists.txt` 增加 `EDS_BUILD_STATION` 选项；升级 `software/umi_slam/Dockerfile` 与 `build_docker.sh` 并实测成功构建及运行 `umi-slam:0.1`。
- [x] **跨平台构建/打包/启动脚本与文档体系**：新增并更新 `build_windows.ps1` / `build_windows.cmd` / `build_linux.sh` / `launch_*.cmd` / `launch_*.sh` / `load_images*`、中英文文档、`MANIFEST.json`、`AGENTS.md` 与 `.agent/memory/`。

## 待实机/外部环境验收项
- [ ] 在真实 Ubuntu 桌面（含 `qt6-base-dev`）执行 `bash build_linux.sh` 验证完整 Linux Qt6 桌面打包产物。
- [ ] 使用真实采集序列（如 `examples/lissajous_20260915/input`）跑通端到端原生与 Docker 轨迹恢复对比。
- [ ] 在安装有 Docker Desktop 的 Windows 主机上验证 `slam_station.exe --cli --check --mode docker` 与容器跑批。
