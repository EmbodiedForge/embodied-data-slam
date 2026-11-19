# 架构与技术决策（`decisions.md`）

1. **叶子库 `slam_common` 与 5 大功能库零交叉依赖**：
   - `slam_common` 仅依赖 C++17 标准库与 `Eigen3::Eigen`，不含 OpenCV 或 Qt6。
   - `calibration_targets`、`camera_imu_calibration`、`handeye_calibration`、`vision_calibration`、`preview_3d` 严格禁止互相 `#include` 或反向引用上层模块，由 `test_zero_cross_dependencies_between_functional_packages` 强制检查。
2. **Windows UTF-8 清单与 `CreateProcessW`**：
   - Windows 中文系统默认 ACP 为 936（GBK），若仅用 `std::filesystem::u8path` 会导致中文路径在 libc++ 抛出 `locale not supported`。
   - 所有 Windows 可执行目标（含独立构建的 `umi_slam_runner`）嵌入 `cmake/windows_utf8.rc`（含 `#include <winuser.h>` 使 `RT_MANIFEST` 正确解析为资源类型 `24`，从而激活 `<activeCodePage>UTF-8</activeCodePage>`），配合 `SetConsoleOutputCP(CP_UTF8)` 与 `CreateProcessW` 彻底解决中文/空格/`&` 路径问题。
3. **Eigen 3.4 / 5.0 与 OpenCV 4 / 5 跨平台兼容写法**：
   - 统一使用 `Eigen::JacobiSVD<Eigen::MatrixXd>(M, Eigen::ComputeThinU | Eigen::ComputeThinV)` 构造函数写法，避免 `bdcSvd<...>()` 在 Eigen 3.4 无法编译以及 `bdcSvd(...)` 在 Eigen 5 产生弃用警告。
4. **可重定位后端配置与独立打包**：
   - `write_backend_config()` 在 `backend.json` 中写入相对 `build_dir` 的 POSIX 路径（替代旧 Python 版的绝对路径），`load_backend_config()` 与 `find_native_backend()` 自动按 `backend.json` 所在目录解析。
   - `build_windows.ps1` 与 `build_linux.sh` 分别使用 `windeployqt6` + `llvm-objdump` 与 `ldd` + `patchelf` 打包完全自包含的 `dist/SLAMStation/` 目录。
5. **Kalibr 保留容器内辅助脚本**：
   - `.bag` 生成依赖 ROS1 `rosbag` API，因此恢复 `calibration/tools/kalibr_to_bag.py` 等 4 个容器内 Python 脚本配合 `calibration/kalibr.sh` 使用，其余业务与上位机全部纯 C++ 化。
6. **SHA256SUMS 规则与跨平台文本规范化**：
   - `slam_common::compute_canonical_sha256` 对文本文件规整 `CRLF->LF`，对二进制与 `.cmd/.bat/.ps1`（`.gitattributes` 锁定 CRLF）按原始字节计算。`update_repo_checksums` 扫描当前在盘代码/脚本/配置，保留未随 Git 分发的 `images/`、`examples/`、`source/`、`calibration/reference_20260908/` 离线记录，清理已删除的旧 Python 条目。
