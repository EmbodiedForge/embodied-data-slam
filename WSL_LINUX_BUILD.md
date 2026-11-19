# WSL2 / Ubuntu Linux 原生编译与测试指南（`WSL_LINUX_BUILD.md`）

本文档用于在 **WSL2（Ubuntu 22.04 / 24.04 / 26.04）** 或原生 Ubuntu 桌面环境下，通过 **Conan 2 + Ninja + GCC/Clang** 构建并测试跨平台原生版 SLAM Station（Dear ImGui + ORB-SLAM3 后端）。

> **重要提醒（Git Worktree）**：
> `C:\code\embodied-data-slam-cpp` 是 Windows 侧创建的 Git worktree，WSL2 中的 Linux `git` 无法解析 Windows 盘符 `.git` 指针。**在 WSL2 中请只做编译与运行测试，所有 `git` 命令一律回到 Windows PowerShell 执行。**

---

## 1. 环境准备（首次在 WSL2 中运行）

### 1.1 检查编译器、CMake、Ninja、Conan 2

```bash
g++ --version
cmake --version
ninja --version
conan --version
```

若缺少 `conan`（需 Conan 2.x），可用 `pipx` 或虚拟环境安装（无需 `sudo`）：

```bash
python3 -m venv ~/.venvs/conan
source ~/.venvs/conan/bin/activate
pip install -U conan
conan --version
```

若缺少基础编译工具或 OpenGL / X11 / EGL 开发包（有 `sudo` 权限时执行）：

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake ninja-build pkg-config \
  libgl1-mesa-dev libglu1-mesa-dev libegl1-mesa-dev libepoxy-dev \
  libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
  libwayland-dev libxkbcommon-dev \
  zenity fonts-wqy-microhei
```

> **说明**：
> - `libepoxy-dev` / `libegl1-mesa-dev`：`Pangolin` 在 Linux 下编译需要；
> - `libx11-dev` 等 + `libgl1-mesa-dev`：`GLFW` + `Dear ImGui` 编译需要；
> - `zenity`：Linux 原生文件/目录选择弹窗运行时调用；
> - `fonts-wqy-microhei`：Dear ImGui 界面中文字体显示。
> - 如果当前 WSL2 **没有 `sudo` 且缺少 X11/OpenGL 开发头**，可先用下文的**无头模式（`EDS_BUILD_STATION=OFF`）**验证核心管线与 `test_pipeline_cpp`。

### 1.2 初始化 Conan 默认 Profile

```bash
conan profile detect --exist-ok
conan profile show
```

确认输出中的 `os=Linux`、`compiler=gcc`（或 `clang`）、`build_type=Release`。

---

## 2. 一键编译与测试

进入仓库目录（对应 Windows 的 `C:\code\embodied-data-slam-cpp`）：

```bash
cd /mnt/c/code/embodied-data-slam-cpp
```

`build_linux.sh` 已针对 WSL2 做过专门优化：检测到当前路径位于 `/mnt/*` 时，会自动把 Conan 生成文件、Ninja 中间产物和 ORB-SLAM3 构建目录放到 **Linux 原生 ext4 分区**（`~/.cache/eds-cpp-linux-build`），既避免 `/mnt/c` 跨文件系统 I/O 极慢的问题，又不会覆盖 Windows MSVC 的 `build/Release` 目录。

### 方式 A：完整构建（含 Dear ImGui GUI + CTest + ORB-SLAM3 原生后端）

```bash
# 若 Conan 安装在 venv 中，先激活：source ~/.venvs/conan/bin/activate
BUILD_JOBS=8 bash build_linux.sh
```

> **首次构建提示**：`conanfile.py` 已开启 `opencv/*:with_ffmpeg=True` 以支持旧版 `stereo.mkv` 视频解码，同时关闭了 FFmpeg 的外部/GPL 编码器（`libx264`/`libx265`/`libvpx`/`libaom` 等），仅保留 FFmpeg 内置 MKV/MP4/H.264/HEVC/MJPEG/FFV1 解码器，编译更快且保持纯 LGPL。完成后全部依赖缓存在 `~/.conan2`，后续增量构建仅需数秒。

### 清理旧构建与 `dist-linux` 产物

```bash
# 仅清理旧编译目录与 dist-linux（保留 ~/.conan2 缓存）
bash build_linux.sh --clean-only

# 清理后重新完整编译
BUILD_JOBS=8 bash build_linux.sh --clean
```

### 方式 B：快速验证主程序与 CTest（跳过 ORB-SLAM3 后端）

```bash
SKIP_BACKEND=1 BUILD_JOBS=8 bash build_linux.sh
```

### 方式 C：无头服务器 / 无 X11 开发包模式（仅编 `umi_slam_pipeline` + 后端）

```bash
EDS_BUILD_STATION=OFF BUILD_JOBS=8 bash build_linux.sh
```

---

## 3. 验证与运行测试

构建完成后，独立产物位于 `/mnt/c/code/embodied-data-slam-cpp/dist-linux/SLAMStation/`（或可直接运行 `~/.cache/eds-cpp-linux-build/build/Release/` 下的二进制）。

### 3.1 CLI 与后端自检

```bash
cd /mnt/c/code/embodied-data-slam-cpp/dist-linux/SLAMStation

# 检查本地原生后端就绪状态
./slam_station --cli --check

# 检查动态库依赖（核心第三方依赖均为静态链接，仅剩系统基础库与 GL/X11）
ldd ./slam_station
ldd ./backend/umi_slam_runner
```

### 3.2 启动 Dear ImGui 桌面端（需 Windows 11 WSLg 或 X Server）

```bash
# 检查显示环境
echo "DISPLAY=$DISPLAY WAYLAND_DISPLAY=$WAYLAND_DISPLAY"

# 启动完整 SLAM Station 主窗口
./slam_station

# 或仅启动独立免 ROS 标定工作台
./slam_station --calibrator
```

> **WSLg 显卡驱动提示**：若在 WSLg 中遇到 OpenGL 3.0+ 上下文创建失败或黑屏，可尝试强制软件渲染验证：
> ```bash
> LIBGL_ALWAYS_SOFTWARE=1 ./slam_station
> ```

### 3.3 手动单独重跑 CTest

```bash
ctest --test-dir ~/.cache/eds-cpp-linux-build/build/Release --output-on-failure
```

---

## 4. 常见问题排查

1. **`conan install` 提示 `system_requirements` 缺系统包（如 `xorg/system`、`opengl/system`）**：
   - 有 `sudo` 权限时加 `-c tools.system.package_manager:mode=install -c tools.system.package_manager:sudo=True`，或先按第 1.1 节手动 `apt-get install`。
2. **`Pangolin` 提示缺少 `epoxy` 或 `EGL`**：
   - 安装 `libepoxy-dev libegl1-mesa-dev`。
3. **中文字体显示为方框**：
   - 安装 `fonts-wqy-microhei` 或 `fonts-noto-cjk`，`platform_ui.cpp` 会自动扫描 `/usr/share/fonts/` 下的文泉驿与 Noto CJK 字体。
4. **重新清理 Linux 构建缓存**：
   ```bash
   rm -rf ~/.cache/eds-cpp-linux-build
   ```
   不会影响 `~/.conan2` 已编译好的第三方库缓存，也不会影响 Windows 侧构建目录。
