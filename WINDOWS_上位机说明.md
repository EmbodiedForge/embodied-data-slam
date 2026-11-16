# SLAM Station 0.2 · Windows 本地 Docker 上位机

新上位机位置：`software/slam_station`。原 `software/umi_slam` 与 Docker 算法保持原样，新旧界面可以分别启动。新版本提供任务启动/停止、实时日志、设备模板、标定快照、历史结果、轨迹报告及可选目标坐标系导出。

## Windows 使用

1. 使用 Windows x64；安装 Python 3.10 或更高版本，保留 Python 安装程序中的 Tcl/Tk 组件。安装并启动 Docker Desktop，启用 WSL 2 后端并使用 Linux containers。具体系统要求以 [Docker 官方安装说明](https://docs.docker.com/desktop/setup/install/windows-install/) 为准。
2. 将整个工具包解压到本机磁盘。中文、空格、`&` 和 Windows 盘符可作为路径；不需要改旧电脑的路径。
3. 第一次双击 `load_images_windows.cmd`。它核对原镜像归档的 SHA256，再导入 `umi-slam:0.1`。这个明确的导入操作可能更新本机同名镜像标签。不自动下载算法镜像。
4. 双击 `launch_windows.cmd`，点击“检查 Docker”。Python 版上位机仅用标准库，无需宿主机安装 OpenCV、NumPy 或 ROS。
5. 选择采集目录、结果保存目录、对应设备的标定 YAML、设备 JSON。选择“仅检查并准备数据”可先验证输入；取消勾选后运行完整 SLAM。
6. 完成后查看 HTML 报告或打开任务目录。报告可离线打开。停止/关闭窗口会请求停止本任务容器，等待退出后关闭。

现有 UMI 数据可使用“填入现有 UMI 示例”，它填写随包的示例路径和历史标定。若使用不含 examples 的轻量包，另外选择真实数据目录。新设备使用“设备与扩展”里的 UMI/EGO 模板，另存后填写实际参数；模板不会自动生成标定。

Docker 计算运行在 Linux 容器内，Windows 上运行界面。无需 GPU；当前后端主要使用 CPU。大量小图片在 Windows 文件系统挂载下可能慢于 WSL Linux 文件系统，见 [Docker WSL 性能说明](https://docs.docker.com/desktop/features/wsl/best-practices/)。输入始终只读挂载，结果写入新的任务目录。

## 输出

```text
SLAM_Results/run_日期_随机ID/
  station_job.json          # 上位机状态、参数、镜像 ID、设备信息
  host.log
  config/settings.yaml     # 标定快照
  config/device.json       # 完整设备接口
  config/profile.json      # 传给现有算法的数据配置
  result/
    job.json / slam.log / prepared/
    imu_trajectory.csv / imu_trajectory.tum
    quality.json / report.html / tracking.csv
    target_trajectory.csv / target_frame.json  # 仅提供合法外参时生成
```

IMU 默认世界系沿用 ORB-SLAM3 地图，不是机器人底座；coverage 仅表示轨迹覆盖率，不是精度评价。UMI/EGO 外参、时间同步及辅助流接口详见 [DEVICE_API.md](software/slam_station/DEVICE_API.md)。

## 可选打包 EXE

本次交付的是可在 Windows 用 Python 启动的完整源码包，未在本 Ubuntu 工作站上生成或验证原生 Windows EXE。

在 Windows PowerShell 中运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\software\slam_station\build_windows.ps1
```

脚本在独立 `.venv-build` 中安装 PyInstaller（构建时需要联网），生成 `software\slam_station\dist\SLAMStation\SLAMStation.exe`。分发整个 SLAMStation 目录，不能只取 exe。打包后目标机不需要 Python，仍然需要 Docker Desktop 和算法镜像。需要示例时将工具包 `examples` 放到 exe 所在目录。

[PyInstaller 官方说明](https://www.pyinstaller.org/en/stable/) 要求在目标操作系统上构建对应可执行文件。本次 Windows 真机的 Docker Desktop 挂载、桌面操作和 EXE 构建仍需要 Windows 验收，Linux 回归测试不能替代这项验证。

## Ubuntu 启动与命令行

```bash
bash launch_station.sh
python3 software/slam_station/cli.py diagnose
python3 software/slam_station/cli.py run \
  --source examples/lissajous_20260915/input \
  --parent results_station \
  --settings software/umi_slam/profiles/umi_20260908.yaml \
  --device software/slam_station/devices/umi_20260908.json --stride 5
```

`--prepare-only` 仅准备；`--max-frames 200` 限制诊断输入；`--timeout 0` 自动计算后端超时。GUI 不需要联网，只有首次安装 Docker/Python 或构建 EXE 需要外部软件。

## 当前边界

- 已有 UMI 原始数据、标准双目+IMU EuRoC 可执行；新 UMI/EGO 私有协议尚未确定，预留适配契约和模板。
- EGO 模板不包含单目 SLAM、人体骨骼或手势估计。
- 有效 `T_imu_target` 支持刚体目标坐标导出；外参空白时不会假造夹爪或头部位姿。
- 辅助流当前只做文件声明与路径检查，不参与融合。
- Kalibr 保留现有工具包入口，本次未集成 Windows 标定按钮。
- 原离线包的 MANIFEST.json / SHA256SUMS 描述历史 0.1 内容；新增上位机有独立发布清单，不改历史校验文件。
