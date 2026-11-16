# UMI 轨迹恢复与 Kalibr 离线工具包

目标工作站：Ubuntu 22.04，x86_64。此包由 2026-09-21 已运行的工程导出。轨迹软件输出 IMU 原点位姿；夹爪钢轨中点的外参尚未加入。

## 到工作站后

建议把整个 `UMI_Toolkit_20260921` 文件夹复制到工作站内置 SSD，例如 `~/UMI_Toolkit_20260921`。保留原 U 盘副本作为备份。大量图像的解码、保存和读取使用内置 SSD 通常更合适。

工作站需要已经安装 Docker Engine、Python 3；桌面界面另外需要 `python3-tk`。镜像包不包含宿主机的 Docker 安装程序。检查：

```bash
docker info
python3 -c 'import tkinter'
```

如果缺 Tk，联网时可安装 `sudo apt install python3-tk`；不使用界面时不需要 Tk。Docker 权限问题应先在工作站解决，不要以 root 身份启动桌面软件。

进入复制后的工具包目录：

```bash
cd "$HOME/UMI_Toolkit_20260921"
bash load_images.sh
bash launch_software.sh
```

`load_images.sh` 会先验证 SHA256，再从文件导入镜像并核对镜像 ID；不拉取网络镜像。再次导入也可行。界面运行方式选 **Docker**，无需编译 ORB-SLAM3，也不依赖原电脑的 `/home/jt/Projects/SLAM` 路径。

可选：复制到固定位置后执行 `python3 software/umi_slam/install_launcher.py`，添加「UMI 轨迹恢复」应用菜单入口。以后移动文件夹需重新安装入口；脚本不会静默覆盖指向其他位置的旧入口。

## 先试一遍现有轨迹

界面选择：

- 数据：`examples/lissajous_20260915/input`
- 标定：`software/umi_slam/profiles/umi_20260908.yaml`
- 数据配置：`software/umi_slam/profiles/umi_20260908.json`
- 运行方式 Docker，抽帧间隔 5。结果目录建议放工作站内置 SSD。

命令行等价入口（不需要 Tk，也不需要宿主机 OpenCV）：

```bash
python3 software/umi_slam/docker_runner.py \
  --input examples/lissajous_20260915/input \
  --output results/lissajous_run001 \
  --settings software/umi_slam/profiles/umi_20260908.yaml \
  --profile software/umi_slam/profiles/umi_20260908.json --stride 5
```

输出目录必须是新目录。已有参考输出、三维轨迹 PNG、HTML 和质量报告放在 `examples/lissajous_20260915/reference_result`，可直接查看。完整流程已在原机器跑通，覆盖率约 82%–84%，包含初始化和失跟缺失；不是精度认证。

## Kalibr：内参、双目外参、相机—IMU 外参和时间偏移

`kalibr:portable-20260921` 内含 Ubuntu 20.04、ROS Noetic 和 Kalibr，并补齐历史镜像缺少的 igraph。**宿主机保持 Ubuntu 22.04，不需要再装 Ubuntu 20.04 系统、虚拟机或 ROS。** 镜像中还保留了原 `kalibr:ros1-20.04` 和 `ubuntu:20.04` 标签，供追溯/重建。

目前 Kalibr 提供独立脚本入口，没有并入轨迹软件的按钮。所有命令均以普通用户离线运行，工作目录挂载为容器内 `/work`，工具包挂载为 `/toolkit`。

下面是工作目录约定，建议放在工作站 SSD：

```text
calibration_work/
  session/             # stereo.mkv、umi_video_timestamps.csv、umi_imu_timestamps.bin
  target.yaml          # 符合实际标定板尺寸
  imu.yaml             # 实际 IMU 的噪声、随机游走和频率
```

`imu.yaml` 的输入格式可参考 `calibration/templates/imu_ms901m_example.yaml`，其中参数在顶层。`reference_20260908/stereo_imu-imu.yaml` 是 Kalibr 输出的嵌套格式，不要直接当作 `--imu` 的输入文件；模板中的历史噪声参数也必须按实际传感器确认。

第一步，原始数据生成 bag（不需要预先知道相机内参）：

```bash
bash calibration/kalibr.sh "$HOME/calibration_work" bag \
  --input /work/session --output /work/dataset.bag --stride 5 \
  --imu-hz 250 --accel-range-g 4 --gyro-range-dps 2000
```

该转换器针对当前 MS901M 原始协议，输入时间戳是相机驱动纳秒时间，IMU 使用历史均匀时间重建。新传感器不能盲目套用量程、轴序或时间重建。仅做双目标定可加 `--images-only`，无需 IMU 文件。原有参考 bag `examples/calibration_20260908/stereo_imu.bag` 已含 `/cam0/image_raw`、`/cam1/image_raw` 和 `/imu0`，可复制到工作目录试用。

第二步，标定相机内参和左右相机外参：

```bash
bash calibration/kalibr.sh "$HOME/calibration_work" stereo \
  --bag /work/dataset.bag --target /work/target.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw
```

第三步，使用上一步生成的 camchain 标定相机—IMU 外参和时间偏移：

```bash
bash calibration/kalibr.sh "$HOME/calibration_work" imu \
  --bag /work/dataset.bag --target /work/target.yaml \
  --cams /work/dataset-camchain.yaml --imu /work/imu.yaml
```

Kalibr 的实际输出文件名以日志为准；上面的 `dataset-camchain.yaml` 按 bag 文件名示例书写。标定程序输出 PDF 和 YAML，不打开容器内图形窗口。

第四步，将相机—IMU 标定输出转换为轨迹软件配置：

```bash
bash calibration/kalibr.sh "$HOME/calibration_work" convert \
  --cam /work/dataset-camchain-imucam.yaml \
  --imu /work/imu.yaml --output-dir /work/slam_config --fps 25 \
  --capture-profile /toolkit/software/umi_slam/profiles/umi_20260908.json
```

`--capture-profile` 只在硬件协议/量程仍与旧 UMI 一致时使用；新设备必须更新。EuRoC 数据可省略它。输出 `orbslam.yaml` 和 `device_profile.json`，两者都应在轨迹软件中选择，后者携带标定得到的时间偏移。转换器目前支持双目 pinhole-radtan，生成配置默认关闭实验性的 fastInit。

这个转换遵循当前打包后端的矩阵约定，已与历史标定矩阵数值核对。噪声密度、随机游走不能由相机内参标定替代；模板 IMU 参数不是新硬件实测结果。Kalibr 的这一步也不负责 IMU 到夹爪中心的几何外参。

## 标定板尺寸必须看这里

- 用户已确认当前实物标签边长为 **34.5 mm**，使用 `calibration/targets/aprilgrid_6x6_measured34p5mm.yaml`，其中 `tagSize=0.0345`（单位：米）。历史 2026-09 标定也使用这个尺寸，无需因本次确认修改旧标定结果。
- `calibration/targets/aprilgrid_6x6_nominal40mm_A3.pdf` 是历史标定板 PDF，文件名中的 40 mm 不代表当前实物尺寸；`nominal40mm.example.yaml` 仅是示例，不用于当前板。
- 当前配置沿用历史 6×6 布局及 `tagSpacing=0.3`；本次确认的是标签边长，未重新确认间距比例。
- `tagSpacing` 是标签间距除以标签边长的比例，不是米。
- 不要仅凭 PDF 文件名选尺寸。打印缩放、重新打印或更换标定板后，测量实体标签的黑色正方形边长和标签间距，再修改 YAML。尺寸错误会影响基线、外参和轨迹尺度。

`calibration/reference_20260908` 中的旧标定和报告只用于复现当前硬件，不是新硬件的标定。

## 包内内容与性能

| 目录/文件 | 用途 |
|---|---|
| `software/umi_slam` | 可迁移的桌面界面、Docker 入口、命令行和测试 |
| `images/*.tar.gz` | 已验证镜像的 Docker save 离线归档 |
| `calibration/` | Kalibr 包装脚本、bag 转换、配置转换、标定板与参考参数 |
| `examples/` | 原始 Lissajous 数据及参考结果、一份历史标定 bag |
| `source/SLAM_source.tar.gz` | 软件、原 ORB-SLAM3/Pangolin 源码、隔离构建脚本、Kalibr Dockerfile |
| `MANIFEST.json` / `SHA256SUMS` | 版本、镜像 ID、文件校验 |

当前 ORB-SLAM3 和 Kalibr 主要使用 CPU；这套软件没有 CUDA/GPU 计算路径。5090 显卡本身不会直接加速它们。当前 SLAM 入口为保持后台优化节奏按采集时间间隔运行，因此更快 CPU 可以改善处理余量，但不保证总耗时按显卡性能缩短。本版本尚未提供经过验证的无等待极速模式。

U 盘为 exFAT。源码用 tar.gz 保存，以保留 Linux 权限和符号链接；若需源码构建，把归档解压到工作站的 Linux 文件系统后操作。日常使用推荐已经打包的 Docker 后端。宿主机原生构建需要依赖安装，旧机器的绝对路径编译缓存没有直接复制为可用软件。
