# UMI 双目 IMU 轨迹后处理 · 0.1

Ubuntu 桌面界面、命令行和 Docker 共用一条离线处理流水线。现阶段输出 **IMU 原点的位姿**，尚不输出钢轨中点/夹爪 TCP，也不需要 IMU 到夹爪的外参。相机与 IMU 的标定仍然必需。

## 当前支持

- 现有 UMI：`stereo.mkv`、`umi_video_timestamps.csv`、`umi_imu_timestamps.bin`。
- EuRoC 风格目录：左右图片、相同的纳秒时间戳、IMU CSV；支持 PNG/JPG。
- 检查时间单调性、标定分辨率、IMU 频率/单位疑点/缺口、图像解码和惯性初始化。
- 每次运行创建独立目录；不会覆盖既有结果。后端无窗口运行。
- CSV/TUM 导出、逐帧跟踪状态、日志、质量报告、离线 HTML 三个正交投影图。
- 停止、超时、崩溃、未完成惯性初始化均不会标为成功。

## 本机启动

在工程根目录：

```bash
bash umi_slam/launch.sh
```

`python3 umi_slam/install_launcher.py` 可添加用户级应用菜单入口「UMI 轨迹恢复」，不需要管理员权限。

界面选择采集目录、结果父目录和标定后开始处理。默认每 5 帧取一帧，适用于现有 125 fps 慢速实验（约 25 fps）；不是所有设备的通用最佳设置。附带标定只用于 2026-09 的现有 UMI，不适用于新 UMI/EGO。

### 首次在其他 Ubuntu 22.04 主机安装

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev libopencv-dev \
  libboost-serialization-dev libssl-dev libgl1-mesa-dev libegl1-mesa-dev \
  libepoxy-dev libglew-dev libjpeg-dev libpng-dev \
  python3 python3-numpy python3-opencv python3-tk
python3 umi_slam/build_backend.py --jobs 2
bash umi_slam/launch.sh
```

需要同时带上 `stereo_imu_trajectory/ORB_SLAM3` 和 `stereo_imu_trajectory/Pangolin` 源码。构建脚本在 `.build` 内制作独立副本；原实验源码不修改。源码变更后使用新的 `--build-dir`，并在界面选择其 `backend.json`。已编译结果内含绝对路径，移动工程后应重新构建。

## 命令行

```bash
python3 umi_slam/pipeline.py \
  --input data/handeye_validation_20260915_160344/20260915_160354 \
  --output outputs/umi_imu_run_001 \
  --settings umi_slam/profiles/umi_20260908.yaml \
  --profile umi_slam/profiles/umi_20260908.json \
  --stride 5
```

`--output` 必须为不存在的新目录，且不能与输入目录互相嵌套。`--prepare-only` 仅做预检及转换；`--max-frames 200` 可限制诊断数据量（短序列可能无法初始化）。`--timeout` 为后端超时秒数，0 表示按输入时长计算。

重跑已准备的数据时，可用 `--prepared-input --input <之前的结果>/prepared`，并提供原来的 `--settings` 和新的结果目录。此时沿用已准备的抽帧和时间配置，`--stride`、`--max-frames`、`--profile` 不重新作用于数据；需要更改这些参数时重新准备原始数据。

EuRoC 输入结构：

```text
dataset/
  timestamps.txt                  # 每行一个整数纳秒时间戳，可用左右 data.csv 替代
  mav0/cam0/data/<timestamp>.png
  mav0/cam1/data/<timestamp>.png
  mav0/imu0/data.csv               # ns,gx,gy,gz,ax,ay,az；rad/s、m/s²
```

如果使用 `mav0/cam0/data.csv` 和 `cam1/data.csv`，每行为 `timestamp_ns,filename`，表头以 `#` 开始。第一版要求左右时间戳一一对应；异步双目需先完成明确的同步处理，不会静默配对。标准 EuRoC 输入可省略 `--profile`。

原始 UMI 的 UART 协议/量程和均匀时间重建是设备专用适配。它假设传感器未丢样，不能消除 UART 延迟或时钟漂移。新设备优先提供每个 IMU 样本的硬件时间戳。配置 `camera_to_imu_offset_ns` 的正号定义为 `t_imu = t_camera + offset`；导出保持原图像时间戳。

## Docker

本机已编译后，可利用已有的固定版本 Ubuntu 22.04 ORB-SLAM 基础镜像离线打包同一后端：

```bash
python3 umi_slam/build_runtime_docker.py --tag umi-slam:0.1
```

此入口要求 Ubuntu 22.04 x86_64、系统 Python 3.10 及本机后端，打包其动态库、Python 数值依赖和对应源码，镜像内运行与本机隔离。基础镜像固定为脚本中的 digest；不使用基础镜像的旧 SLAM 可执行程序。

从 Ubuntu 基础镜像全量编译的备选入口：

```bash
python3 umi_slam/build_docker.py --tag umi-slam:0.1 --jobs 2
```

该命令只发送源码和词袋压缩包，兼容 legacy builder。不要直接对包含大量原始数据的整个工程执行无忽略规则的 `docker build .`。

从工程根目录运行：

```bash
mkdir -p outputs/docker_umi
docker run --rm --init --network none \
  --user "$(id -u):$(id -g)" \
  -v "$PWD/data/handeye_validation_20260915_160344/20260915_160354:/input:ro" \
  -v "$PWD/outputs/docker_umi:/output" \
  -v "$PWD/umi_slam/profiles:/config:ro" \
  umi-slam:0.1 \
  --input /input --output /output/run001 \
  --settings /config/umi_20260908.yaml \
  --profile /config/umi_20260908.json --stride 5
```

其他设备的标定目录通过只读挂载传入。桌面界面可选择「Docker」或「本机」运行方式；没有本机 backend.json 的便携副本默认使用 Docker，并隐藏本机后端配置。Docker 计算不需要 GPU、ROS、X11 或访问设备串口；桌面界面需要宿主机 Python Tk。全量源码构建镜像需要联网安装 Ubuntu 依赖；运行可完全离线。构建参数和源码指纹记录在 `backend.json`。源码 Dockerfile 的 Ubuntu 基础镜像及 apt 仓库尚未做完全版本锁定。

复制到另一台 x86_64 Docker 主机：在本机运行 `docker save umi-slam:0.1 | gzip > umi-slam_0.1.tar.gz`，传输文件后运行 `docker load < umi-slam_0.1.tar.gz`，随后使用上面的只读输入挂载命令。

## 输出与坐标约定

| 文件 | 内容 |
|---|---|
| `imu_trajectory.csv` | 原图像纳秒时间戳、相对输入起点时间、IMU 位置（米）、四元数 xyzw、segment_id |
| `imu_trajectory.tum` | 时间戳换算为秒的同一组 IMU 位姿 |
| `imu_euroc.txt` | ORB-SLAM3 原始导出，可能含初始化前帧；主要使用过滤后的 CSV/TUM |
| `tracking.csv` | 每个处理帧的跟踪状态、当时惯性初始化状态、地图 ID |
| `quality.json` / `report.html` | 覆盖率、时间范围、分段及警告，不是精度认证 |
| `job.json` / `slam.log` | 运行参数、后端来源、状态和日志 |
| `prepared/` | 自包含中间数据和实际运行标定；PNG 转换可能需要数 GB 空间 |

`T_world_imu` 把 IMU 坐标变换到导出世界系；姿态是 IMU 相对该世界系的朝向。世界原点/轴沿用 ORB-SLAM3 导出所选地图首个关键帧的 IMU 坐标系，不是机器人底座，也不保证第一张输入图像恰为原点。时间戳沿用相机时钟，不能直接当 Unix 时间使用。

只发布跟踪状态为 OK 且当时惯性已经初始化的位姿，不补齐失跟区间。ORB-SLAM3 默认只导出关键帧最多的地图；地图切换会提示，未导出的其他地图不会被拼接成一条看似连续的轨迹。`segment_id` 区分缺口和地图切换。轨迹覆盖率的分母为选中处理的图片数量，不是采集原始帧总数。

## 实验版本的限制

- 打包的是当前工程的 ORB-SLAM3 分支，保留其慢速初始化阈值修改。附带 YAML 开启 `IMU.fastInit`，用于复现既有慢速 Lissajous 实验，不应直接认定为新设备默认参数。
- 打包副本修复 Shutdown 未等待后台线程的问题，新增空地图保护与有边界检查的离线读取入口；关闭时不再并发周期导出轨迹。进程崩溃时保留日志及逐帧状态，不承诺恢复完整轨迹。
- 运行成功和高覆盖率不代表达到毫米级准确度。已有手眼/姿态一致性问题仍需独立排查；本软件没有以机器人参考轨迹替换 SLAM 输出。
- 新设备仍需匹配的双目和相机—IMU 标定、量程、轴序、时间同步；当前没有自动标定功能。

## 检查

```bash
python3 -m unittest discover -s umi_slam/tests -v
```

ORB-SLAM3 及各依赖的许可证见随附源码的 License/Dependencies 文件；镜像保留源码及原许可证。
