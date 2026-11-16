# UMI / EGO 设备接口 v1

这是一套离线后处理接口。当前算法是双目惯性 ORB-SLAM3，输出 `T_world_imu`。设备系列标签不是算法能力声明：EGO 的单目视频、人体骨骼、手势识别和多刚体融合尚未实现。

## 设备配置

`devices/` 提供现有 UMI、通用 EuRoC、新 UMI、EGO 四个 JSON。先另存模板，填写真实标定与协议参数；界面选择 JSON 和本设备的 ORB-SLAM3 YAML。切换模板会清空旧标定路径，避免误用。

| 字段 | 约定 |
|---|---|
| schema_version | 当前为 1，其他版本拒绝执行 |
| id / name / family | 唯一设备标识、显示名称、UMI / EGO / generic |
| adapter | `umi_ms901m_uniform` 或 `euroc`；未知协议拒绝执行 |
| timestamp_unit / timestamp_clock | `ns`；实际时钟名称，不默认是 Unix 时间 |
| gyro_unit / accel_unit | `rad/s` / `m/s^2` |
| stereo_sync | `exact`，左右图像必须拥有相同时间戳 |
| camera_to_imu_offset_ns | 整数；正号表示 `t_imu = t_camera + offset` |
| imu_hz / accel_range_g / gyro_range_dps | 仅当前 MS901M 原始协议需要，不用于猜测新硬件量程 |
| target_frame | 例如 `gripper_tcp`、`head_rig`；只用作元数据 |
| T_imu_target | 4×4 齐次刚体外参或 null；平移单位为米 |
| auxiliary_streams | 夹爪开合/关节/手部等辅助文件声明，当前只验证路径并保存配置，不参与 SLAM 或融合 |

时间偏移模板值 0 只是待填写的初始值，不代表已测量同步。原始数据和标定采用一致的轴顺序；不在本层偷偷交换坐标轴。每个样本时间戳优先使用硬件时钟。

## 新采集设备接入

先实现 `devices.DatasetAdapter.normalize(source, destination, device) -> Path`，将设备原始数据转换到新的、自包含 EuRoC 目录。该 Protocol 是开发扩展契约，当前没有动态插件加载器；集成程序需显式调用适配器，再将返回目录提交给 `jobs.Job`，选择 `adapter=euroc` 的设备 JSON。

```text
dataset/
  mav0/cam0/data.csv      # timestamp_ns,filename（# 开头为注释）
  mav0/cam0/data/*.png
  mav0/cam1/data.csv
  mav0/cam1/data/*.png
  mav0/imu0/data.csv      # timestamp_ns,gx,gy,gz,ax,ay,az
```

图片也支持 JPG；另一种方式是根目录 `timestamps.txt` 配合 `<timestamp>.png/jpg`。时间戳需严格递增，双目一一对应；异步双目必须在适配阶段显式完成同步。IMU 频率、图像尺寸和相机/IMU 外参需要匹配 YAML。

独立开发适配器时至少验证：时间戳精度与单调性、丢样、IMU 单位、轴定义、双目配对、图像尺寸，以及标定来源。未知格式不能只改 family 标签就当作已支持。

## 目标坐标系

定义 `T_A_B` 将 B 系坐标变换到 A 系：

```text
T_world_target = T_world_imu @ T_imu_target
```

`T_imu_target` 必须来自真实外参测量，不能将未知外参填单位阵伪装成已标定。空值时只输出 IMU 轨迹。合法外参会增加 `target_trajectory.csv` 与 `target_frame.json`，保留原 IMU CSV、纳秒时间戳、断段 segment_id；不插补失跟段，不变换到机器人底座。

辅助流预留示例（文件必须存在，路径相对于输入目录）：

```json
{"name":"gripper_width", "kind":"scalar_csv", "path":"aux/gripper.csv", "timestamp_clock":"camera_hardware_clock"}
```

建议 CSV 为 `timestamp_ns,value`，量纲、时间偏移、轴系由新增适配器显式声明。当前版本不会自动读入这些数值、时间对齐或生成人体运动学结果。

## 程序接入

从 `software/slam_station` 导入 `Job`（仅 Python 标准库，Docker 负责计算）：

```python
from jobs import Job
job = Job(source='capture', parent='results', settings='calibration.yaml',
          device='my_device.json', stride=5)
job.start()                 # 后台执行
# job.stop()                # 请求停止本任务的命名容器
# job.active                # 是否仍在运行
# job.folder / job.result   # 可审计任务目录 / 算法输出
```

每个任务生成独立目录，保存 `station_job.json`、设备/标定快照、`host.log`、后端输出及实际镜像 ID。状态为 queued/checking/running/prepared/completed/failed/cancelled。只有退出码为 0 且后端 job.json 状态符合预期才标为成功。结果不覆盖。
