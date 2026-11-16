# SLAM Station 0.2 验证记录 · 2026-09-22

环境：Ubuntu 工作站，真实本机 Docker Linux amd64 后端；Windows 路径逻辑有单元测试，尚无 Windows 真机。

- 新增 10 项单元测试通过：Windows 盘符/中文/逗号挂载编码、Windows 不调用 Unix UID、只读挂载、输入输出隔离、未知协议/单位/外参拒绝、坐标变换、180° 旋转、纳秒时间戳与断段保留、启动前取消、容器创建期间取消重试、配置快照和防止误报成功。
- 原有 9 项 SLAM 工具测试通过。
- Linux 虚拟显示 GUI 冒烟通过：窗口创建、Docker 检查、示例填入、EGO 模板切换清空旧标定、正常关闭；截图为 `界面预览.png`。
- GUI 真实 Docker 准备数据通过（20 帧）；输出路径含中文、空格、逗号、`&`，界面正确显示准备完成并登记历史任务。
- 真实 Docker 中途取消通过：上位机状态 cancelled，命名容器已删除，无遗留任务容器。
- 真实参考轨迹的目标坐标导出通过；新建目标输出，保留原始 IMU 文件及时间戳/断段。
- 完整 UMI 示例通过：4,047 个选中双目帧，后端和上位机状态 completed，退出码 0，覆盖率 87.571%。CSV/TUM 行数与质量报告一致。覆盖率不是精度认证。
- 完整结果：工具包 `validation_station/run_20260922_153919_ad451e50/result/`。后端实际 ID：`sha256:a057d1331173a39bfbd788a90cd7fd0e9f4cec1692e76c18bb576b0c188eddef`；上位机记录运行时 ID，不用历史清单中的 ID 代替实测。

本次没有重新编译 ORB-SLAM3，也没有更改原始算法镜像。旧包清单列出的历史镜像 ID 与本机标签 ID 不同；原镜像归档仍按原 SHA256SUMS 校验。完整序列的本次指标仅对应上述本机实际 ID。

待 Windows 验收：安装 Python/Tk 与 Docker Desktop 后的双击启动、驱动器共享/挂载权限、停止与关闭，以及可选 PyInstaller EXE 构建。EXE 构建脚本已提供，未宣称已生成原生 Windows 可执行文件。

测试命令：

```bash
python3 -m unittest discover -s software/slam_station/tests -v
python3 -m unittest discover -s software/umi_slam/tests -v
```
