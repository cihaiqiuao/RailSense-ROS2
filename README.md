# RailSense-ROS2

RailSense-ROS2 是一个基于 ROS 2 Humble 的列车多源感知与运行状态评估系统，工程内部包名统一使用 `trainros_*`。系统面向 Linux 边缘计算平台，将 IMU、GPS、激光测距和视觉检测接入同一 ROS 2 数据链路，并提供状态融合、平稳性评估、运行诊断、事件日志和 rosbag2 数据录制能力。

## 系统架构

```text
IMU / GPS / Laser / Camera
           |
           v
  ROS 2 Drivers + Diagnostics
           |
           +----------> YOLO Detection
           |                   |
           v                   v
       Sensor Fusion <---------+
           |
           v
   Train State Estimation
           |
           v
   Stability Evaluation
           |
           +----------> Monitor / Logger / Recorder
```

源码按职责分为六层：

```text
src/
  interfaces/   # 自定义 Message 和 Action
  drivers/      # IMU、GPS、Laser、Camera 数据接入
  perception/   # YOLO 视觉检测
  estimation/   # 多传感器融合与平稳性评估
  runtime/      # 监控、业务日志与 rosbag2 录制
  bringup/      # Launch、参数和部署配置
```

## 核心能力

- 传感器接入：支持 IMU 33 字节协议、GPS NMEA RMC 和激光测距协议解析，包含串口重连、帧统计与异常诊断。
- 多源融合：使用 `message_filters::ApproximateTime` 同步 IMU/Laser，以 GPS 作为低频校正源、视觉结果作为最近值缓存，通过轻量 Kalman Filter 估计位置、速度和加速度。
- 平稳性评估：基于滑动窗口计算 RMS、峰值、频带 PSD 和 TSI，输出评分及 `normal`、`warning`、`alarm` 状态。
- 视觉链路：支持本地视频发布到 `/camera/image_raw`，并通过 ONNX Runtime 节点输出 `/coupler_detection`；依赖或模型不可用时进入可观测的降级状态。
- 可观测性：传感器、融合、评估和系统监控统一发布 `/diagnostics`，覆盖串口状态、丢帧、数据新鲜度和链路延时。
- 数据闭环：Logger 将关键运行事件写入 JSONL；Recorder 通过 `Record.action` 管理 `ros2 bag record` 子进程，实现数据录制、取消和落盘。

## ROS 2 包

| 分层 | 包 | 职责 |
| --- | --- | --- |
| Interfaces | `trainros_interfaces` | 定义 `Coupler`、`TrainState`、`Stability`、`SystemStatus` 和 `Record.action` |
| Drivers | `trainros_imu_driver` | IMU 串口读取、协议解析与诊断 |
| Drivers | `trainros_gps_driver` | GPS NMEA 解析、定位状态与诊断 |
| Drivers | `trainros_laser_driver` | 激光测距解析、距离发布与诊断 |
| Drivers | `trainros_camera_driver` | Camera 节点和本地视频 Topic 发布 |
| Perception | `trainros_yolo_detection` | ONNX 视觉推理与车钩状态发布 |
| Estimation | `trainros_fusion` | 多源同步、Kalman 状态融合与时延统计 |
| Estimation | `trainros_stability_evaluator` | RMS、PSD、TSI 和告警等级计算 |
| Runtime | `trainros_monitor` | Topic 存活状态与端到端延时监控 |
| Runtime | `trainros_logger` | 结构化业务事件日志 |
| Runtime | `trainros_recorder` | rosbag2 录制 Action 服务 |
| Bringup | `trainros_bringup` | 系统 Launch 与分层参数配置 |

## 主要 Topic 与接口

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU 数据 |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | GPS 定位数据 |
| `/laser/scan` | `sensor_msgs/msg/LaserScan` | 激光测距数据 |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 相机或视频图像 |
| `/coupler_detection` | `trainros_interfaces/msg/Coupler` | 视觉检测结果 |
| `/train_state` | `trainros_interfaces/msg/TrainState` | 融合后的运行状态 |
| `/stability` | `trainros_interfaces/msg/Stability` | 平稳性指标与等级 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 全链路运行诊断 |
| `/record` | `trainros_interfaces/action/Record` | rosbag2 录制控制 |

## 构建

开发与验证环境为 Ubuntu 22.04、ROS 2 Humble 和 colcon。建议在 Linux 原生文件系统中构建；WSL 下直接在 drvfs 挂载盘使用 `--symlink-install` 可能受到符号链接权限影响。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动

启动完整系统：

```bash
ros2 launch trainros_bringup trainros_system.launch.py
```

无硬件环境可使用三路 PTY 伪串口启动端到端链路：

```bash
ros2 launch trainros_bringup trainros_fake_serial.launch.py \
  imu_port:=/dev/pts/1 \
  gps_port:=/dev/pts/2 \
  laser_port:=/dev/pts/3
```

使用本地视频验证 Camera 到 YOLO 的链路：

```bash
ros2 launch trainros_bringup trainros_video_yolo.launch.py \
  video_path:=/path/to/video.mp4
```

更完整的构建、Topic、参数、日志和 rosbag2 操作见 [ROS2 命令与使用说明](docs/ROS2_COMMANDS_AND_USAGE.md)，系统边界与 QoS 设计见 [架构说明](docs/ARCHITECTURE.md)。

## 配置

运行参数集中在 `src/bringup/trainros_bringup/params/`：

- `sensors.yaml`：真实传感器、Camera 和 YOLO 参数。
- `sensors_fake_serial.yaml`：伪串口测试参数。
- `fusion.yaml`：同步窗口、超时阈值、Kalman 与平稳性参数。
- `logging.yaml`：业务日志、rosbag2 Topic 和输出路径。
- `monitor.yaml`：节点存活与延时监控参数。

## 测试与验证

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

当前自动化测试覆盖：

- IMU、GPS、Laser 协议解析单元测试。
- 伪串口到 ROS 2 Topic、Fusion、Stability 和 Diagnostics 的端到端测试。
- Kalman 状态更新与平稳性指标计算测试。
- Logger、Recorder、Monitor 和 Launch 参数测试。
- Camera/YOLO 缺少视频、模型或运行依赖时的降级路径。
- C++ 节点的 AddressSanitizer 与 UndefinedBehaviorSanitizer 验证入口。

## 工程边界

- 已完成 12 个 ROS 2 包的构建及无硬件端到端验证。
- 串口协议、融合、评估、监控、日志和录制链路已有自动化测试覆盖。
- 真实列车传感器、RK3588 平台 Camera/GStreamer、NPU/TensorRT 加速和现场参数标定仍需结合目标硬件验证。
