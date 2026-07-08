# TrainROS 项目记忆

## 项目目的

TrainROS 是对现有列车平稳性与车钩识别工作的 ROS2 Humble 工程化重构。目标是把耦合度较高的 Qt/Python 原型逐步改造成模块化 ROS2 平台，用于多源传感器接入、视频推理、时间同步、运行状态融合、平稳性评估、rosbag2、RViz、日志和系统监控。

## 当前工作区

- 新工作区：`E:\ros2\TrainROS`
- 旧工程根目录：`E:\毕业设计\03_代码工程`
- 当前阶段：基础软件闭环已形成，真实硬件和视觉推理尚未接入。
- 使用说明：`docs/ROS2_COMMANDS_AND_USAGE.md` 记录构建、无硬件运行、Topic/Node/Param/Action、Logger 和 rosbag2 命令。
- 模型转换说明：`docs/MODEL_CONVERSION.md` 记录车钩 YOLOv8-Pose `.pt` 到 ONNX 的转换结果。

## 包结构

- `trainros_interfaces`：自定义消息和 `Record.action`。
- `trainros_imu_driver`：IMU 串口 driver，33 字节帧解析、校验、重连、统计和 diagnostics。
- `trainros_gps_driver`：GPS/NMEA driver，RMC 解析、fix/no-fix、重连、统计和 diagnostics。
- `trainros_laser_driver`：激光测距 driver，`0xAA` 12 字节帧解析、重连、统计和 diagnostics。
- `trainros_camera_driver`：Camera/GStreamer 占位框架，并提供 `video_file_publisher` 将本地 MP4 发布到 `/camera/image_raw`。
- `trainros_yolo_detection`：YOLO 检测框架，已放入 `coupler_yolov8_pose_best.pt` 和导出的 `coupler_yolov8_pose_best.onnx`，已加入 Python ONNX Runtime 推理入口；缺 OpenCV/onnxruntime 时降级发布 `unknown`。
- `trainros_fusion`：使用 `message_filters::ApproximateTime` 同步 IMU/Laser，GPS 作为低频校正源，维护 Coupler 最近状态，使用三维 Kalman Filter 估计 position/speed/acceleration，IMU 更新加速度、GPS 校正速度、Laser 校正距离/位置，发布 `/train_state` 和延时 diagnostics。
- `trainros_stability_evaluator`：RMS、峰值和阈值评估，发布 `/stability`。
- `trainros_recorder`：`Record.action` server，启动/取消真实 `ros2 bag record` 子进程，默认不录制 `/camera/image_raw`。
- `trainros_logger`：订阅 `/diagnostics` 和 `/stability`，写业务 JSONL 并发布 `/log_status`。
- `trainros_monitor`：Linux `/proc` CPU/内存监控、Topic 端到端延时统计、`/system_status` 和 `/diagnostics`。
- `trainros_bringup`：launch、参数、RViz、systemd 和部署配置。

## 参数和启动

- 参数已拆分为 `sensors.yaml`、`sensors_fake_serial.yaml`、`fusion.yaml`、`logging.yaml`、`monitor.yaml`。
- `trainros_system.launch.py`：硬件模式，加载真实串口默认参数。
- `trainros_fake_serial.launch.py`：伪串口模式，可通过 `imu_port`、`gps_port`、`laser_port` 覆盖端口。
- Topic 名和节点名保持不变。

## 验证状态

- ROS2 Humble 和 colcon 已安装在 `Ubuntu-22.04` WSL。
- Windows `E:` 盘 drvfs 可能限制 `--symlink-install`，正式验证使用 `/tmp/trainros_ws`。
- `colcon build --symlink-install`：12 个包全部通过。
- IMU/GPS/Laser parser gtest：全部通过。
- fake serial 集成测试：自动创建 PTY，启动三路 driver、Fusion 和 Monitor，验证 `/imu/data`、`/gps/fix`、`/laser/scan`、`/train_state`、Fusion 三维 Kalman diagnostics 和 Monitor 延时 diagnostics。
- Logger 冒烟验证：模拟 diagnostics/stability 后，JSONL 记录串口打开、重连、丢帧和平稳性 alarm。
- Recorder 冒烟验证：Action 启动并取消 `ros2 bag record`，生成 `.db3` 和 `metadata.yaml`。
- `trainros_system.launch.py` 和 `trainros_fake_serial.launch.py` 已通过 5 秒启动冒烟验证。

## 旧工程映射

- `imu_project_patch_files/serialworker.cpp`：Qt 串口采集、重试、日志和状态行为。
- `imu_project_patch_files/protocol_processor.*`：迁移前需要审查的解析和协议 helper。
- `车钩识别模型代码_整理版/src/inference`：车钩图片和视频推理脚本。
- `车钩识别模型代码_整理版/src/features`：车钩状态几何特征提取。
- `python项目/PythonProject4/数据处理`：可能可用于平稳性离线分析，迁移前要确认是否是 demo 残留。

## 约束

- 未明确要求时，不修改旧工程。
- 一次只迁移一个清晰功能面。
- 默认不把模型、数据集、bag 和 generated runs 放进 git。
- 硬件测试前优先准备可回放 parser 测试和 fake serial 集成测试。
- 项目写入文档、README、memory 和占位说明默认使用中文。

## 下一步

1. 在真实 RK3588/Ubuntu 环境接入 `/dev/ttyUSB0`、`/dev/ttyS9`、`/dev/ttyS0` 做硬件联调并观察延时 diagnostics。
2. 接入 Camera/GStreamer 和 image_transport。
3. 接入 YOLO/ONNX Runtime/TensorRT。
4. 将当前三维 Kalman 扩展为完整 EKF/多传感器状态估计，并完善 PSD/TSI 和 RViz。
