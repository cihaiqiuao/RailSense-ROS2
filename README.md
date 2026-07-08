# RailSense-ROS2

RailSense-ROS2（工程内部包名 TrainROS）是一个基于 ROS2 Humble 的多源感知与列车运行状态评估平台。项目从旧工程 `E:\毕业设计\03_代码工程` 逐步迁移，但不修改旧工程目录。

## 工程结构

```text
TrainROS/
  src/
    trainros_interfaces/          # 自定义消息和 Record.action
    trainros_imu_driver/          # IMU 串口解析，发布 /imu/data
    trainros_gps_driver/          # GPS RMC 解析，发布 /gps/fix
    trainros_laser_driver/        # 激光测距解析，发布 /laser/scan
    trainros_camera_driver/       # Camera/GStreamer 占位框架
    trainros_yolo_detection/      # YOLO 检测占位框架
    trainros_fusion/              # ApproximateTime 同步融合，发布 /train_state
    trainros_stability_evaluator/ # RMS/峰值/阈值评估，发布 /stability
    trainros_recorder/            # Record.action 框架
    trainros_logger/              # JSONL 日志状态
    trainros_monitor/             # CPU/内存监控与 diagnostics
    trainros_bringup/             # launch、参数、RViz、部署配置
  docs/                           # 架构、迁移计划、路线图
  tools/                          # 伪串口回放和辅助工具
  memory/                         # Engramory 风格项目记忆
```

## 当前功能

- 已建立 12 个 ROS2 包和基础节点入口。
- `trainros_interfaces` 已定义 `Coupler.msg`、`TrainState.msg`、`Stability.msg`、`SystemStatus.msg`、`Record.action`。
- IMU driver 已迁移 33 字节帧解析、`0x55 0x51/52/53` 校验、加速度/角速度/姿态换算和串口重连。
- GPS driver 已迁移 `$GNRMC/$GPRMC` 解析、经纬度换算、fix/no-fix 发布和串口重连。
- Laser driver 已迁移 `0xAA` 12 字节距离帧解析和米制距离发布。
- IMU/GPS/Laser driver 统一发布 `/diagnostics`，包含串口状态、端口、有效帧数、丢弃帧数、重连次数、最近接收时间和最近错误。
- Fusion 节点使用 `message_filters::ApproximateTime` 同步 IMU/GPS/Laser，维护车钩最近状态，并以 50 Hz 发布基础 `/train_state`。
- Stability 节点实现滑动窗口 RMS、峰值加速度、warning/alarm 阈值，发布 `/stability`。
- Logger 订阅 `/diagnostics` 和 `/stability`，把串口开闭、重连、parser 丢帧、GPS no-fix、Fusion/Monitor 诊断变化和平稳性 warning/alarm 写入业务 JSONL，并发布 `/log_status`。
- Recorder 通过 `Record.action` 真正启动 `ros2 bag record` 子进程，支持 Action 取消时停止 rosbag2 并落盘。

## 参数和启动

参数已按职责拆分：

- `src/trainros_bringup/params/sensors.yaml`：真实硬件传感器、Camera、YOLO 参数。
- `src/trainros_bringup/params/sensors_fake_serial.yaml`：伪串口传感器参数。
- `src/trainros_bringup/params/fusion.yaml`：融合和平稳性参数，包含 `sync_queue_size` 和 `sync_slop_ms`。
- `src/trainros_bringup/params/logging.yaml`：录制 topic 列表、bag 输出路径和日志参数。
- `src/trainros_bringup/params/monitor.yaml`：系统监控参数。

启动文件：

```bash
ros2 launch trainros_bringup trainros_system.launch.py
ros2 launch trainros_bringup trainros_fake_serial.launch.py imu_port:=/dev/pts/1 gps_port:=/dev/pts/2 laser_port:=/dev/pts/3
```

## 构建和测试

常用 ROS2 命令、无硬件运行流程和录制功能说明见：

- `docs/ROS2_COMMANDS_AND_USAGE.md`

Windows `E:` 盘通过 drvfs 挂载时，`--symlink-install` 可能受符号链接权限影响。当前验证使用 WSL ext4 临时工作区 `/tmp/trainros_ws`。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test --packages-select trainros_imu_driver trainros_gps_driver trainros_laser_driver trainros_bringup --event-handlers console_direct+
```

当前验证结果：

- `colcon build --symlink-install`：12 个包全部通过。
- IMU/GPS/Laser parser gtest：全部通过。
- fake serial 集成测试：自动创建三路 PTY，启动三个 driver 和 Fusion，验证 `/imu/data`、`/gps/fix`、`/laser/scan`、`/train_state` 和 `/diagnostics` 均有有效输出。
- Logger 冒烟验证：发布模拟 diagnostics/stability 后，JSONL 中出现 `serial_opened`、`serial_reconnect`、`parser_drop_frame`、`stability_alarm`。
- Recorder 冒烟验证：Action 启动 `ros2 bag record`，取消后生成 `metadata.yaml` 和 `.db3` bag 文件。
- `trainros_system.launch.py` 和 `trainros_fake_serial.launch.py` 均通过 5 秒启动冒烟验证。

## 后续任务

- 接真实 RK3588/Ubuntu 串口硬件联调。
- 接入 Camera/GStreamer 和 image_transport。
- 接入 YOLO/ONNX Runtime/TensorRT。
- 完整实现卡尔曼滤波、PSD/TSI 和 RViz 可视化。
