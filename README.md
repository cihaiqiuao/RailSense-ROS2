# RailSense-ROS2

RailSense-ROS2（工程内部包名 TrainROS）是一个基于 ROS2 Humble 的多源感知与列车运行状态评估平台。项目从旧工程 `E:\毕业设计\03_代码工程` 逐步迁移，但不修改旧工程目录。

## 工程结构

```text
TrainROS/
  src/
    interfaces/                   # 自定义消息、Action 和接口包
    drivers/                      # IMU/GPS/Laser/Camera 驱动包
    perception/                   # YOLO 等视觉感知包
    estimation/                   # Fusion 和 Stability 状态估计包
    runtime/                      # Recorder/Logger/Monitor 运行支撑包
    bringup/                      # launch、参数、RViz、部署配置
  docs/                           # 架构、迁移计划、路线图
  tools/replay/                   # 伪串口回放和辅助工具
  config/                         # rosbag2、RViz 等根级运行配置
  data/                           # 本地 bag、样例数据和临时数据
  memory/                         # Engramory 风格项目记忆
```

## 当前功能

- 已建立 12 个 ROS2 包和基础节点入口。
- `trainros_interfaces` 已定义 `Coupler.msg`、`TrainState.msg`、`Stability.msg`、`SystemStatus.msg`、`Record.action`。
- IMU driver 已迁移 33 字节帧解析、`0x55 0x51/52/53` 校验、加速度/角速度/姿态换算和串口重连。
- GPS driver 已迁移 `$GNRMC/$GPRMC` 解析、经纬度换算、fix/no-fix 发布和串口重连。
- Laser driver 已迁移 `0xAA` 12 字节距离帧解析和米制距离发布。
- IMU/GPS/Laser driver 统一发布 `/diagnostics`，包含串口状态、端口、有效帧数、丢弃帧数、重连次数、最近接收时间和最近错误。
- Fusion 节点使用 `message_filters::ApproximateTime` 同步 IMU/Laser，GPS 作为低频速度校正源，车钩检测作为最近状态缓存，并用三维 Kalman Filter 融合 position/speed/acceleration 后以 50 Hz 发布 `/train_state`；其中 IMU 更新加速度，GPS 更新速度，Laser 更新距离/位置。
- Stability 节点实现滑动窗口 RMS、峰值加速度、简化 PSD、TSI 评分和 warning/alarm 阈值，发布 `/stability` 与 `/diagnostics`。
- Logger 订阅 `/diagnostics` 和 `/stability`，把串口开闭、重连、parser 丢帧、GPS no-fix、Fusion/Monitor 诊断变化和平稳性 warning/alarm 写入业务 JSONL，并发布 `/log_status`。
- Recorder 通过 `Record.action` 真正启动 `ros2 bag record` 子进程，支持 Action 取消时停止 rosbag2 并落盘。

## 参数和启动

参数已按职责拆分：

- `src/bringup/trainros_bringup/params/sensors.yaml`：真实硬件传感器、Camera、YOLO 参数。
- `src/bringup/trainros_bringup/params/sensors_fake_serial.yaml`：伪串口传感器参数。
- `src/bringup/trainros_bringup/params/fusion.yaml`：融合和平稳性参数，包含同步窗口、传感器超时和 Kalman 噪声参数。
- `src/bringup/trainros_bringup/params/logging.yaml`：录制 topic 列表、bag 输出路径和日志参数。
- `src/bringup/trainros_bringup/params/monitor.yaml`：系统监控参数。

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
- fake serial 集成测试：自动创建三路 PTY，启动三个 driver、Fusion、Stability 和 Monitor，验证 `/imu/data`、`/gps/fix`、`/laser/scan`、`/train_state`、`/stability`、三维 Kalman diagnostics、平稳性 diagnostics 和延时 diagnostics 均有有效输出。
- Logger 冒烟验证：发布模拟 diagnostics/stability 后，JSONL 中出现 `serial_opened`、`serial_reconnect`、`parser_drop_frame`、`stability_alarm`。
- Recorder 冒烟验证：Action 启动 `ros2 bag record`，取消后生成 `metadata.yaml` 和 `.db3` bag 文件。
- `trainros_system.launch.py` 和 `trainros_fake_serial.launch.py` 均通过 5 秒启动冒烟验证。

## 后续任务

- 接真实 RK3588/Ubuntu 串口硬件联调。
- 接入 Camera/GStreamer 和 image_transport。
- 接入 YOLO/ONNX Runtime/TensorRT。
- 将当前三维 Kalman 扩展为更完整的 EKF/多传感器状态估计，并完善 PSD/TSI 和 RViz 可视化。
