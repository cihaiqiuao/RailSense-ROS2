# 2026-07-06 集成测试、参数分层和 Driver Diagnostics

## 背景

基础 IMU/GPS/Laser driver 已经有 parser 单测和手动 fake serial 回放，但还缺少自动化端到端验证，也缺少统一的 driver 健康状态输出。单一 `trainros_default.yaml` 不利于后续区分硬件、仿真、融合、日志和监控参数。

## 决策

- 新增 `trainros_bringup/test/test_fake_serial_integration.py`，在测试内创建三路 PTY，启动 IMU/GPS/Laser driver，并订阅 topic 验证数据。
- 保留 parser gtest；集成测试只覆盖“伪串口 -> driver -> ROS topic -> diagnostics”的端到端链路。
- 删除单一 `trainros_default.yaml`，拆分为：
  - `sensors.yaml`
  - `sensors_fake_serial.yaml`
  - `fusion.yaml`
  - `logging.yaml`
  - `monitor.yaml`
- `trainros_system.launch.py` 默认加载硬件参数。
- 新增 `trainros_fake_serial.launch.py`，支持通过 `imu_port`、`gps_port`、`laser_port` 覆盖伪串口端口。
- IMU/GPS/Laser driver 统一发布 `diagnostic_msgs/msg/DiagnosticArray` 到 `/diagnostics`。
- diagnostics 统一字段：`serial_open`、`port`、`valid_frames`、`dropped_frames`、`reconnect_count`、`last_receive_time`、`last_error`。
- IMU 额外字段：`last_frame_valid`。
- GPS 额外字段：`has_fix`。
- Laser 额外字段：`latest_distance_m`。

## 验证

- 在 WSL `/tmp/trainros_ws` 执行 `colcon build --symlink-install`，12 个包构建通过。
- 执行 `colcon test --packages-select trainros_imu_driver trainros_gps_driver trainros_laser_driver trainros_bringup --event-handlers console_direct+`，parser gtest 和 fake serial 集成测试全部通过。
- `trainros_system.launch.py` 和 `trainros_fake_serial.launch.py` 均通过 5 秒启动冒烟验证。

## 后续

- 真实硬件接入后，重点观察 `/diagnostics` 中重连次数、丢弃帧数、最近接收时间和 GPS fix 状态。
- Camera/GStreamer 和 YOLO 接入前，先保持传感器主链路稳定。
