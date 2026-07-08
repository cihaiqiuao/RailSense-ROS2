# 决策：Parser 测试和伪串口回放

日期：2026-07-06

## 决策

把 IMU/GPS/Laser 的解析逻辑从节点中抽成头文件，直接做 parser 单元测试，并提供伪串口回放工具用于无硬件端到端验证。

## 已实现

- IMU parser：`include/trainros_imu_driver/imu_parser.hpp`，测试有效 33 字节帧、错误帧头、错误校验。
- GPS parser：`include/trainros_gps_driver/gps_parser.hpp`，测试有效 RMC、no-fix RMC、非 RMC 语句。
- Laser parser：`include/trainros_laser_driver/laser_parser.hpp`，测试有效 12 字节帧、短帧、错帧头。
- 伪串口工具：`tools/fake_serial_replay.py`，创建 IMU/GPS/Laser 三个 PTY 并循环写样例数据。

## 验证

- 在 `Ubuntu-22.04` WSL 的 `/tmp/trainros_ws` 执行 `colcon build --symlink-install`，12 个包构建通过。
- 执行 `colcon test --packages-select trainros_imu_driver trainros_gps_driver trainros_laser_driver`，12 条测试 0 失败。
- 运行伪串口回放后，启动三个 driver，`ros2 topic echo --once` 成功获取 `/imu/data`、`/gps/fix`、`/laser/scan`。
