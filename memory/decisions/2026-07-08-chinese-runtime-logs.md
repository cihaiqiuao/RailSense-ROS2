# 2026-07-08 运行日志中文化

## 背景

用户要求运行时打印日志尽量使用中文，Topic、节点名、参数名、event 名等代码标识可以保留英文。

## 决策

- 将主要 C++ 节点的 `RCLCPP_INFO/WARN` 文本改为中文。
- 将 `/diagnostics` 中面向人的 `status.message` 改为中文。
- 保留 `event`、`level`、Topic、节点名、参数名、`KeyValue.key` 等机器可解析字段不变。
- 保留 `IMU`、`GPS`、`Laser`、`Fusion`、`Monitor`、`Logger`、`YOLO`、`rosbag2`、`serial` 等工程名词。

## 验证

- WSL `/tmp/trainros_ws` 中构建相关 9 个包通过。
- parser、Stability 和 fake serial 集成测试通过。
- `colcon test-result --verbose` 结果为 `21 tests, 0 errors, 0 failures, 0 skipped`。
