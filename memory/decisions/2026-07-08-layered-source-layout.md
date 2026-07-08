# 2026-07-08 分层源码目录重构

## 背景

原 `src/` 下 12 个 ROS2 包平铺，随着 driver、perception、fusion、runtime 和 bringup 增多，目录边界不够清晰。

## 决策

- 不修改 ROS2 包名、节点名、Topic 名、Action 类型和消息类型。
- 将 `src/` 按职责分层：
  - `interfaces/`：接口定义包。
  - `drivers/`：IMU、GPS、Laser、Camera 驱动包。
  - `perception/`：视觉和 AI 感知包。
  - `estimation/`：融合和平稳性评估包。
  - `runtime/`：录制、日志和监控包。
  - `bringup/`：launch、参数、RViz 和部署配置包。
- 将伪串口回放工具从 `tools/fake_serial_replay.py` 移到 `tools/replay/fake_serial_replay.py`。
- 新增 `docs/PROJECT_STRUCTURE.md` 作为后续新增文件的目录归属规则。

## 验证

- 清理 WSL `/tmp/trainros_ws/build`、`install`、`log`，避免 colcon 缓存旧平铺路径。
- `colcon list` 能发现 12 个包，路径均为新分层目录。
- `colcon build --symlink-install` 通过，12 个包全部构建完成。
- `colcon test --packages-select trainros_imu_driver trainros_gps_driver trainros_laser_driver trainros_stability_evaluator trainros_bringup --event-handlers console_direct+` 通过。
- `colcon test-result --verbose` 结果为 `21 tests, 0 errors, 0 failures, 0 skipped`。
