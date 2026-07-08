# 2026-07-08 低延时与可观测性优化

## 决策

- Fusion 主链路改为 IMU/Laser `ApproximateTime` 同步。
- GPS 改为普通订阅缓存，作为低频速度校正源，不再阻塞每次融合更新。
- Coupler 检测继续使用最近值缓存，避免视觉推理延时影响 `/train_state`。
- Monitor 订阅 `/imu/data`、`/gps/fix`、`/laser/scan`、`/train_state`，通过 `/diagnostics` 输出端到端延时。
- IMU/GPS/Laser/Fusion/Monitor/Logger 使用 `MultiThreadedExecutor` 和 callback group，关键共享状态加互斥保护。
- Recorder 默认不录制 `/camera/image_raw`，通过 `record_camera:=true` 或手动加入 `record_topics` 开启。

## 参数

- `sync_slop_ms`: 50
- `gps_timeout_ms`: 1000
- `coupler_timeout_ms`: 1000
- `sensor_timeout_ms`: 500
- `topic_timeout_ms`: 1000
- `record_camera`: false

## 验证

- WSL ext4 `/tmp/trainros_ws` 中 `colcon build --symlink-install` 通过，12 个包完成构建。
- `colcon test --packages-select trainros_imu_driver trainros_gps_driver trainros_laser_driver trainros_bringup --event-handlers console_direct+` 通过。
- 测试结果：14 tests，0 errors，0 failures，0 skipped。
