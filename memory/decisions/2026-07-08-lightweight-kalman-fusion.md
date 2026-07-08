# 2026-07-08 轻量 Kalman 融合增强

## 决策

- 不改 `TrainState.msg`，在现有字段内增强 `speed` 和 `acceleration` 输出。
- Fusion 内部新增轻量 Kalman Filter，状态量为 `speed` 和 `acceleration`。
- IMU 加速度用于高频预测和加速度观测更新。
- GPS 位置差分得到的速度用于低频校正。
- IMU/Laser 仍作为主同步链路，GPS 不参与强同步，Coupler 继续最近值缓存。

## 参数

- `enable_kalman_filter`: true
- `process_noise_speed`: 0.2
- `process_noise_acceleration`: 1.0
- `gps_speed_measurement_noise`: 0.5
- `imu_acceleration_measurement_noise`: 0.2

## diagnostics

- `kalman_enabled`
- `kalman_initialized`
- `kalman_speed`
- `kalman_acceleration`
- `raw_gps_speed`
- `kalman_imu_update_count`
- `kalman_gps_update_count`

## 验证

- WSL ext4 `/tmp/trainros_ws` 中 `colcon build --symlink-install` 通过，12 个包完成构建。
- `colcon test --packages-select trainros_imu_driver trainros_gps_driver trainros_laser_driver trainros_bringup --event-handlers console_direct+` 通过。
- 测试结果：14 tests，0 errors，0 failures，0 skipped。
