# 2026-07-08 三维 Kalman 与激光测距融合

## 背景

原 Fusion 轻量 Kalman 只估计 `speed` 和 `acceleration`，Laser 只作为最近距离值写入 `TrainState.laser_distance`，没有真正参与状态估计。

## 决策

- 将 Kalman 状态从二维 `[speed, acceleration]` 扩展为三维 `[position, speed, acceleration]`。
- IMU 加速度作为 `acceleration` 观测。
- GPS 经纬度差分得到的速度作为 `speed` 观测。
- Laser 测距作为 `position` 观测，用于校正距离状态。
- 不修改 `TrainState.msg`，保持接口兼容。
- `/train_state.laser_distance` 输出滤波后的距离/位置状态。
- 原始 Laser 测距仍通过 `/laser/scan` 和 `/diagnostics` 中的 `raw_laser_distance` 查看。

## 参数

参数位于 `src/trainros_bringup/params/fusion.yaml`：

- `process_noise_position`
- `process_noise_speed`
- `process_noise_acceleration`
- `laser_distance_measurement_noise`
- `gps_speed_measurement_noise`
- `imu_acceleration_measurement_noise`

## 验证

- WSL `/tmp/trainros_ws` 中重新构建 `trainros_fusion` 和 `trainros_bringup`。
- IMU/GPS/Laser parser gtest 通过。
- fake serial 集成测试通过，验证 `/train_state` 输出、Fusion diagnostics、Monitor 延时 diagnostics。
- 测试继续要求 `/laser/scan` 原始距离接近 `1.234`，并要求 `raw_laser_distance` 诊断值接近 `1.234`。
