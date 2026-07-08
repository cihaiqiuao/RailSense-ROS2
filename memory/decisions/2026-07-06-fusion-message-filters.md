# 2026-07-06 Fusion 接入 message_filters

## 背景

Fusion 是 TrainROS 的核心节点。之前实现是分别订阅 IMU/GPS/Laser/Coupler，并缓存最近值，虽然能发布 `/train_state`，但还没有体现 ROS2 多源时间同步能力。

## 决策

- 使用 `message_filters::Subscriber` 订阅 `/imu/data`、`/gps/fix`、`/laser/scan`。
- 使用 `message_filters::Synchronizer<ApproximateTime>` 做三路传感器近似时间同步。
- Coupler 检测结果仍用普通订阅并维护最近状态，因为视觉检测频率和可用性可能与三路基础传感器不同。
- 保留 50 Hz 定时发布 `/train_state`，同步回调只更新最新融合状态。
- 新增参数：`sync_queue_size`、`sync_slop_ms`。

## 验证

- 在 WSL `/tmp/trainros_ws` 执行 `colcon build --symlink-install`，12 个包构建通过。
- 执行 `colcon test --packages-select trainros_bringup trainros_fusion --event-handlers console_direct+`，fake serial 集成测试通过。
- 集成测试现在会启动三路 driver 和 Fusion，验证 `/train_state` 中 IMU 加速度非零、Laser 距离接近 `1.234`。
