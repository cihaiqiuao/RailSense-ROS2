# 迁移计划

## 源工程

| 源路径 | 迁移目标 | 说明 |
| --- | --- | --- |
| `legacy_project/imu_project_patch_files/serialworker.cpp` | `trainros_imu_driver`、`trainros_gps_driver`、`trainros_laser_driver`、`trainros_logger`、`trainros_monitor` | 包含 Qt 串口、重试计数、`/userdata` 日志、状态上报和传感器缓冲逻辑。迁移时只提取解析与可靠性行为，不照搬 Qt 事件循环架构。 |
| `legacy_project/imu_project_patch_files/protocol_processor.*` | 传感器 driver 解析辅助代码 | 迁移前先审查帧解析和 CRC，再抽成普通 C++ helper。 |
| `legacy_project/coupler_detection/src/inference` | `trainros_yolo_detection` | 将离线图片/视频推理改造成 ROS2 图像回调流水线。 |
| `legacy_project/coupler_detection/models` | `src/perception/trainros_yolo_detection/models` | 默认不把大模型文件纳入 git，除非后续明确需要。 |
| `legacy_project/data_processing` | `trainros_stability_evaluator` 和离线工具 | 迁移前先确认脚本价值，其中有些文件可能只是 PyCharm/demo 残留。 |

## 增量里程碑

1. 接口包
   - 添加 `Coupler.msg`、`TrainState.msg`、`Stability.msg`、`SystemStatus.msg`、`Record.action`。
   - 验证命令：`colcon build --packages-select trainros_interfaces`。

2. 传感器 driver
   - 按 IMU、GPS、Laser 一个传感器一个传感器迁移串口帧解析。
   - 先用合成串口/文件回放测试，再做硬件测试。

3. Camera 与检测
   - 先添加 GStreamer 图像发布。
   - 图像 Topic 稳定后再接 ONNX Runtime 推理。
   - 用录制视频或 rosbag2 回放验证。

4. 融合与平稳性评估
   - 先做 ApproximateTime 同步和 50 Hz 输出。
   - Topic 时序可观测后再加入插值和卡尔曼滤波。
   - 验证 `/train_state` 时间戳和发布频率。

5. Bringup、日志和监控
   - 添加 launch、RViz、rosbag2 预设和部署文档。
   - 验证完整图启动和 Topic 健康状态。

## 约束

- 迁移过程中保持旧工程文件不变。
- 不急着复制大数据集、runs 或模型权重。
- 优先形成小而可构建的 ROS2 包，不做一次性大搬迁。
- 每个迁移出来的 parser 都要有可回放测试输入。
- 本项目写入的文档、README、memory 和占位说明默认使用中文。
