# trainros_imu_driver

IMU 串口数据 ROS2 driver 包。

计划输出：

- Topic：`/imu/data`
- 类型：`sensor_msgs/msg/Imu`
- 频率：200 Hz
- QoS：Best Effort，Keep Last 5

迁移来源：

- `legacy_project/imu_project_patch_files/serialworker.cpp`
- `legacy_project/imu_project_patch_files/protocol_processor.*`

初始范围：

- 抽取帧解析和 CRC 校验。
- 添加重连和统计计数。
- 硬件验证前先添加可回放 parser 测试。
