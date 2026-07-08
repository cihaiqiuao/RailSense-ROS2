# trainros_laser_driver

激光测距数据 ROS2 driver 包。

计划输出：

- Topic：`/laser/scan`
- 类型：`sensor_msgs/msg/LaserScan`
- QoS：Best Effort

迁移来源：

- `serialworker.cpp` 中的 Laser 串口处理逻辑。

初始范围：

- 抽取激光帧 parser。
- 发布 scan/range 兼容数据。
- 添加距离异常检测。
