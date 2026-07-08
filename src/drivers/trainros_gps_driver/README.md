# trainros_gps_driver

GPS/NMEA 数据 ROS2 driver 包。

计划输出：

- Topic：`/gps/fix`
- 类型：`sensor_msgs/msg/NavSatFix`
- QoS：Reliable

迁移来源：

- `serialworker.cpp` 中的 GPS 串口处理逻辑。

初始范围：

- 解析 NMEA。
- 发布 `NavSatFix`。
- 原始 GPS 发布稳定后，再添加 UTM/ENU 转换工具。
