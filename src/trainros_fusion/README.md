# trainros_fusion

多传感器融合核心包。

计划输入：

- `/imu/data`
- `/gps/fix`
- `/laser/scan`
- `/coupler_detection`

计划输出：

- `/train_state`
- 类型：`trainros_interfaces/msg/TrainState`
- 频率：50 Hz
- QoS：Reliable

初始范围：

- 添加 ApproximateTime 同步。
- 统一时间戳和 GPS 时间。
- 将各传感器原始频率插值到 50 Hz。
- 同步输出可测后再添加卡尔曼滤波。
