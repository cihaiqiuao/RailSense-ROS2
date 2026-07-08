# trainros_recorder

录制控制 ROS2 包。

计划接口：

- `trainros_interfaces/action/Record`

初始范围：

- 启动和取消 rosbag2 录制。
- 通过 Action feedback 返回当前录制时长。
- rosbag 配置放在 `config/rosbag2`。
