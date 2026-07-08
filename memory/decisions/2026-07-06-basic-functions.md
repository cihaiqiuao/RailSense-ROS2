# 决策：基础功能填充

日期：2026-07-06

## 决策

按“传感器优先”路线填充旧工程已有基础功能，不在本轮展开完整视觉推理和高级算法。

## 已实现

- IMU：Linux 串口 fd 读取、33 字节帧拆分、`0x55 0x51/52/53` 三段校验、加速度/角速度/姿态解算、发布 `/imu/data`。
- GPS：Linux 串口 fd 读取、RMC 行解析、经纬度转换、no-fix 状态发布、发布 `/gps/fix`。
- Laser：Linux 串口 fd 读取、`0xAA` 12 字节帧解析、距离米制转换、发布 `/laser/scan`。
- Fusion：订阅 IMU/GPS/Laser/Coupler，维护最新值，50 Hz 发布 `/train_state`。
- Stability：滑动窗口 RMS、峰值、阈值告警，发布 `/stability`。
- Logger：写 JSONL 心跳日志，发布 `/log_status`。
- Monitor：读取 `/proc/stat` 和 `/proc/meminfo`，发布 `/system_status` 和 diagnostics。

## 验证

- `Ubuntu-22.04` WSL 已安装 ROS2 Humble 和 colcon。
- 在 `/tmp/trainros_ws` 临时 ext4 工作区执行 `colcon build --symlink-install`，12 个包全部构建通过。
- 执行 8 秒 `ros2 launch trainros_bringup trainros_system.launch.py` 冒烟验证，10 个节点均启动；无串口设备时只输出打开失败和重连告警。

## 后续

- 补 parser 单元测试和伪串口回放测试。
- 做真实硬件串口联调。
- 再接 Camera/GStreamer、YOLO、完整卡尔曼滤波、PSD/TSI 和 rosbag2 进程控制。
