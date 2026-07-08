# 决策：第一版 ROS2 代码大框架

日期：2026-07-06

## 决策

先补齐 ROS2 包级代码框架，不在本阶段实现复杂业务逻辑。

## 范围

已添加：

- `trainros_interfaces` 的 4 个消息和 1 个 Action；
- 每个功能包的 `package.xml` 和 `CMakeLists.txt`；
- IMU、GPS、Laser、Camera、YOLO、Fusion、Stability、Recorder、Logger、Monitor 的最小 C++ 节点；
- `trainros_bringup` 的系统级 launch 和默认参数文件；
- rosbag2 Topic 录制清单占位。

## 取舍

- 节点先发布或转发占位数据，保留 Topic、QoS、参数和入口结构。
- 不在本阶段迁移 Qt 串口解析、GStreamer pipeline、ONNX Runtime、卡尔曼滤波、PSD/TSI、JSONL/WAL 细节。
- 这样后续迁移时主要替换节点内部逻辑，工程边界不用频繁改。

## 验证

已完成静态结构检查：

- 12 个包均存在 `package.xml` 和 `CMakeLists.txt`；
- 10 个功能包均存在 C++ 节点入口。

未完成真实构建：

- 当前 Windows/WSL 环境没有可用的 ROS2 Humble/colcon。
- 后续需要在 ROS2 Humble 环境执行 `colcon build --symlink-install`。
