# 决策：TrainROS 初始框架

日期：2026-07-06

## 决策

在 `E:\ros2\TrainROS` 创建新的 ROS2 Humble 工作区，不直接在旧工程里改。

## 原因

- 旧工程目录包含多个历史项目，不是单一 ROS2 workspace。
- 干净的 ROS2 包结构能让 Topic 契约、构建边界和迁移步骤更清楚。
- 保持旧文件不变，可以降低迁移 parser、推理代码和平稳性逻辑时的风险。

## 初始范围

已创建：

- 目录骨架；
- 各包 README；
- 架构、迁移、路线图文档；
- 项目记忆结构。

尚未创建：

- ROS2 `package.xml` / `CMakeLists.txt`；
- 自定义消息和 Action 定义；
- 可运行节点；
- launch 文件。

## 后续验证目标

下一个实现里程碑应让 `trainros_interfaces` 可以通过以下命令构建：

```bash
colcon build --packages-select trainros_interfaces
```
