# TrainROS 工程目录说明

本文说明当前仓库目录边界。ROS2 包名、节点名、Topic 名保持不变；这里只调整源码在工作区里的分层。

## 根目录

```text
TrainROS/
  src/          # ROS2 colcon 源码工作区
  docs/         # 架构、命令、迁移和设计说明
  tools/        # 不参与 colcon 构建的辅助工具
  config/       # 根级运行配置，例如 rosbag2 topic 列表
  data/         # 本地样例数据、bag 和临时数据
  memory/       # Engramory 项目记忆
```

`build/`、`install/` 和 `log/` 是 colcon 构建产物，不属于源码结构。

## src 分层

```text
src/
  interfaces/
    trainros_interfaces/
  drivers/
    trainros_imu_driver/
    trainros_gps_driver/
    trainros_laser_driver/
    trainros_camera_driver/
  perception/
    trainros_yolo_detection/
  estimation/
    trainros_fusion/
    trainros_stability_evaluator/
  runtime/
    trainros_recorder/
    trainros_logger/
    trainros_monitor/
  bringup/
    trainros_bringup/
```

- `interfaces`：只放消息、服务和 Action 定义，不依赖业务节点。
- `drivers`：只负责外设或数据源接入，把硬件协议转换成 ROS2 Topic。
- `perception`：放视觉、AI 推理和目标/状态识别。
- `estimation`：放多传感器融合、状态估计和平稳性评估。
- `runtime`：放录制、日志、监控等运行支撑节点。
- `bringup`：放系统级 launch、参数、RViz 和部署配置。

## 工具和数据

- `tools/replay/`：伪串口、离线回放、调试辅助脚本。
- `config/rosbag2/`：rosbag2 topic 列表等根级配置。
- `data/bags/`：本地录制 bag，默认不进入 git。
- `data/samples/`：本地样例数据，默认不进入 git。

## 约束

- 新 ROS2 包必须放在 `src/` 下对应分层目录中。
- 不把 `build/`、`install/`、`log/`、bag、运行输出和大模型产物作为源码提交。
- 不为了整理目录修改包名、节点名、Topic 名或消息类型。
- 新增参数优先放在 `src/bringup/trainros_bringup/params/`，除非该参数只服务于单包独立测试。
