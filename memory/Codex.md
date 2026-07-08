# TrainROS 记忆主索引

本文件是 Engramory 风格三层记忆的第一层，只保留主索引，控制在 200 行内。

## 规则

- 先加载本文件，再按任务需要打开详情文件。
- 不记录密钥、密码、令牌、私钥或大数据集。
- 上下文用到一半左右时，把当前进展摘要写入 `memory/handoffs/`。
- 任务收尾时，把关键项目事实同步到本目录；用户明确要求时，再同步到 Codex 长期记忆。
- 过期或重复笔记定期移动到 `memory/archive/`。
- 本项目写入 README、docs、memory 和占位说明时默认使用中文。

## 当前项目

- 项目：TrainROS，基于 ROS2 Humble 的多源感知与列车运行状态评估平台。
- 工作区：`E:\ros2\TrainROS`
- 旧工程来源：`E:\毕业设计\03_代码工程`
- 详情文件：`memory/projects/TrainROS.md`

## 详情索引

- `memory/projects/TrainROS.md`：当前架构、包职责、实现状态、验证状态。
- `memory/decisions/2026-07-06-initial-framework.md`：初始目录和记忆架构决策。
- `memory/decisions/2026-07-06-code-framework.md`：第一版 ROS2 代码框架决策。
- `memory/decisions/2026-07-06-integration-diagnostics.md`：集成测试、参数分层和 driver diagnostics 决策。
- `docs/ARCHITECTURE.md`：Topic、QoS、节点职责和系统结构。
- `docs/MIGRATION_PLAN.md`：旧工程迁移计划。
- `docs/ROADMAP.md`：后续版本路线图。
- `docs/ROS2_COMMANDS_AND_USAGE.md`：常用 ROS2 命令、无硬件使用流程和录制功能说明。
- `docs/MODEL_CONVERSION.md`：车钩 YOLOv8-Pose 模型转 ONNX 记录。

## 按需加载提示

打开 `memory/projects/TrainROS.md` 的情况：

- 添加或解释 ROS2 包、节点、接口。
- 迁移旧 Qt/Python/车钩识别代码。
- 回答当前项目有哪些功能、哪些还没做。
- 准备简历、毕业设计或面试表述。

打开 `docs/MIGRATION_PLAN.md` 的情况：

- 决定下一步迁移哪个旧文件。
- 检查旧工程路径、迁移边界和风险。

打开 `docs/ARCHITECTURE.md` 的情况：

- 检查 Topic 图、QoS、包边界或节点职责。

## 当前状态摘要

- 已完成目录骨架、项目记忆、接口定义、12 个 ROS2 包和基础节点。
- IMU/GPS/Laser 已接入 Linux 串口 fd 读取、旧协议基础解析、重连和统计。
- IMU/GPS/Laser 已统一发布 `/diagnostics`。
- Fusion 已使用 `message_filters::ApproximateTime` 同步 IMU/GPS/Laser；Stability/Logger/Monitor 已从占位改为基础可用逻辑。
- Logger 已订阅 `/diagnostics` 和 `/stability`，业务 JSONL 记录串口、丢帧、GPS no-fix、Fusion/Monitor 诊断和平稳性告警事件。
- Recorder 已用 `Record.action` 启动和取消真实 `ros2 bag record` 子进程。
- 已将旧工程 `coupler_yolov8_pose_best.pt` 转为 `trainros_yolo_detection/models/coupler_yolov8_pose_best.onnx`。
- `trainros_yolo_detection` 已加入 Python ONNX Runtime 推理入口，缺依赖时降级发布 `unknown`。
- `trainros_camera_driver` 已加入 `video_file_publisher`，可用本地 MP4 发布 `/camera/image_raw`；需要 WSL 安装 `ffmpeg`。
- 参数已拆分为 `sensors.yaml`、`sensors_fake_serial.yaml`、`fusion.yaml`、`logging.yaml`、`monitor.yaml`。
- 已新增 fake serial 集成测试，覆盖伪串口到 ROS topic、Fusion `/train_state` 和 diagnostics 的端到端链路。
- Fusion 已优化为 IMU/Laser 高频同步、GPS 低频校正、Coupler 最近值缓存；Monitor 已增加 Topic 延时 diagnostics。
- Fusion 已加入轻量 Kalman Filter，状态量为 speed/acceleration，IMU 高频更新、GPS 低频校正。
- Recorder 默认不录制 `/camera/image_raw`，可用 `record_camera:=true` 开启原始图像录制。
- ROS2 Humble/colcon 已在 `Ubuntu-22.04` WSL 中可用；验证工作区使用 `/tmp/trainros_ws`。
- 12 个包构建通过；parser gtest 和 fake serial 集成测试通过。
- 下一步建议：真实硬件串口联调，然后再接 Camera/GStreamer、YOLO/ONNX、完整融合算法和 RViz。
