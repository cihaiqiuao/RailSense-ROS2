# TrainROS 架构说明

## 运行时 Topic 图

```text
IMU      GPS      Laser      Camera
 |        |        |          |
 v        v        v          v
/imu/data /gps/fix /laser/scan /camera/image_raw
 |        |        |          |
 +--------+--------+          v
          |             trainros_yolo_detection
          |                    |
          |             /coupler_detection
          v                    |
        trainros_fusion <------+
          |
          v
      /train_state
          |
          v
trainros_stability_evaluator
          |
          v
      /stability
```

旁路节点：

- `trainros_recorder`：通过 `Record.action` 控制 rosbag2 录制。
- `trainros_logger`：异步写业务日志。
- `trainros_monitor`：发布系统状态和诊断信息。
- `trainros_bringup`：统一管理 launch、参数、RViz 配置和部署预设。

## 包职责

| 包名 | 职责 | 主要 Topic |
| --- | --- | --- |
| `trainros_interfaces` | 自定义消息、服务和 Action | `Coupler`、`TrainState`、`Stability`、`SystemStatus`、`Record` |
| `trainros_imu_driver` | IMU 串口解析、CRC、重连、统计 | `/imu/data` |
| `trainros_gps_driver` | NMEA 解析、UTM/ENU 转换 | `/gps/fix` |
| `trainros_laser_driver` | 激光串口解析、距离异常检测 | `/laser/scan` |
| `trainros_camera_driver` | GStreamer/image_transport 图像发布 | `/camera/image_raw` |
| `trainros_yolo_detection` | OpenCV 预处理、ONNX Runtime 推理、NMS、姿态状态识别 | `/coupler_detection` |
| `trainros_fusion` | IMU/Laser ApproximateTime 同步、GPS 低频速度校正、Laser 距离观测、三维 Kalman 融合 | `/train_state` |
| `trainros_stability_evaluator` | RMS、峰值、PSD、TSI 与阈值评估 | `/stability` |
| `trainros_recorder` | 录制 Action 与 rosbag2 控制 | `Record.action` |
| `trainros_logger` | 异步日志、JSONL、轮转、WAL 风格恢复 | `/log_status` |
| `trainros_monitor` | CPU、内存、I/O、温度、FPS、队列、DDS 延迟 | `/system_status`、diagnostics |
| `trainros_bringup` | 系统 launch、参数、RViz 和部署配置 | launch 文件 |

## QoS 设计

| Topic | QoS |
| --- | --- |
| `/imu/data` | Best Effort，Keep Last 5 |
| `/camera/image_raw` | Best Effort |
| `/laser/scan` | Best Effort |
| `/gps/fix` | Reliable |
| `/coupler_detection` | Reliable |
| `/train_state` | Reliable |
| `/stability` | Reliable |
| `/system_status` | Reliable |

高频传感器流优先保证实时性和新鲜度；融合结果、检测结果和系统状态属于关键业务数据，优先保证可靠送达。
