# 2026-07-07 本地视频发布到 camera image topic

## 背景

用户已有本地视频 `D:\chegou\视频\WeChat_20250616101426.mp4`，需要先验证视觉链路输入，不依赖真实摄像头。

## 决策

- 在 `trainros_camera_driver` 新增 Python 节点 `video_file_publisher`。
- 该节点用 `ffprobe` 获取视频宽、高、帧率，用 `ffmpeg` 管道输出 `bgr24` raw frame。
- 节点发布 `/camera/image_raw`，消息类型 `sensor_msgs/msg/Image`，编码 `bgr8`。
- 新增 `trainros_bringup/launch/trainros_video_yolo.launch.py`，一键启动视频发布和 YOLO 检测节点。
- 不依赖 OpenCV；但 WSL 需要安装 `ffmpeg`。

## 验证

- Python 语法检查通过。
- 在 WSL `/tmp/trainros_ws` 构建 `trainros_interfaces`、`trainros_camera_driver`、`trainros_yolo_detection`、`trainros_bringup` 通过。
- 当前 WSL 未安装 `ffmpeg/ffprobe`，节点启动时能明确报错提示安装，不影响构建。
