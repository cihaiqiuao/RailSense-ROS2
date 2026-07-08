# trainros_camera_driver

ROS2 图像发布包。

## 输出 Topic

- Topic：`/camera/image_raw`
- 类型：`sensor_msgs/msg/Image`
- 编码：`bgr8`
- QoS：Best Effort

## 当前功能

- `camera_driver_node`：占位 camera publisher，后续接入 MIPI Camera / GStreamer。
- `video_file_publisher`：本地视频文件发布工具，将 MP4 解码后发布为 `/camera/image_raw`。

## 本地视频发布

无真实摄像头时，可以用本地 MP4 发布 `/camera/image_raw`：

```bash
ros2 run trainros_camera_driver video_file_publisher --ros-args \
  -p video_path:=/mnt/d/chegou/视频/WeChat_20250616101426.mp4
```

依赖：

```bash
sudo apt-get update
sudo apt-get install -y ffmpeg
```

也可以启动视频发布和 YOLO 检测：

```bash
ros2 launch trainros_bringup trainros_video_yolo.launch.py \
  video_path:=/mnt/d/chegou/视频/WeChat_20250616101426.mp4
```

## 后续

- 接入 MIPI Camera -> ISP -> GStreamer -> ROS2 Image Publisher。
- 支持 `image_transport`。
- 和 `trainros_yolo_detection` 联调真实车钩检测结果。
