# TrainROS 模型转换记录

## 转换目标

将旧工程中的 YOLOv8-Pose 车钩关键点检测模型转换为 ONNX，供后续 `trainros_yolo_detection` 节点通过 ONNX Runtime 加载。

## 源模型

```text
E:\毕业设计\03_代码工程\车钩识别模型代码_整理版\models\trained\coupler_yolov8_pose_best.pt
```

## 输出模型

```text
E:\ros2\TrainROS\src\trainros_yolo_detection\models\coupler_yolov8_pose_best.onnx
```

同时保留一份 PyTorch 源模型：

```text
E:\ros2\TrainROS\src\trainros_yolo_detection\models\coupler_yolov8_pose_best.pt
```

## 转换参数

- 框架：Ultralytics `8.3.104`
- 源模型：YOLOv8n-Pose
- 输入尺寸：`640x640`
- ONNX opset：`12`
- dynamic：`false`
- simplify：`false`

## 模型结构检查

ONNX checker 已通过。

```text
input images [1, 3, 640, 640]
output output0 [1, 29, 8400]
```

输出 `29` 维含义按 YOLOv8-Pose 常见格式理解：

```text
4 bbox + 1 score + 8 keypoints * 3 = 29
```

其中每个关键点通常包含：

```text
x, y, confidence
```

## 后续接入

`trainros_yolo_detection` 已加入 Python ROS2 推理节点，当前完成：

1. ONNX Runtime session 加载。
2. OpenCV 图像转 tensor。
3. Letterbox 缩放和坐标反变换。
4. YOLOv8-Pose 输出基础解码。
5. NMS。
6. 发布 `trainros_interfaces/msg/Coupler`。

真实推理还需要 WSL Python 环境安装：

```bash
python3 -m pip install --user opencv-python onnxruntime
```

后续还需要用真实图片和视频验证关键点解码，并接入旧工程几何特征状态分类逻辑。
