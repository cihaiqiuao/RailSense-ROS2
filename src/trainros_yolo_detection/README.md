# trainros_yolo_detection

车钩检测与状态识别 ROS2 包。

## 当前模型

已从旧工程训练模型导出 ONNX：

- PyTorch 源模型：`models/coupler_yolov8_pose_best.pt`
- ONNX 模型：`models/coupler_yolov8_pose_best.onnx`
- 输入：`images [1, 3, 640, 640]`
- 输出：`output0 [1, 29, 8400]`
- 模型类型：YOLOv8-Pose 车钩关键点检测模型

导出来源：

```text
E:\毕业设计\03_代码工程\车钩识别模型代码_整理版\models\trained\coupler_yolov8_pose_best.pt
```

当前 `trainros_bringup/params/sensors.yaml` 中默认配置：

```yaml
trainros_yolo_detection:
  ros__parameters:
    model_path: models/coupler_yolov8_pose_best.onnx
    input_width: 640
    input_height: 640
```

## 计划输入

- `/camera/image_raw`

## 计划输出

- `/coupler_detection`
- 类型：`trainros_interfaces/msg/Coupler`
- QoS：Reliable

## 当前节点状态

- 已提供 Python ROS2 节点入口 `yolo_detection_node`。
- 已实现 OpenCV letterbox 预处理。
- 已实现 ONNX Runtime session 加载。
- 已实现 YOLOv8-Pose 输出基础解码。
- 已实现 bbox NMS。
- 已将 bbox、keypoints、score、status 发布为 `Coupler.msg`。
- 缺少 `opencv-python` 或 `onnxruntime` 时，节点不会崩溃，会降级发布 `unknown`。

## WSL 运行依赖

真实推理需要安装：

```bash
python3 -m pip install --user opencv-python onnxruntime
```

如果 WSL 里还没有 pip，需要先安装：

```bash
sudo apt-get update
sudo apt-get install -y python3-pip
```

当前环境如果没有 sudo 密码，需要你在 WSL 终端手动输入密码安装。

## 后续增强

- 用真实图片和视频验证 keypoint 解码精度。
- 结合旧工程几何特征逻辑完成车钩状态分类。
- 后续可迁移到 C++ ONNX Runtime 或 TensorRT。
- 结合旧工程几何特征逻辑完成车钩状态分类。
