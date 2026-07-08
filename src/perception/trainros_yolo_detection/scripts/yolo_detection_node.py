#!/usr/bin/env python3
import math
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from trainros_interfaces.msg import Coupler

try:
    import cv2
except Exception:
    cv2 = None

try:
    import onnxruntime as ort
except Exception:
    ort = None


def resolve_model_path(model_path):
    path = Path(model_path)
    if path.is_absolute():
        return path
    package_share = Path(get_package_share_directory("trainros_yolo_detection"))
    return package_share / path


def image_msg_to_bgr(msg):
    if msg.height == 0 or msg.width == 0 or not msg.data:
        return None

    data = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("bgr8", "rgb8"):
        expected = int(msg.height) * int(msg.width) * 3
        if data.size < expected:
            return None
        image = data[:expected].reshape((msg.height, msg.width, 3))
        if msg.encoding == "rgb8":
            image = image[:, :, ::-1]
        return image.copy()

    if msg.encoding in ("mono8", "8UC1"):
        expected = int(msg.height) * int(msg.width)
        if data.size < expected:
            return None
        gray = data[:expected].reshape((msg.height, msg.width))
        if cv2 is None:
            return np.repeat(gray[:, :, None], 3, axis=2)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    return None


def letterbox(image, target_w, target_h):
    h, w = image.shape[:2]
    scale = min(target_w / float(w), target_h / float(h))
    new_w = int(round(w * scale))
    new_h = int(round(h * scale))
    pad_w = (target_w - new_w) / 2.0
    pad_h = (target_h - new_h) / 2.0

    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((target_h, target_w, 3), 114, dtype=np.uint8)
    left = int(round(pad_w - 0.1))
    top = int(round(pad_h - 0.1))
    canvas[top:top + new_h, left:left + new_w] = resized
    return canvas, scale, left, top


def nms_xyxy(boxes, scores, iou_threshold):
    if not boxes:
        return []

    boxes_np = np.asarray(boxes, dtype=np.float32)
    scores_np = np.asarray(scores, dtype=np.float32)
    order = scores_np.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break

        rest = order[1:]
        xx1 = np.maximum(boxes_np[i, 0], boxes_np[rest, 0])
        yy1 = np.maximum(boxes_np[i, 1], boxes_np[rest, 1])
        xx2 = np.minimum(boxes_np[i, 2], boxes_np[rest, 2])
        yy2 = np.minimum(boxes_np[i, 3], boxes_np[rest, 3])

        inter_w = np.maximum(0.0, xx2 - xx1)
        inter_h = np.maximum(0.0, yy2 - yy1)
        inter = inter_w * inter_h

        area_i = (boxes_np[i, 2] - boxes_np[i, 0]) * (boxes_np[i, 3] - boxes_np[i, 1])
        area_rest = (boxes_np[rest, 2] - boxes_np[rest, 0]) * (boxes_np[rest, 3] - boxes_np[rest, 1])
        union = np.maximum(area_i + area_rest - inter, 1e-6)
        iou = inter / union
        order = rest[iou <= iou_threshold]

    return keep


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__("trainros_yolo_detection")
        self.model_path = self.declare_parameter("model_path", "models/coupler_yolov8_pose_best.onnx").value
        self.input_width = int(self.declare_parameter("input_width", 640).value)
        self.input_height = int(self.declare_parameter("input_height", 640).value)
        self.conf_threshold = float(self.declare_parameter("conf_threshold", 0.25).value)
        self.iou_threshold = float(self.declare_parameter("iou_threshold", 0.45).value)

        self.session = None
        self.input_name = ""
        self.output_name = ""
        self.load_model()

        self.publisher = self.create_publisher(Coupler, "/coupler_detection", 10)
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.on_image,
            rclpy.qos.QoSProfile(depth=5, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT),
        )
        self.get_logger().info("YOLO detection node 已启动")

    def load_model(self):
        if cv2 is None:
            self.get_logger().warning("OpenCV 未安装，YOLO 节点将发布 unknown")
            return
        if ort is None:
            self.get_logger().warning("onnxruntime 未安装，YOLO 节点将发布 unknown")
            return

        path = resolve_model_path(self.model_path)
        if not path.exists():
            self.get_logger().warning(f"模型文件不存在：{path}")
            return

        self.session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.get_logger().info(f"ONNX 模型已加载：{path}")

    def publish_empty(self, header, status):
        msg = Coupler()
        msg.header = header
        msg.bbox = [0.0, 0.0, 0.0, 0.0]
        msg.score = 0.0
        msg.status = status
        self.publisher.publish(msg)

    def preprocess(self, image):
        padded, scale, pad_x, pad_y = letterbox(image, self.input_width, self.input_height)
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        tensor = rgb.astype(np.float32) / 255.0
        tensor = np.transpose(tensor, (2, 0, 1))[None, :, :, :]
        return np.ascontiguousarray(tensor), scale, pad_x, pad_y

    def decode(self, output, image_shape, scale, pad_x, pad_y):
        pred = np.asarray(output)
        if pred.ndim == 3:
            pred = pred[0]
        if pred.shape[0] == 29:
            pred = pred.T

        h, w = image_shape[:2]
        boxes = []
        scores = []
        keypoints_all = []

        for row in pred:
            score = float(row[4])
            if score < self.conf_threshold or not math.isfinite(score):
                continue

            cx, cy, bw, bh = [float(v) for v in row[:4]]
            x1 = (cx - bw / 2.0 - pad_x) / scale
            y1 = (cy - bh / 2.0 - pad_y) / scale
            x2 = (cx + bw / 2.0 - pad_x) / scale
            y2 = (cy + bh / 2.0 - pad_y) / scale
            x1 = max(0.0, min(float(w), x1))
            y1 = max(0.0, min(float(h), y1))
            x2 = max(0.0, min(float(w), x2))
            y2 = max(0.0, min(float(h), y2))

            kps = []
            kp_raw = row[5:]
            for i in range(0, min(len(kp_raw), 24), 3):
                point = Point()
                point.x = float((kp_raw[i] - pad_x) / scale)
                point.y = float((kp_raw[i + 1] - pad_y) / scale)
                point.z = float(kp_raw[i + 2])
                kps.append(point)

            boxes.append([x1, y1, x2, y2])
            scores.append(score)
            keypoints_all.append(kps)

        keep = nms_xyxy(boxes, scores, self.iou_threshold)
        if not keep:
            return None
        best = keep[0]
        return boxes[best], scores[best], keypoints_all[best]

    def on_image(self, image_msg):
        image = image_msg_to_bgr(image_msg)
        if image is None:
            self.publish_empty(image_msg.header, "no_image")
            return
        if self.session is None:
            self.publish_empty(image_msg.header, "unknown")
            return

        tensor, scale, pad_x, pad_y = self.preprocess(image)
        output = self.session.run([self.output_name], {self.input_name: tensor})[0]
        decoded = self.decode(output, image.shape, scale, pad_x, pad_y)
        if decoded is None:
            self.publish_empty(image_msg.header, "not_detected")
            return

        box, score, keypoints = decoded
        msg = Coupler()
        msg.header = image_msg.header
        msg.bbox = [float(box[0]), float(box[1]), float(box[2]), float(box[3])]
        msg.keypoints = keypoints
        msg.score = float(score)
        msg.status = "detected"
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
