#!/usr/bin/env python3
import json
import shutil
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def parse_fps(value):
    if not value or value == "0/0":
        return 30.0
    if "/" in value:
        num, den = value.split("/", 1)
        den_value = float(den)
        if den_value == 0.0:
            return 30.0
        return float(num) / den_value
    return float(value)


class VideoFilePublisher(Node):
    def __init__(self):
        super().__init__("trainros_video_file_publisher")
        self.video_path = self.declare_parameter("video_path", "").value
        self.frame_id = self.declare_parameter("frame_id", "camera_link").value
        self.loop = bool(self.declare_parameter("loop", True).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 0.0).value)
        self.output_width = int(self.declare_parameter("output_width", 0).value)
        self.output_height = int(self.declare_parameter("output_height", 0).value)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(Image, "/camera/image_raw", qos)

        self.process = None
        self.width = 0
        self.height = 0
        self.source_fps = 30.0
        self.frame_size = 0

        if not self.open_video():
            self.timer = self.create_timer(1.0, self.publish_empty)
            return

        period = 1.0 / (self.publish_rate_hz if self.publish_rate_hz > 0.0 else self.source_fps)
        self.timer = self.create_timer(period, self.publish_frame)
        self.get_logger().info(
            f"视频发布已启动：{self.video_path}，尺寸 {self.width}x{self.height}，频率 {1.0 / period:.2f} Hz")

    def destroy_node(self):
        self.close_process()
        super().destroy_node()

    def open_video(self):
        if not self.video_path:
            self.get_logger().error("未设置 video_path 参数")
            return False

        path = Path(self.video_path)
        if not path.exists():
            self.get_logger().error(f"视频文件不存在：{path}")
            return False

        if shutil.which("ffprobe") is None or shutil.which("ffmpeg") is None:
            self.get_logger().error("未找到 ffmpeg/ffprobe，请先安装：sudo apt-get install -y ffmpeg")
            return False

        try:
            info_raw = subprocess.check_output([
                "ffprobe",
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=width,height,avg_frame_rate",
                "-of",
                "json",
                str(path),
            ], text=True)
            stream = json.loads(info_raw)["streams"][0]
            src_width = int(stream["width"])
            src_height = int(stream["height"])
            self.source_fps = parse_fps(stream.get("avg_frame_rate", "30/1"))
        except Exception as exc:
            self.get_logger().error(f"读取视频信息失败：{exc}")
            return False

        self.width = self.output_width if self.output_width > 0 else src_width
        self.height = self.output_height if self.output_height > 0 else src_height
        self.frame_size = self.width * self.height * 3

        command = ["ffmpeg", "-hide_banner", "-loglevel", "error"]
        if self.loop:
            command += ["-stream_loop", "-1"]
        command += ["-i", str(path)]
        if self.width != src_width or self.height != src_height:
            command += ["-vf", f"scale={self.width}:{self.height}"]
        command += ["-f", "rawvideo", "-pix_fmt", "bgr24", "pipe:1"]

        try:
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except Exception as exc:
            self.get_logger().error(f"启动 ffmpeg 失败：{exc}")
            return False

        return True

    def close_process(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.process = None

    def publish_empty(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.encoding = "bgr8"
        msg.height = 0
        msg.width = 0
        msg.step = 0
        self.publisher.publish(msg)

    def publish_frame(self):
        if self.process is None or self.process.stdout is None:
            return

        data = self.process.stdout.read(self.frame_size)
        if len(data) != self.frame_size:
            if self.loop:
                self.get_logger().warning("视频帧读取不足，正在重启 ffmpeg")
                self.close_process()
                self.open_video()
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = self.width * 3
        msg.data = data
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = VideoFilePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
