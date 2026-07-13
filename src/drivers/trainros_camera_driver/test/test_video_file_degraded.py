#!/usr/bin/env python3
import os
import signal
import subprocess
import time


def stop_process(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=4.0)
    except Exception:
        process.kill()
        process.wait(timeout=4.0)


def test_video_file_publisher_missing_file_publishes_empty_image():
    os.environ.setdefault("ROS_DOMAIN_ID", str(140 + (os.getpid() % 20)))

    import rclpy
    from sensor_msgs.msg import Image
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "trainros_camera_driver",
            "video_file_publisher",
            "--ros-args",
            "-p",
            "video_path:=/tmp/trainros_missing_video.mp4",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )

    try:
        rclpy.init()
        node = rclpy.create_node("trainros_video_file_degraded_test")
        received = {"image": None}
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Image, "/camera/image_raw", lambda msg: received.__setitem__("image", msg), qos)

        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and received["image"] is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received["image"] is not None
        assert received["image"].height == 0
        assert received["image"].width == 0
        assert received["image"].encoding == "bgr8"
    finally:
        try:
            if "node" in locals():
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            stop_process(process)
