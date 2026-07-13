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


def test_yolo_missing_model_publishes_unknown_for_valid_image():
    os.environ.setdefault("ROS_DOMAIN_ID", str(160 + (os.getpid() % 20)))

    import rclpy
    from sensor_msgs.msg import Image
    from trainros_interfaces.msg import Coupler
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "trainros_yolo_detection",
            "yolo_detection_node",
            "--ros-args",
            "-p",
            "model_path:=/tmp/trainros_missing_model.onnx",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )

    try:
        rclpy.init()
        node = rclpy.create_node("trainros_yolo_degraded_test")
        image_pub = node.create_publisher(
            Image,
            "/camera/image_raw",
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        received = {"coupler": None}
        node.create_subscription(Coupler, "/coupler_detection", lambda msg: received.__setitem__("coupler", msg), 10)

        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and received["coupler"] is None:
            image = Image()
            image.header.stamp = node.get_clock().now().to_msg()
            image.height = 2
            image.width = 2
            image.encoding = "bgr8"
            image.step = 6
            image.data = bytes([0, 0, 255] * 4)
            image_pub.publish(image)
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received["coupler"] is not None
        assert received["coupler"].status == "unknown"
        assert received["coupler"].score == 0.0
    finally:
        try:
            if "node" in locals():
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            stop_process(process)
