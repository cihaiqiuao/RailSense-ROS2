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


def test_monitor_publishes_latency_diagnostics_and_warns_on_timeout():
    os.environ.setdefault("ROS_DOMAIN_ID", str(80 + (os.getpid() % 20)))

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from sensor_msgs.msg import Imu, LaserScan, NavSatFix
    from trainros_interfaces.msg import TrainState
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "trainros_monitor",
            "monitor_node",
            "--ros-args",
            "-p",
            "publish_rate_hz:=10.0",
            "-p",
            "topic_timeout_ms:=300",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )

    try:
        rclpy.init()
        node = rclpy.create_node("trainros_monitor_latency_test")
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        imu_pub = node.create_publisher(Imu, "/imu/data", sensor_qos)
        gps_pub = node.create_publisher(NavSatFix, "/gps/fix", 10)
        laser_pub = node.create_publisher(LaserScan, "/laser/scan", sensor_qos)
        train_state_pub = node.create_publisher(TrainState, "/train_state", 10)

        received = {"ok": None, "warn": None}

        def on_diag(msg):
            for status in msg.status:
                if status.name != "trainros_monitor":
                    continue
                values = {item.key: item.value for item in status.values}
                if {
                    "imu_latency_ms",
                    "gps_latency_ms",
                    "laser_latency_ms",
                    "train_state_latency_ms",
                }.issubset(values):
                    if status.level == DiagnosticStatus.OK:
                        received["ok"] = values
                    if status.level == DiagnosticStatus.WARN:
                        received["warn"] = values

        node.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 10)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and received["ok"] is None:
            stamp = node.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp = stamp
            gps = NavSatFix()
            gps.header.stamp = stamp
            laser = LaserScan()
            laser.header.stamp = stamp
            train_state = TrainState()
            train_state.header.stamp = stamp
            imu_pub.publish(imu)
            gps_pub.publish(gps)
            laser_pub.publish(laser)
            train_state_pub.publish(train_state)
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received["ok"] is not None
        assert float(received["ok"]["imu_latency_ms"]) >= 0.0
        assert float(received["ok"]["train_state_latency_ms"]) >= 0.0

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and received["warn"] is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received["warn"] is not None
    finally:
        try:
            if "node" in locals():
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            stop_process(process)
