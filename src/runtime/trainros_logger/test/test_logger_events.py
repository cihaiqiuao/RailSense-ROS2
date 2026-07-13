#!/usr/bin/env python3
import json
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path


def stop_process(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=4.0)
    except Exception:
        process.kill()
        process.wait(timeout=4.0)


def wait_for_events(log_path, expected_events, timeout_sec=10.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if log_path.exists():
            events = []
            with log_path.open("r", encoding="utf-8") as handle:
                for line in handle:
                    if line.strip():
                        events.append(json.loads(line)["event"])
            if expected_events.issubset(set(events)):
                return events
        time.sleep(0.1)
    return []


def make_key(key, value):
    from diagnostic_msgs.msg import KeyValue

    item = KeyValue()
    item.key = key
    item.value = value
    return item


def publish_diagnostics(node, publisher, serial_open, reconnect_count, dropped_frames, has_fix):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

    msg = DiagnosticArray()
    msg.header.stamp = node.get_clock().now().to_msg()
    status = DiagnosticStatus()
    status.name = "trainros_gps_driver"
    status.hardware_id = "software"
    status.level = DiagnosticStatus.OK if serial_open == "true" and has_fix == "true" else DiagnosticStatus.WARN
    status.message = "测试诊断状态"
    status.values = [
        make_key("serial_open", serial_open),
        make_key("port", "/tmp/fake_gps"),
        make_key("reconnect_count", str(reconnect_count)),
        make_key("dropped_frames", str(dropped_frames)),
        make_key("has_fix", has_fix),
    ]
    msg.status.append(status)
    publisher.publish(msg)


def publish_stability(node, publisher, level, warning=False, alarm=False):
    from trainros_interfaces.msg import Stability

    msg = Stability()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.score = 30.0 if alarm else 60.0
    msg.rms_acceleration = 1.5
    msg.peak_acceleration = 3.0
    msg.warning = warning
    msg.alarm = alarm
    msg.level = level
    publisher.publish(msg)


def test_logger_writes_business_events_without_duplicate_spam():
    os.environ.setdefault("ROS_DOMAIN_ID", str(60 + (os.getpid() % 20)))

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from trainros_interfaces.msg import Stability

    with tempfile.TemporaryDirectory(prefix="trainros_logger_test_") as log_dir:
        log_path = Path(log_dir) / "trainros_business.jsonl"
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "trainros_logger",
                "logger_node",
                "--ros-args",
                "-p",
                f"log_dir:={log_dir}",
                "-p",
                "status_period_ms:=100",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        try:
            rclpy.init()
            node = rclpy.create_node("trainros_logger_event_test")
            diag_pub = node.create_publisher(DiagnosticArray, "/diagnostics", 10)
            stability_pub = node.create_publisher(Stability, "/stability", 10)

            time.sleep(1.0)
            publish_diagnostics(node, diag_pub, "true", 0, 0, "true")
            publish_diagnostics(node, diag_pub, "false", 1, 2, "false")
            publish_stability(node, stability_pub, "warning", warning=True)
            publish_stability(node, stability_pub, "alarm", warning=True, alarm=True)

            expected = {
                "logger_started",
                "serial_opened",
                "serial_closed",
                "serial_reconnect",
                "parser_drop_frame",
                "gps_fix",
                "gps_no_fix",
                "stability_warning",
                "stability_alarm",
            }
            events = wait_for_events(log_path, expected)
            assert expected.issubset(set(events))

            old_count = len(events)
            publish_diagnostics(node, diag_pub, "false", 1, 2, "false")
            publish_stability(node, stability_pub, "alarm", warning=True, alarm=True)
            time.sleep(0.5)
            with log_path.open("r", encoding="utf-8") as handle:
                new_events = [json.loads(line)["event"] for line in handle if line.strip()]
            assert len(new_events) == old_count
        finally:
            try:
                if "node" in locals():
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                stop_process(process)
