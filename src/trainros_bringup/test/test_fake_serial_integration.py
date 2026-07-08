#!/usr/bin/env python3
import math
import os
import pty
import signal
import struct
import subprocess
import threading
import time


def checksum_11(frame):
    frame[10] = sum(frame[:10]) & 0xFF


def make_imu_frame():
    accel = bytearray(11)
    accel[0] = 0x55
    accel[1] = 0x51
    struct.pack_into("<hhh", accel, 2, 1024, -512, 2048)
    checksum_11(accel)

    gyro = bytearray(11)
    gyro[0] = 0x55
    gyro[1] = 0x52
    struct.pack_into("<hhh", gyro, 2, 20, -20, 40)
    checksum_11(gyro)

    angle = bytearray(11)
    angle[0] = 0x55
    angle[1] = 0x53
    struct.pack_into("<hhh", angle, 2, 100, -100, 50)
    checksum_11(angle)
    return bytes(accel + gyro + angle)


def make_laser_frame(distance_mm=1234):
    frame = bytearray(12)
    frame[0] = 0xAA
    struct.pack_into("<H", frame, 9, distance_mm)
    return bytes(frame)


class FakeSerialPorts:
    def __init__(self):
        self.imu_master, self.imu_slave = pty.openpty()
        self.gps_master, self.gps_slave = pty.openpty()
        self.laser_master, self.laser_slave = pty.openpty()
        self.imu_port = os.ttyname(self.imu_slave)
        self.gps_port = os.ttyname(self.gps_slave)
        self.laser_port = os.ttyname(self.laser_slave)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        for fd in [
            self.imu_master,
            self.imu_slave,
            self.gps_master,
            self.gps_slave,
            self.laser_master,
            self.laser_slave,
        ]:
            try:
                os.close(fd)
            except OSError:
                pass

    def _write(self, fd, data):
        try:
            os.write(fd, data)
        except OSError:
            pass

    def _run(self):
        imu_frame = make_imu_frame()
        gps_line = b"$GNRMC,092751.000,A,5321.6802,N,00630.3372,W,0.06,31.66,280511,,,A*43\r\n"
        laser_frame = make_laser_frame()
        last_gps = 0.0
        last_laser = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            self._write(self.imu_master, imu_frame)
            if now - last_gps >= 0.1:
                self._write(self.gps_master, gps_line)
                last_gps = now
            if now - last_laser >= 0.05:
                self._write(self.laser_master, laser_frame)
                last_laser = now
            time.sleep(0.005)


def start_driver(package, executable, port):
    return subprocess.Popen(
        [
            "ros2",
            "run",
            package,
            executable,
            "--ros-args",
            "-p",
            f"port:={port}",
            "-p",
            "reconnect_ms:=100",
            "-p",
            "diagnostics_period_ms:=200",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def start_fusion():
    return subprocess.Popen(
        [
            "ros2",
            "run",
            "trainros_fusion",
            "fusion_node",
            "--ros-args",
            "-p",
            "sync_slop_ms:=80",
            "-p",
            "diagnostics_period_ms:=200",
            "-p",
            "gps_timeout_ms:=1000",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def start_monitor():
    return subprocess.Popen(
        [
            "ros2",
            "run",
            "trainros_monitor",
            "monitor_node",
            "--ros-args",
            "-p",
            "publish_rate_hz:=5.0",
            "-p",
            "topic_timeout_ms:=1000",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def stop_process(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=4.0)
    except Exception:
        process.kill()
        process.wait(timeout=4.0)


def test_fake_serial_drivers_publish_topics_and_diagnostics():
    os.environ.setdefault("ROS_DOMAIN_ID", str(120 + (os.getpid() % 20)))

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import Imu, LaserScan, NavSatFix, NavSatStatus
    from trainros_interfaces.msg import TrainState
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    ports = FakeSerialPorts()
    ports.start()
    processes = []

    try:
        processes = [
            start_driver("trainros_imu_driver", "imu_driver_node", ports.imu_port),
            start_driver("trainros_gps_driver", "gps_driver_node", ports.gps_port),
            start_driver("trainros_laser_driver", "laser_driver_node", ports.laser_port),
            start_fusion(),
            start_monitor(),
        ]

        rclpy.init()
        node = rclpy.create_node("trainros_fake_serial_integration_test")
        received = {
            "imu": None,
            "gps": None,
            "laser": None,
            "train_state": None,
            "train_state_count": 0,
            "diagnostics": set(),
            "diagnostic_values": {},
        }

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        node.create_subscription(Imu, "/imu/data", lambda msg: received.__setitem__("imu", msg), sensor_qos)
        node.create_subscription(NavSatFix, "/gps/fix", lambda msg: received.__setitem__("gps", msg), 10)
        node.create_subscription(
            LaserScan, "/laser/scan", lambda msg: received.__setitem__("laser", msg), sensor_qos)
        node.create_subscription(
            TrainState,
            "/train_state",
            lambda msg: (
                received.__setitem__("train_state", msg),
                received.__setitem__("train_state_count", received["train_state_count"] + 1),
            ),
            10)

        def on_diagnostics(msg):
            for status in msg.status:
                received["diagnostics"].add(status.name)
                received["diagnostic_values"][status.name] = {item.key: item.value for item in status.values}

        node.create_subscription(DiagnosticArray, "/diagnostics", on_diagnostics, 10)

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                received["imu"] is not None
                and received["gps"] is not None
                and received["laser"] is not None
                and received["train_state"] is not None
                and {
                    "trainros_imu_driver",
                    "trainros_gps_driver",
                    "trainros_laser_driver",
                    "trainros_fusion",
                    "trainros_monitor",
                }.issubset(
                    received["diagnostics"]
                )
            ):
                break

        assert received["imu"] is not None
        assert not math.isclose(received["imu"].linear_acceleration.x, 0.0)

        assert received["gps"] is not None
        assert received["gps"].status.status == NavSatStatus.STATUS_FIX
        assert math.isclose(received["gps"].latitude, 53.3613366667, rel_tol=0.0, abs_tol=1e-5)
        assert math.isclose(received["gps"].longitude, -6.50562, rel_tol=0.0, abs_tol=1e-5)

        assert received["laser"] is not None
        assert len(received["laser"].ranges) == 1
        assert math.isclose(received["laser"].ranges[0], 1.234, rel_tol=0.0, abs_tol=1e-3)

        assert received["train_state"] is not None
        assert not math.isclose(received["train_state"].acceleration, 0.0)
        assert math.isclose(received["train_state"].laser_distance, 1.234, rel_tol=0.0, abs_tol=1e-3)

        assert received["train_state_count"] >= 5
        assert {"trainros_imu_driver", "trainros_gps_driver", "trainros_laser_driver"}.issubset(
            received["diagnostics"]
        )
        assert "trainros_fusion" in received["diagnostics"]
        assert "imu_laser_sync_count" in received["diagnostic_values"]["trainros_fusion"]
        assert "gps_age_ms" in received["diagnostic_values"]["trainros_fusion"]
        assert "state_latency_ms" in received["diagnostic_values"]["trainros_fusion"]
        assert received["diagnostic_values"]["trainros_fusion"].get("kalman_enabled") == "true"
        assert received["diagnostic_values"]["trainros_fusion"].get("kalman_initialized") == "true"
        assert "kalman_speed" in received["diagnostic_values"]["trainros_fusion"]
        assert "kalman_acceleration" in received["diagnostic_values"]["trainros_fusion"]
        assert "kalman_imu_update_count" in received["diagnostic_values"]["trainros_fusion"]
        assert "kalman_gps_update_count" in received["diagnostic_values"]["trainros_fusion"]
        assert "trainros_monitor" in received["diagnostics"]
        assert "imu_latency_ms" in received["diagnostic_values"]["trainros_monitor"]
        assert "laser_latency_ms" in received["diagnostic_values"]["trainros_monitor"]
        assert "train_state_latency_ms" in received["diagnostic_values"]["trainros_monitor"]
    finally:
        try:
            if "node" in locals():
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for process in processes:
                stop_process(process)
            ports.stop()
