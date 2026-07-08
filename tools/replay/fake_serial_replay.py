#!/usr/bin/env python3
"""TrainROS 伪串口回放工具。

运行后会创建 IMU/GPS/Laser 三个 PTY，并打印对应的从设备路径。
可把这些路径传给 `trainros_fake_serial.launch.py` 的
`imu_port`、`gps_port`、`laser_port` 参数，在无硬件环境验证传感器 Topic。
"""

import os
import pty
import signal
import struct
import time


def checksum_11(frame: bytearray) -> None:
    frame[10] = sum(frame[:10]) & 0xFF


def make_imu_frame() -> bytes:
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


def make_laser_frame(distance_mm: int = 1234) -> bytes:
    frame = bytearray(12)
    frame[0] = 0xAA
    struct.pack_into("<H", frame, 9, distance_mm)
    return bytes(frame)


def open_pty(name: str):
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    print(f"{name}: {slave_name}", flush=True)
    return master


def main() -> None:
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    imu_master = open_pty("imu_port")
    gps_master = open_pty("gps_port")
    laser_master = open_pty("laser_port")
    print("把上面三个路径传给 trainros_fake_serial.launch.py 后启动 TrainROS。按 Ctrl+C 退出。", flush=True)

    imu_frame = make_imu_frame()
    laser_frame = make_laser_frame()
    gps_line = b"$GNRMC,092751.000,A,5321.6802,N,00630.3372,W,0.06,31.66,280511,,,A*43\r\n"

    last_gps = 0.0
    last_laser = 0.0
    while running:
        now = time.monotonic()
        os.write(imu_master, imu_frame)
        if now - last_gps >= 0.1:
            os.write(gps_master, gps_line)
            last_gps = now
        if now - last_laser >= 0.05:
            os.write(laser_master, laser_frame)
            last_laser = now
        time.sleep(0.005)


if __name__ == "__main__":
    main()
