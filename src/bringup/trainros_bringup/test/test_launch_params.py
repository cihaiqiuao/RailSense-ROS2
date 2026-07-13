#!/usr/bin/env python3
from pathlib import Path

import yaml


def params_for(root, file_name, node_name):
    data = yaml.safe_load((root / "params" / file_name).read_text(encoding="utf-8"))
    return data[node_name]["ros__parameters"]


def test_launch_files_and_yaml_exist():
    root = Path(__file__).resolve().parents[1]
    for path in [
        root / "launch" / "trainros_system.launch.py",
        root / "launch" / "trainros_fake_serial.launch.py",
        root / "params" / "sensors.yaml",
        root / "params" / "sensors_fake_serial.yaml",
        root / "params" / "fusion.yaml",
        root / "params" / "logging.yaml",
        root / "params" / "monitor.yaml",
    ]:
        assert path.exists(), f"缺少工程入口文件: {path}"


def test_sensor_parameters_are_layered_and_complete():
    root = Path(__file__).resolve().parents[1]
    for node_name in ["trainros_imu_driver", "trainros_gps_driver", "trainros_laser_driver"]:
        params = params_for(root, "sensors.yaml", node_name)
        assert params["port"]
        assert isinstance(params["baud_rate"], int)
        assert params["baud_rate"] > 0
        assert params["frame_id"]

    fake_imu = params_for(root, "sensors_fake_serial.yaml", "trainros_imu_driver")
    assert fake_imu["port"] != params_for(root, "sensors.yaml", "trainros_imu_driver")["port"]


def test_fusion_stability_logging_and_monitor_parameters_exist():
    root = Path(__file__).resolve().parents[1]

    fusion = params_for(root, "fusion.yaml", "trainros_fusion")
    assert fusion["output_rate_hz"] == 50.0
    assert "sync_slop_ms" in fusion
    assert "gps_timeout_ms" in fusion
    assert "enable_kalman_filter" in fusion

    stability = params_for(root, "fusion.yaml", "trainros_stability_evaluator")
    for key in [
        "sample_rate_hz",
        "min_window_samples",
        "psd_min_hz",
        "psd_max_hz",
        "rms_weight",
        "peak_weight",
        "psd_weight",
        "tsi_alarm_value",
    ]:
        assert key in stability

    logging = params_for(root, "logging.yaml", "trainros_logger")
    assert logging["log_dir"]
    recorder = params_for(root, "logging.yaml", "trainros_recorder")
    assert recorder["record_camera"] is False
    assert "/camera/image_raw" not in recorder["record_topics"]

    monitor = params_for(root, "monitor.yaml", "trainros_monitor")
    assert monitor["publish_rate_hz"] > 0.0
    assert monitor["topic_timeout_ms"] > 0
