#!/usr/bin/env python3
import os
import signal
import subprocess
import tempfile
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


def test_recorder_action_starts_feedback_and_cancels():
    os.environ.setdefault("ROS_DOMAIN_ID", str(100 + (os.getpid() % 20)))

    import rclpy
    from rclpy.action import ActionClient
    from trainros_interfaces.action import Record

    with tempfile.TemporaryDirectory(prefix="trainros_bag_test_") as bag_dir:
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "trainros_recorder",
                "recorder_node",
                "--ros-args",
                "-p",
                f"default_output_uri:={bag_dir}/bag",
                "-p",
                "record_topics:=[/diagnostics]",
                "-p",
                "record_camera:=false",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        try:
            rclpy.init()
            node = rclpy.create_node("trainros_recorder_action_test")
            client = ActionClient(node, Record, "/record")
            assert client.wait_for_server(timeout_sec=8.0)

            feedback_seen = {"value": False}

            def on_feedback(feedback_msg):
                if feedback_msg.feedback.current_duration_sec >= 0.0:
                    feedback_seen["value"] = True

            goal = Record.Goal()
            goal.record = True
            goal.output_uri = f"{bag_dir}/bag"
            send_future = client.send_goal_async(goal, feedback_callback=on_feedback)
            rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
            goal_handle = send_future.result()
            assert goal_handle is not None
            assert goal_handle.accepted

            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not feedback_seen["value"]:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert feedback_seen["value"]

            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
            cancel_response = cancel_future.result()
            assert cancel_response is not None
            assert len(cancel_response.goals_canceling) == 1
        finally:
            try:
                if "node" in locals():
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                stop_process(process)
