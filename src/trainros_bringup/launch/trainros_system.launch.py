from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def param_file(name):
    return PathJoinSubstitution([
        FindPackageShare("trainros_bringup"),
        "params",
        name,
    ])


def generate_launch_description():
    params = [
        param_file("sensors.yaml"),
        param_file("fusion.yaml"),
        param_file("logging.yaml"),
        param_file("monitor.yaml"),
    ]
    common_output = "screen"

    return LaunchDescription([
        Node(
            package="trainros_imu_driver",
            executable="imu_driver_node",
            name="trainros_imu_driver",
            output=common_output,
            parameters=params,
        ),
        Node(
            package="trainros_gps_driver",
            executable="gps_driver_node",
            name="trainros_gps_driver",
            output=common_output,
            parameters=params,
        ),
        Node(
            package="trainros_laser_driver",
            executable="laser_driver_node",
            name="trainros_laser_driver",
            output=common_output,
            parameters=params,
        ),
        Node(
            package="trainros_camera_driver",
            executable="camera_driver_node",
            name="trainros_camera_driver",
            output=common_output,
            parameters=params,
        ),
        Node(
            package="trainros_yolo_detection",
            executable="yolo_detection_node",
            name="trainros_yolo_detection",
            output=common_output,
            parameters=params,
        ),
        Node(
            package="trainros_fusion",
            executable="fusion_node",
            name="trainros_fusion",
            output=common_output,
            parameters=params,
        ),
        Node(
            package="trainros_stability_evaluator",
            executable="stability_evaluator_node",
            name="trainros_stability_evaluator",
            output=common_output,
            parameters=params,
        ),
        Node(
            package="trainros_recorder",
            executable="recorder_node",
            name="trainros_recorder",
            output=common_output,
            parameters=params,
        ),
        Node(
            package="trainros_logger",
            executable="logger_node",
            name="trainros_logger",
            output=common_output,
            parameters=params,
        ),
        Node(
            package="trainros_monitor",
            executable="monitor_node",
            name="trainros_monitor",
            output=common_output,
            parameters=params,
        ),
    ])
