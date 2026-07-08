from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def param_file(name):
    return PathJoinSubstitution([
        FindPackageShare("trainros_bringup"),
        "params",
        name,
    ])


def generate_launch_description():
    params = [
        param_file("sensors_fake_serial.yaml"),
        param_file("fusion.yaml"),
        param_file("logging.yaml"),
        param_file("monitor.yaml"),
    ]
    common_output = "screen"

    return LaunchDescription([
        DeclareLaunchArgument("imu_port", default_value="/tmp/trainros_imu_fake"),
        DeclareLaunchArgument("gps_port", default_value="/tmp/trainros_gps_fake"),
        DeclareLaunchArgument("laser_port", default_value="/tmp/trainros_laser_fake"),
        Node(
            package="trainros_imu_driver",
            executable="imu_driver_node",
            name="trainros_imu_driver",
            output=common_output,
            parameters=[*params, {"port": LaunchConfiguration("imu_port")}],
        ),
        Node(
            package="trainros_gps_driver",
            executable="gps_driver_node",
            name="trainros_gps_driver",
            output=common_output,
            parameters=[*params, {"port": LaunchConfiguration("gps_port")}],
        ),
        Node(
            package="trainros_laser_driver",
            executable="laser_driver_node",
            name="trainros_laser_driver",
            output=common_output,
            parameters=[*params, {"port": LaunchConfiguration("laser_port")}],
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
    ])
