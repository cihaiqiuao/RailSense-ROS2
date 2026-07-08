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
    params = [param_file("sensors.yaml")]

    return LaunchDescription([
        DeclareLaunchArgument("video_path", default_value="/mnt/d/chegou/视频/WeChat_20250616101426.mp4"),
        DeclareLaunchArgument("loop", default_value="true"),
        DeclareLaunchArgument("publish_rate_hz", default_value="0.0"),
        Node(
            package="trainros_camera_driver",
            executable="video_file_publisher",
            name="trainros_video_file_publisher",
            output="screen",
            parameters=[{
                "video_path": LaunchConfiguration("video_path"),
                "loop": LaunchConfiguration("loop"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            }],
        ),
        Node(
            package="trainros_yolo_detection",
            executable="yolo_detection_node",
            name="trainros_yolo_detection",
            output="screen",
            parameters=params,
        ),
    ])
