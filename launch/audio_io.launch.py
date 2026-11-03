import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("audio_io"), "config", "audio_io.yaml"
    )
    config = LaunchConfiguration("config")
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            # 允许 duck_bringup 等上层 launch 传入自己的参数文件；
            # 不传时使用 audio_io 包内的默认配置。
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to the audio_io parameter file",
            ),
            # 默认不设置 namespace，节点全名为 /audio_io。
            # 上层需要多机器人/分组时显式传入 namespace:=robot1。
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional ROS namespace; empty means no namespace",
            ),
            Node(
                package="audio_io",
                executable="audio_io_node",
                name="audio_io",
                namespace=namespace,
                output="screen",
                parameters=[config],
            ),
        ]
    )
