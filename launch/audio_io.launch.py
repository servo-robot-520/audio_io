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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to the audio_io parameter file",
            ),
            Node(
                package="audio_io",
                executable="audio_io_node",
                name="io",
                namespace="audio",
                output="screen",
                parameters=[config],
            ),
        ]
    )
