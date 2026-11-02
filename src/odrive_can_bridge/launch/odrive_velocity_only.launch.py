"""与 odrive_can.launch.py 相同的底盘链路。保留这个名字以免旧命令失效。"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('odrive_can_bridge'),
                    'launch',
                    'odrive_can.launch.py',
                )
            )
        )
    ])
