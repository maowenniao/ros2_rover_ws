"""启动多点巡路任务 + 安全监督。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('rover_mission')
    return LaunchDescription([
        DeclareLaunchArgument('waypoints_file',
                              default_value=os.path.join(pkg, 'config', 'waypoints.yaml')),
        DeclareLaunchArgument('loop', default_value='true'),
        Node(
            package='rover_mission',
            executable='waypoint_mission_node',
            name='waypoint_mission_node',
            output='screen',
            parameters=[{
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'loop': LaunchConfiguration('loop'),
            }],
        ),
        Node(
            package='rover_mission',
            executable='safety_monitor_node',
            name='safety_monitor_node',
            output='screen',
        ),
    ])
