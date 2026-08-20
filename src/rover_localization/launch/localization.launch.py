"""启动 EKF 里程计融合 + AMCL 定位。"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('rover_localization')
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'scan_topic': '/scan',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'global_frame_id': 'map',
                'tf_broadcast': True,
                'min_particles': 500,
                'max_particles': 3000,
                'kld_err': 0.05,
                'update_min_d': 0.1,
                'update_min_a': 0.1,
                'resample_interval': 1,
                'transform_tolerance': 0.5,
                'laser_min_range': 0.3,
                'laser_max_range': 12.0,
                'laser_max_beams': 60,
            }],
        ),
    ])
