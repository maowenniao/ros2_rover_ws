"""将 D435i 深度图转换为 2D /scan。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'range_min': 0.3,
                'range_max': 8.0,
                'scan_height': 1,
                'output_frame': 'camera_link',
            }],
        ),
    ])
