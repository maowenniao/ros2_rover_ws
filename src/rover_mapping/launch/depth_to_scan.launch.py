"""深度图转 /scan。射线在深度光学坐标系里计算，frame 必须是该光学帧。"""
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
                ('depth', '/d435i/d435i/depth/image_rect_raw'),
                ('depth_camera_info', '/d435i/d435i/depth/camera_info'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'range_min': 0.28,
                'range_max': 4.0,
                'scan_height': 8,
                'scan_time': 0.067,
                'output_frame': 'd435i_depth_optical_frame',
            }],
        ),
    ])
