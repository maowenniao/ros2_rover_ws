"""odrive_velocity_only.launch.py - 纯 ODrive 速度控制启动文件。

此启动文件不依赖 PX4，直接通过 ODrive 速度控制节点控制电机。

使用方式：
    ros2 launch odrive_can_bridge odrive_velocity_only.launch.py

替代方案：
    如果需要 PX4 桥接，请使用 diffdrive_rover 的启动文件。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup launch nodes with proper parameter handling."""
    pkg = 'odrive_can_bridge'
    pkg_share = get_package_share_directory(pkg)
    config_dir = os.path.join(pkg_share, 'config')
    
    # 默认参数文件
    default_params_file = os.path.join(config_dir, 'odrive_can_params.yaml')
    params_file = LaunchConfiguration('params_file').perform(context)
    
    # 如果未指定参数文件，使用默认的
    if not params_file or params_file == '/dev/null':
        params_file = default_params_file

    # ODrive CAN 桥接节点
    can_node = Node(
        package=pkg,
        executable='odrive_can_node',
        name='odrive_can_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/wheel_cmd', '/wheel_cmd'),
            ('/wheel_state', '/wheel_state'),
        ],
    )

    # 轮速里程计节点
    wheel_odom_node = Node(
        package=pkg,
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen',
        parameters=[
            {
                'wheel_radius': 0.085,    # 6.5 寸轮 (0.085m)
                'track_width': 0.40,       # 左右轮距 (m)
                'wheel_base': 0.50,        # 前后轴距 (m)
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
            }
        ],
        remappings=[
            ('/wheel_state', '/wheel_state'),
            ('/wheel_odom', '/wheel_odom'),
        ],
    )

    # ODrive 速度控制节点（替代 PX4 桥接）
    velocity_control_node = Node(
        package=pkg,
        executable='odrive_velocity_control',
        name='odrive_velocity_control',
        output='screen',
        parameters=[
            {
                'wheel_radius': 0.085,
                'track_width': 0.40,
                'max_linear_vel': 5.58,
                'max_angular_vel': 68.07,
                'cmdvel_timeout': 0.5,
                'enable_safety_clamp': True,
            }
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/wheel_cmd', '/wheel_cmd'),
        ],
    )

    return [can_node, wheel_odom_node, velocity_control_node]


def generate_launch_description() -> LaunchDescription:
    """Generate launch description."""
    # 参数声明
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/dev/null',
        description='ODrive CAN 参数文件路径（默认/dev/null 使用内置默认值）',
    )

    # 使用 OpaqueFunction 动态创建节点
    opaque_launch = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        params_file_arg,
        opaque_launch,
    ])