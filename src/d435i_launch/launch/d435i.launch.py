"""D435i：驱动参数来自 yaml，光学坐标系用静态 TF 接到 URDF 的 d435i_link。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 名义外参来自 realsense2_description 的 D435/D435i xacro，不是单台标定值。
_OPTICAL = ['--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963']


def _static(x, y, z, parent, child, optical=False):
    args = ['--x', str(x), '--y', str(y), '--z', str(z),
            '--frame-id', parent, '--child-frame-id', child]
    if optical:
        args[6:6] = _OPTICAL
    else:
        args[6:6] = ['--roll', '0', '--pitch', '0', '--yaw', '0']
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'tf_{child}',
        output='screen',
        arguments=args,
    )


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('d435i_launch')
    config = os.path.join(share, 'config', 'd435i.yaml')
    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('serial_no', default_value=''),
        DeclareLaunchArgument('camera_name', default_value='d435i'),
        DeclareLaunchArgument('camera_namespace', default_value='d435i'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_namespace': LaunchConfiguration('camera_namespace'),
                'serial_no': LaunchConfiguration('serial_no'),
                'config_file': config,
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'true',
                'publish_tf': 'false',
                'rgb_camera.color_profile': '640x480x15',
                'depth_module.depth_profile': '640x480x15',
            }.items(),
        ),
        _static(0, 0, 0, 'd435i_link', 'd435i_depth_frame'),
        _static(0, 0, 0, 'd435i_depth_frame', 'd435i_depth_optical_frame', optical=True),
        _static(0, 0.015, 0, 'd435i_link', 'd435i_color_frame'),
        _static(0, 0, 0, 'd435i_color_frame', 'd435i_color_optical_frame', optical=True),
        _static(-0.01174, -0.00552, 0.0051, 'd435i_link', 'd435i_imu_frame'),
        _static(0, 0, 0, 'd435i_imu_frame', 'd435i_imu_optical_frame', optical=True),
    ])
