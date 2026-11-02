"""Intel RealSense D435i。分辨率在启动时拼成驱动要求的 profile 字符串。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text: str) -> bool:
    return text.lower() in ('1', 'true', 'yes', 'on')


def _launch(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    depth = f"{arg('depth_width')}x{arg('depth_height')}x{arg('depth_fps')}"
    color = f"{arg('color_width')}x{arg('color_height')}x{arg('color_fps')}"
    return [Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        namespace=arg('camera_name'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'depth_module.depth_profile': depth,
            'rgb_camera.color_profile': color,
            'enable_infra1': _as_bool(arg('enable_infra1')),
            'enable_infra2': _as_bool(arg('enable_infra2')),
            'enable_gyro': _as_bool(arg('enable_gyro')),
            'enable_accel': _as_bool(arg('enable_accel')),
            'pointcloud.enable': _as_bool(arg('enable_pointcloud')),
            'align_depth.enable': _as_bool(arg('enable_align_depth')),
            'unite_imu_method': int(arg('unite_imu_method')),
            'serial_no': arg('serial_no'),
            'device_type': 'd435i',
        }],
    )]


def generate_launch_description() -> LaunchDescription:
    names = [
        ('depth_width', '640'),
        ('depth_height', '480'),
        ('depth_fps', '30'),
        ('color_width', '640'),
        ('color_height', '480'),
        ('color_fps', '30'),
        ('enable_infra1', 'true'),
        ('enable_infra2', 'true'),
        ('enable_gyro', 'true'),
        ('enable_accel', 'true'),
        ('enable_pointcloud', 'true'),
        ('enable_align_depth', 'true'),
        ('unite_imu_method', '1'),
        ('serial_no', ''),
        ('camera_name', 'd435i'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in names]
        + [OpaqueFunction(function=_launch)]
    )
