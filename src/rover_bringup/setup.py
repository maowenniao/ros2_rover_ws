from setuptools import setup
import os
from glob import glob

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        ('lib/' + package_name, [
            'scripts/setup_can.sh',
            'scripts/setup_pixhawk_usb.sh',
            'scripts/install_uxrce_service.sh',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Developer',
    maintainer_email='you@example.com',
    description='四轴差动无人小车系统启动包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_rover = rover_bringup.arm_rover:main',
            'estop_rover = rover_bringup.estop_rover:main',
        ],
    },
)