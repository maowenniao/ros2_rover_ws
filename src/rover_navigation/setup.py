from setuptools import setup
import os
from glob import glob

package_name = 'rover_navigation'

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
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='maowenniao',
    maintainer_email='maowenniao@163.com',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gps_waypoint_node = rover_navigation.gps_waypoint_node:main',
        ],
    },
)
