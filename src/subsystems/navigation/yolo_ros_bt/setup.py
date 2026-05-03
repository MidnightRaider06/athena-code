from setuptools import setup
import os
from glob import glob

package_name = 'yolo_ros_bt'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdul',
    maintainer_email='abdul@example.com',
    description='YOLO ROS 2 perception node for Behavior Tree integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_ros_bt.yolo_ros_node:main',
        ],
    },
)