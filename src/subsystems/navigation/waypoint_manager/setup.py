from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'waypoint_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinavkota06',
    maintainer_email='abhinav.kota06@gmail.com',
    description='Waypoint management and navigation node for simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_manager_node = waypoint_manager.waypoint_manager_node:main',
        ],
    },
)
