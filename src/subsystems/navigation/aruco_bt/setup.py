import os
from glob import glob
from setuptools import setup

package_name = 'aruco_bt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinavkota06',
    maintainer_email='abhinav.kota06@gmail.com',
    description='Python package for aruco detection and pose estimation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = aruco_bt.aruco_node:main',
        ],
    },
)
