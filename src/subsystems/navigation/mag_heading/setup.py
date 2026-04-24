from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mag_heading'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'ahrs'],
    zip_safe=True,
    maintainer='mdurrani',
    maintainer_email='mdurrani808@gmail.com',
    description='Magnetometer calibration and heading publisher for ZED camera IMU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mag_heading_node = mag_heading.mag_heading_node:main',
            'calibrate_mag = mag_heading.calibrate_mag:main',
        ],
    },
)
