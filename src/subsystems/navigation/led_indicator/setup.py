from setuptools import find_packages, setup
from glob import glob
import os

package_name = "led_indicator"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="mdurrani",
    maintainer_email="mdurrani808@gmail.com",
    description="LED status indicator via Adafruit Pixel Trinkey + NeoPixels",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "led_indicator_node = led_indicator.led_indicator_node:main",
        ],
    },
)
