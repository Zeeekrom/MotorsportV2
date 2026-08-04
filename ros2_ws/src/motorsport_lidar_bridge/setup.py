from glob import glob
import os
from setuptools import find_packages, setup


package_name = "motorsport_lidar_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MotorSport Team",
    maintainer_email="motorsport@example.com",
    description="CARLA LiDAR, cone-map, centre-path, and control bridge",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "carla_lidar_bridge = motorsport_lidar_bridge.carla_lidar_bridge:main",
        ],
    },
)
