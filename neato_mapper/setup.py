from glob import glob

from setuptools import setup

package_name = "neato_mapper"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="omkar",
    description="Neato LiDAR + drive base driver and SLAM bringup for Raspberry Pi",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lidar_node = neato_mapper.lidar_node:main",
            "diff_drive_node = neato_mapper.diff_drive_node:main",
            "neato_bringup = neato_mapper.bringup:main",
        ],
    },
)
