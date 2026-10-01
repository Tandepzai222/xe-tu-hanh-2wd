from setuptools import setup
from glob import glob
import os

package_name = "xe_tu_hanh"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sinh vien",
    maintainer_email="sv@example.com",
    description="Cau noi ROS 2 cho xe tu hanh 2 banh vi sai (ESP32 qua WiFi UDP)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "udp_bridge = xe_tu_hanh.udp_bridge:main",
        ],
    },
)
