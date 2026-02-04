from setuptools import setup

package_name = "lidar_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Bridge SHM lidar_scan to ROS2 LaserScan.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lidar_bridge = lidar_bridge.bridge_node:main",
        ],
    },
)
