from setuptools import setup

package_name = "mc_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robo Racer",
    maintainer_email="dev@example.com",
    description="Robo Racer mc_bridge skeleton",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mc_bridge = mc_bridge.bridge_node:main",
        ],
    },
)
