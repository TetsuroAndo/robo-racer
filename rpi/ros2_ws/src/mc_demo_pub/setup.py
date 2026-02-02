from setuptools import setup

package_name = "mc_demo_pub"

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
    maintainer_email="tetsuro997@gmail.com",
    description="Demo LaserScan publisher for RViz",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mc_demo_pub = mc_demo_pub.demo_pub_node:main",
        ],
    },
)
