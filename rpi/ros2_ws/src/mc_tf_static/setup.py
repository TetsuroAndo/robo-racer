from setuptools import setup

package_name = "mc_tf_static"

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
    description="Static TF publisher for base_link to laser",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mc_tf_static = mc_tf_static.static_tf_node:main",
        ],
    },
)
