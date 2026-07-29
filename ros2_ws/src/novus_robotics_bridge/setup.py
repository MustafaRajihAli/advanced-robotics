from setuptools import find_packages, setup

package_name = "novus_robotics_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Novus AI Dynamics",
    maintainer_email="codedizer@gmail.com",
    description="Bridges ROS 2 topics (nav, arm, sensors) to the advanced_robotics Python app layer",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = novus_robotics_bridge.bridge_node:main",
        ],
    },
)
