from setuptools import find_packages, setup

package_name = "robot_bridge_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sierra Base",
    maintainer_email="jhlee@sierrabase.co.kr",
    description="PLC · 작업자 pose 시뮬레이터",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "fake_plc_node = robot_bridge_sim.fake_plc_node:main",
            "fake_worker_node = robot_bridge_sim.fake_worker_node:main",
        ],
    },
)
