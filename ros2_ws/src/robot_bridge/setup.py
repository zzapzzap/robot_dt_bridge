import os
from glob import glob
from setuptools import find_packages, setup

package_name = "robot_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # 저장소 루트의 config/ 를 패키지 share 로 복사해 배포본에서도 동작하게 한다
        (os.path.join("share", package_name, "config"),
         glob(os.path.join("..", "..", "..", "config", "*.yaml"))),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Sierra Base",
    maintainer_email="jhlee@sierrabase.co.kr",
    description="로봇 ↔ PLC ↔ 디지털 트윈 브리지",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_memory_node = robot_bridge.robot_memory_node:main",
            "unity_adapter_node = robot_bridge.unity_adapter_node:main",
            "mode_cli = robot_bridge.mode_cli:main",
        ],
    },
)
