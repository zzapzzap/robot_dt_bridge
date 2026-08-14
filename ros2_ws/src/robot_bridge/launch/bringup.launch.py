"""브리지 일괄 기동.

    # PLC 없이 (내장 시뮬레이터 동시 기동)
    ros2 launch robot_bridge bringup.launch.py profile:=sim

    # 현장
    ros2 launch robot_bridge bringup.launch.py profile:=field
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    profile = LaunchConfiguration("profile")
    robots = LaunchConfiguration("robots")
    config_dir = LaunchConfiguration("config_dir")
    with_sim = PythonExpression(["'true' if '", profile, "' == 'sim' else 'false'"])

    args = [
        DeclareLaunchArgument("profile", default_value="sim",
                              description="plc.yaml 의 프로파일 (sim | field)"),
        DeclareLaunchArgument("robots", default_value="loading",
                              description="기동할 로봇 id 를 쉼표로 구분"),
        DeclareLaunchArgument("config_dir", default_value="",
                              description="config 디렉터리 경로 (비우면 자동 탐색)"),
    ]

    common = [{"profile": profile}, {"config_dir": config_dir}]

    # 시뮬레이터 — profile 이 sim 일 때만
    sim = GroupAction(
        condition=IfCondition(with_sim),
        actions=[
            LogInfo(msg="[sim] 가상 PLC 를 127.0.0.1:5010 에 기동합니다"),
            Node(package="robot_bridge_sim", executable="fake_plc_node",
                 name="fake_plc", output="screen", parameters=common),
            Node(package="robot_bridge_sim", executable="fake_worker_node",
                 name="fake_worker", output="screen", parameters=common),
        ],
    )

    # 로봇별 브리지 — 현재는 loading 1대 기준. 언로딩 활성화 시 인자로 추가.
    bridge = Node(
        package="robot_bridge", executable="robot_memory_node",
        name="robot_memory_loading", output="screen",
        parameters=common + [{"robot_id": "loading"}],
    )

    adapter = Node(
        package="robot_bridge", executable="unity_adapter_node",
        name="unity_adapter", output="screen", parameters=common,
    )

    return LaunchDescription(args + [
        LogInfo(msg=["프로파일 : ", profile, " · 로봇 : ", robots]),
        sim, bridge, adapter,
    ])
