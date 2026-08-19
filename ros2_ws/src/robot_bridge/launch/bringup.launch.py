"""Deprecated compatibility wrapper for the PLC gateway launch.

New operator commands should use ``robot_system.launch.py``.  This wrapper
keeps the former ``profile:=sim|field`` invocation without reviving the unsafe
legacy per-robot writer.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_file = (
        Path(get_package_share_directory("robot_bridge"))
        / "launch"
        / "plc_bringup.launch.py"
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="sim",
            choices=["sim", "field"],
            description="PLC profile (sim or field)",
        ),
        DeclareLaunchArgument(
            "config_dir",
            default_value="",
            description="config directory; empty uses package/repository config",
        ),
        LogInfo(msg=(
            "bringup.launch.py is deprecated; use robot_system.launch.py "
            "sim:=true|false"
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file)),
            launch_arguments={
                "profile": LaunchConfiguration("profile"),
                "config_dir": LaunchConfiguration("config_dir"),
                "debug": "false",
                "with_unity": "true",
            }.items(),
        ),
    ])
