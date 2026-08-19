"""Minimal public launch for Robot <-> PLC <-> Jetson.

Only two operator choices are exposed.  The safe visual default is
``sim:=true debug:=true``.  ``sim:=false`` must be requested explicitly for the
MELSEC PLC.  ``debug`` only adds TF/RViz and never changes write gates.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _as_bool(name: str, value: str) -> bool:
    if value == "true":
        return True
    if value == "false":
        return False
    raise ValueError(f"{name} must be true or false, got {value!r}")


def _include_plc_launch(context: LaunchContext):
    sim = _as_bool("sim", LaunchConfiguration("sim").perform(context))
    debug = _as_bool("debug", LaunchConfiguration("debug").perform(context))
    launch_file = (
        Path(get_package_share_directory("robot_bridge"))
        / "launch"
        / "plc_bringup.launch.py"
    )
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)),
        launch_arguments={
            "profile": "sim" if sim else "field",
            "debug": str(debug).lower(),
            "with_unity": "true",
        }.items(),
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "sim",
            default_value="true",
            choices=["true", "false"],
            description="use a localhost PLC with the same register contract",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="true",
            choices=["true", "false"],
            description="add RViz and namespaced robot TF visualization",
        ),
        OpaqueFunction(function=_include_plc_launch),
    ])
