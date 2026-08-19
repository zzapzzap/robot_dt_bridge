"""Bring up the production Robot <-> PLC <-> Jetson data path.

The Jetson owns one MC Protocol client session to one process PLC.  Robot
instances are register-map roles inside that PLC, not separate IP endpoints.
Field startup performs one read-only preflight before the gateway process is
created.  An uncommissioned register map may be visualized for bring-up, but
its command services remain locked by the profile policy.
"""

import os
import shlex
from collections.abc import Mapping
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from robot_bridge.config_loader import PlcBridgeConfig
from robot_bridge.mc_client import McClient, McConfig


def _as_bool(name: str, value: str) -> bool:
    normalized = value.strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    raise ValueError(f"{name} must be true or false, got {value!r}")


def _value(obj, name: str, default=None):
    if isinstance(obj, Mapping):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _instance_id(instance) -> str:
    for name in ("instance_id", "robot_id", "id"):
        value = str(_value(instance, name, "") or "").strip()
        if value:
            return value
    raise ValueError("configured PLC instance has no id")


def _connection_dict(connection) -> dict:
    if isinstance(connection, Mapping):
        return dict(connection)
    fields = McConfig.__dataclass_fields__
    return {
        name: getattr(connection, name)
        for name in fields
        if hasattr(connection, name)
    }


def _field_preflight(config) -> LogInfo:
    instances = list(config.enabled_instances())
    if not instances:
        raise RuntimeError("PLC config has no enabled robot instance")
    first = instances[0]
    registers = _value(first, "registers", first)
    read_head = str(_value(registers, "read_head", ""))
    read_words = int(_value(registers, "read_words", 0))
    if not read_head or read_words <= 0:
        raise RuntimeError("PLC instance read map is incomplete")

    client = McClient(McConfig.from_dict(_connection_dict(config.connection)))
    try:
        client.connect()
        words = client.read_words(read_head, read_words)
    except (OSError, RuntimeError) as exc:
        raise RuntimeError(
            "PLC read-only preflight failed; no gateway or write service was "
            f"started: {exc}"
        ) from exc
    finally:
        client.close()
    endpoint = config.connection
    commissioning_note = (
        "commissioned map"
        if bool(config.commissioned)
        else "UNCOMMISSIONED map: visualization only; values/calibration "
             "are not validated"
    )
    return LogInfo(msg=(
        "PLC read-only preflight verified "
        f"{_value(endpoint, 'host')}:{_value(endpoint, 'port')} "
        f"({read_head} x {len(words)}; {commissioning_note})"
    ))


def _debug_actions(config, package_share: Path, robot_ids: list[str]):
    description_path = package_share / "urdf" / "ys080_hh050_debug.urdf.xacro"
    rviz_path = package_share / "rviz" / "hi6_debug.rviz"
    if not description_path.is_file() or not rviz_path.is_file():
        raise FileNotFoundError(
            "packaged PLC debug URDF/RViz assets are missing"
        )
    description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            shlex.quote(str(description_path)),
        ]),
        value_type=str,
    )
    actions = []
    for robot_id in robot_ids:
        instance = config.instance(robot_id)
        xyz = list(_value(instance, "visualization_base_xyz_m", [0, 0, 0]))
        rpy = list(_value(instance, "visualization_base_rpy_rad", [0, 0, 0]))
        namespace = f"/robot/{robot_id}"
        actions.extend([
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=namespace,
                name="state_publisher",
                output="screen",
                parameters=[{
                    "robot_description": description,
                    "frame_prefix": f"{robot_id}/",
                    "publish_frequency": 100.0,
                }],
                remappings=[("joint_states", namespace + "/joint_states")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{robot_id}_plc_debug_base_tf",
                output="screen",
                arguments=[
                    "--x", str(xyz[0]), "--y", str(xyz[1]),
                    "--z", str(xyz[2]), "--roll", str(rpy[0]),
                    "--pitch", str(rpy[1]), "--yaw", str(rpy[2]),
                    "--frame-id", "world",
                    "--child-frame-id", f"{robot_id}/base_link",
                ],
            ),
        ])
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="plc_debug_rviz",
            output="screen",
            arguments=["-d", str(rviz_path)],
        ))
    else:
        actions.append(LogInfo(msg=(
            "debug TF publishers enabled; RViz GUI skipped because no "
            "DISPLAY/WAYLAND_DISPLAY is available"
        )))
    return actions


def _launch(context: LaunchContext):
    profile = LaunchConfiguration("profile").perform(context).strip()
    config_dir = LaunchConfiguration("config_dir").perform(context).strip()
    debug = _as_bool("debug", LaunchConfiguration("debug").perform(context))
    with_unity = _as_bool(
        "with_unity", LaunchConfiguration("with_unity").perform(context)
    )
    allow_field_control_writes = _as_bool(
        "allow_field_control_writes",
        LaunchConfiguration("allow_field_control_writes").perform(context),
    )
    if profile not in ("sim", "field"):
        raise ValueError("profile must be sim or field")

    config = PlcBridgeConfig.load(config_dir or None, profile=profile)
    instances = list(config.enabled_instances())
    if not instances:
        raise RuntimeError("PLC config has no enabled robot instance")
    robot_ids = [_instance_id(item) for item in instances]
    package_share = Path(get_package_share_directory("robot_bridge"))
    actions = []

    if profile == "field":
        actions.append(_field_preflight(config))
    else:
        endpoint = config.connection
        actions.append(ExecuteProcess(
            cmd=[
                "ros2", "run", "robot_bridge_sim", "fake_plc",
                "--host", str(_value(endpoint, "host")),
                "--port", str(_value(endpoint, "port")),
            ],
            name="fake_plc",
            output="screen",
            on_exit=Shutdown(
                reason="fake PLC exited; stopping instead of using another listener"
            ),
        ))

    actions.append(Node(
        package="robot_bridge",
        executable="plc_gateway_node",
        name="plc_gateway",
        output="screen",
        parameters=[{
            "profile": profile,
            "config_dir": config_dir,
            "allow_field_control_writes": allow_field_control_writes,
        }],
    ))

    if with_unity:
        actions.append(Node(
            package="robot_bridge",
            executable="unity_adapter_node",
            name="unity_adapter",
            output="screen",
            parameters=[{
                "profile": profile,
                "config_dir": config_dir,
                "robots": ",".join(robot_ids),
                "forward_unity_commands": False,
                "use_hi6_instances": False,
                "use_plc_instances": True,
            }],
        ))
        try:
            get_package_share_directory("ros_tcp_endpoint")
        except PackageNotFoundError:
            actions.append(LogInfo(msg=(
                "ROS Unity topics are enabled, but ros_tcp_endpoint is not "
                "installed; Unity TCP remains offline"
            )))
        else:
            actions.append(Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                name="unity_tcp_endpoint",
                output="screen",
                parameters=[{"ROS_IP": "0.0.0.0", "ROS_TCP_PORT": 10000}],
            ))

    if debug:
        actions.extend(_debug_actions(config, package_share, robot_ids))

    write_mode = (
        "No.9-17 control services enabled"
        if profile == "sim" or allow_field_control_writes
        else "No.9-17 field writes locked"
    )
    actions.insert(0, LogInfo(msg=(
        f"PLC {profile}: {robot_ids} through one MC endpoint; {write_mode}"
    )))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="sim",
            choices=["sim", "field"],
            description="PLC profile from config/plc.yaml",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            choices=["true", "false"],
            description="add robot TF and RViz without changing write policy",
        ),
        DeclareLaunchArgument(
            "with_unity",
            default_value="true",
            choices=["true", "false"],
            description="publish the ROS-side Unity adapter topics",
        ),
        DeclareLaunchArgument(
            "allow_field_control_writes",
            default_value="false",
            choices=["true", "false"],
            description=(
                "explicitly enable allowlisted No.9-17 field writes; ignored "
                "for automatic/startup behavior"
            ),
        ),
        DeclareLaunchArgument(
            "config_dir",
            default_value="",
            description="config directory; empty uses package/repository config",
        ),
        OpaqueFunction(function=_launch),
    ])
