"""
Bring up isolated direct Hi6 sessions.

Safe defaults launch a localhost mock in read-only mode.  A field controller
is only contacted after ``use_mock:=false`` and every selected fixed instance
passes the shared read-only connection preflight.  Controller writes
additionally require ``allow_commands:=true`` and remote start has its own
``allow_start:=true`` gate.
"""

import ipaddress
import os
import shlex
import socket
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

from robot_bridge.config_loader import Hi6Config as Hi6BridgeConfig
from robot_bridge.hi6_connection import (
    Hi6ConnectionConfigError,
    Hi6Endpoint,
    Hi6ProbeStatus,
    preflight_hi6_connections,
)


def _as_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError(
        f"invalid boolean launch value {value!r}; use true or false"
    )


def _instance_id(instance) -> str:
    """Return the canonical id across the current and transitional schemas."""
    for name in ("instance_id", "robot_id", "id"):
        value = str(getattr(instance, name, "") or "").strip()
        if value:
            return value
    raise ValueError("configured Hi6 instance has no non-empty id")


def _configured_instance(config, instance_id: str):
    """Resolve one instance while accepting the former robot() API."""
    getter = getattr(config, "instance", None)
    if callable(getter):
        return getter(instance_id)
    return config.robot(instance_id)


def _enabled_instances(config) -> list:
    """Read enabled instances, accepting the former robot API."""
    getter = getattr(config, "enabled_instances", None)
    if not callable(getter):
        getter = getattr(config, "enabled_robots", None)
    if not callable(getter):
        raise ValueError("Hi6 config has no enabled instance selector")
    instances = list(getter())
    if not instances:
        raise ValueError("Hi6 config has no enabled instances")
    return instances


def _select_instances(config, text: str) -> tuple[list[str], list]:
    """Select explicit ids, or all enabled instances for blank input."""
    requested = [item.strip() for item in text.split(",") if item.strip()]
    if len(set(requested)) != len(requested):
        raise ValueError("instances contains duplicate ids")
    if requested:
        return requested, [
            _configured_instance(config, instance_id)
            for instance_id in requested
        ]
    instances = _enabled_instances(config)
    instance_ids = [_instance_id(instance) for instance in instances]
    if len(set(instance_ids)) != len(instance_ids):
        raise ValueError("enabled Hi6 instances contain duplicate ids")
    return instance_ids, instances


def _simulation_value(config, name: str, default):
    """Read the optional simulation object/mapping during schema migration."""
    simulation = getattr(config, "simulation", None)
    if simulation is None:
        return default
    if isinstance(simulation, Mapping):
        return simulation.get(name, default)
    return getattr(simulation, name, default)


def _simulation_port(config, index: int, port_base: int) -> int:
    """Use the schema helper when present, else retain port-base behavior."""
    simulation = getattr(config, "simulation", None)
    port_for_index = getattr(simulation, "port_for_index", None)
    if callable(port_for_index):
        return int(port_for_index(index))
    return port_base + index


def _local_ipv4_is_assigned(address: str) -> bool:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind((address, 0))
        return True
    except OSError:
        return False


def _assert_mock_endpoints_available(host: str, ports: list[int]) -> None:
    """Reject a second sim before nodes can attach to stale mock servers."""
    for port in ports:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                probe.bind((host, port))
        except OSError as exc:
            raise RuntimeError(
                f"Hi6 mock endpoint {host}:{port} is already in use; "
                "stop the previous simulation before launching another"
            ) from exc


def _field_preflight(config, instance_ids: list[str], instances: list) -> list:
    """Verify every selected fixed instance before creating any ROS action."""
    endpoints = [
        Hi6Endpoint(instance.host, instance.rest_port)
        for instance in instances
    ]
    supported_versions = sorted({
        version
        for instance in instances
        for version in instance.supported_api_versions
    })
    subnet = str(config.network.get("subnet") or "").strip() or None
    network_hosts = config.network.get("hosts") or {}
    local_control_ip = (
        str(network_hosts.get("jetson") or "").strip()
        if isinstance(network_hosts, Mapping)
        else ""
    )
    try:
        control_network = ipaddress.ip_network(subnet, strict=True)
        source_ip = ipaddress.ip_address(local_control_ip)
        controller_ips = [
            ipaddress.ip_address(endpoint.host) for endpoint in endpoints
        ]
    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            "Hi6 control addresses must be literal IPv4 values in a valid "
            "subnet"
        ) from exc
    if not isinstance(control_network, ipaddress.IPv4Network):
        raise RuntimeError("Hi6 control subnet must be IPv4")
    if not isinstance(source_ip, ipaddress.IPv4Address):
        raise RuntimeError("Hi6 Jetson control address must be IPv4")
    if any(
        not isinstance(value, ipaddress.IPv4Address)
        for value in controller_ips
    ):
        raise RuntimeError("Hi6 controller addresses must be IPv4")
    if source_ip not in control_network or any(
        value not in control_network for value in controller_ips
    ):
        raise RuntimeError(
            "Jetson and every selected Hi6 controller address must belong to "
            f"{control_network}"
        )
    if len({
        (value, endpoint.port)
        for value, endpoint in zip(controller_ips, endpoints)
    }) != len(endpoints):
        raise RuntimeError(
            "selected Hi6 instances must use different IP:port endpoints"
        )
    if not _local_ipv4_is_assigned(str(source_ip)):
        raise RuntimeError(
            "Hi6 control network is not ready: Jetson address "
            f"{source_ip} is not assigned to a local NIC"
        )

    try:
        result = preflight_hi6_connections(
            endpoints,
            scan_subnet=str(control_network),
            source_address=str(source_ip),
            supported_api_versions=supported_versions,
            connect_timeout_s=0.15,
            read_timeout_s=0.5,
            scan_workers=32,
        )
    except Hi6ConnectionConfigError as exc:
        raise RuntimeError(
            f"Hi6 preflight configuration is invalid: {exc}"
        ) from exc

    by_endpoint = {
        item.endpoint: item for item in result.configured_results
    }
    actions = []
    failed = []
    for instance_id, endpoint in zip(instance_ids, endpoints):
        probe = by_endpoint[endpoint]
        if probe.status is Hi6ProbeStatus.VERIFIED:
            actions.append(LogInfo(msg=(
                f"Hi6 connection {instance_id}: verified "
                f"{endpoint.host}:{endpoint.port} "
                f"(API v{probe.api_version}, controller "
                f"{probe.controller_version})"
            )))
        else:
            failed.append((instance_id, probe))

    discovered = [
        item.endpoint
        for item in result.discovered_results
        if item.verified and item.endpoint not in endpoints
    ]
    if failed:
        failure_text = ", ".join(
            f"{instance_id}={probe.status.value}"
            for instance_id, probe in failed
        )
        candidate_text = ", ".join(
            f"{item.host}:{item.port}" for item in discovered
        )
        suffix = (
            f"; unassigned candidate(s): {candidate_text}"
            if candidate_text
            else ""
        )
        raise RuntimeError(
            "every selected fixed Hi6 instance must pass read-only "
            f"preflight ({failure_text}){suffix}"
        )
    if discovered:
        candidates = ", ".join(
            f"{item.host}:{item.port}" for item in discovered
        )
        actions.append(LogInfo(msg=(
            "WARNING: unassigned Hi6 candidate(s) found: " + candidates
            + "; instance roles are never guessed—set fixed IPs first"
        )))
    return actions


def _launch_sessions(context: LaunchContext):
    instances_text = LaunchConfiguration("instances").perform(context)
    config_dir = LaunchConfiguration("config_dir").perform(context)
    use_mock = _as_bool(LaunchConfiguration("use_mock").perform(context))
    debug = _as_bool(LaunchConfiguration("debug").perform(context))
    allow_commands = _as_bool(
        LaunchConfiguration("allow_commands").perform(context)
    )
    allow_speed_increase = _as_bool(
        LaunchConfiguration("allow_speed_increase").perform(context)
    )
    allow_start = _as_bool(LaunchConfiguration("allow_start").perform(context))
    allow_unverified_start = _as_bool(
        LaunchConfiguration("allow_unverified_start").perform(context)
    )
    with_unity = _as_bool(LaunchConfiguration("with_unity").perform(context))
    mock_random_pose = _as_bool(
        LaunchConfiguration("mock_random_pose").perform(context)
    )
    if mock_random_pose and not use_mock:
        raise ValueError("mock_random_pose requires use_mock:=true")
    mock_speed_delay_text = LaunchConfiguration(
        "mock_speed_readback_delay"
    ).perform(context)
    mock_stop_delay_text = LaunchConfiguration(
        "mock_stop_readback_delay"
    ).perform(context)
    try:
        mock_speed_delay = float(mock_speed_delay_text)
        mock_stop_delay = float(mock_stop_delay_text)
    except ValueError as exc:
        raise ValueError(
            "mock readback delays must be non-negative numbers"
        ) from exc
    if mock_speed_delay < 0.0 or mock_stop_delay < 0.0:
        raise ValueError(
            "mock readback delays must be non-negative numbers"
        )

    bridge_config = Hi6BridgeConfig.load(config_dir or None)
    robot_ids, instances = _select_instances(bridge_config, instances_text)
    robot_description = None
    rviz_config_path = None
    if debug:
        package_share = Path(get_package_share_directory("robot_bridge"))
        description_text = LaunchConfiguration(
            "robot_description_file"
        ).perform(context).strip()
        description_path = (
            Path(description_text).expanduser().resolve()
            if description_text
            else package_share / "urdf" / "ys080_hh050_debug.urdf.xacro"
        )
        if not description_path.is_file():
            raise FileNotFoundError(
                f"robot description file does not exist: {description_path}"
            )
        rviz_text = LaunchConfiguration("rviz_config_file").perform(
            context
        ).strip()
        rviz_config_path = (
            Path(rviz_text).expanduser().resolve()
            if rviz_text
            else package_share / "rviz" / "hi6_debug.rviz"
        )
        if not rviz_config_path.is_file():
            raise FileNotFoundError(
                f"RViz config file does not exist: {rviz_config_path}"
            )
        robot_description = ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                shlex.quote(str(description_path)),
            ]),
            value_type=str,
        )

    actions = (
        []
        if use_mock
        else _field_preflight(bridge_config, robot_ids, instances)
    )
    mock_host = str(_simulation_value(
        bridge_config, "host", "127.0.0.1"
    ))
    mock_port_base = int(_simulation_value(
        bridge_config, "port_base", 18888
    ))
    mock_pose_hz = float(_simulation_value(
        bridge_config, "pose_hz", 30.0
    ))
    mock_status_hz = float(_simulation_value(
        bridge_config, "status_hz", 5.0
    ))
    mock_status_publish_hz = float(_simulation_value(
        bridge_config, "status_publish_hz", 30.0
    ))
    mock_ports = []
    if use_mock:
        mock_ports = [
            _simulation_port(bridge_config, index, mock_port_base)
            for index in range(len(robot_ids))
        ]
        if len(set(mock_ports)) != len(mock_ports):
            raise RuntimeError("Hi6 mock instances require unique ports")
        _assert_mock_endpoints_available(mock_host, mock_ports)
    field_source_address = ""
    if not use_mock:
        network_hosts = bridge_config.network.get("hosts") or {}
        field_source_address = (
            str(network_hosts.get("jetson") or "").strip()
            if isinstance(network_hosts, Mapping)
            else ""
        )
    for index, (robot_id, robot_config) in enumerate(zip(
        robot_ids, instances
    )):
        parameters = {
            "robot_id": robot_id,
            "config_dir": config_dir,
            "pose_hz": robot_config.pose_hz,
            "status_hz": robot_config.status_hz,
            "status_publish_hz": robot_config.status_publish_hz,
            "allow_commands": allow_commands,
            "allow_speed_increase": allow_speed_increase,
            "allow_start": allow_start,
            "allow_unverified_start": allow_unverified_start,
        }
        if not use_mock:
            # Pin the exact endpoint and source address that just passed
            # preflight.  A config edit between launch construction and child
            # startup must not redirect a command-capable node elsewhere.
            parameters.update({
                "host": robot_config.host,
                "rest_port": robot_config.rest_port,
                "source_address": field_source_address,
            })
        if use_mock:
            port = mock_ports[index]
            mock_cmd = [
                "ros2", "run", "robot_bridge_sim", "fake_hi6",
                "--host", mock_host,
                "--port", str(port),
                "--robot-id", robot_id,
                "--speed-readback-delay", str(mock_speed_delay),
                "--stop-readback-delay", str(mock_stop_delay),
                "--quiet",
            ]
            if mock_random_pose:
                mock_cmd.append("--random-pose")
            actions.append(
                ExecuteProcess(
                    cmd=mock_cmd,
                    name=f"fake_hi6_{robot_id}",
                    output="screen",
                    on_exit=Shutdown(
                        reason=(
                            f"fake Hi6 {robot_id} exited; stopping the "
                            "simulation instead of using another listener"
                        )
                    ),
                )
            )
            parameters.update({
                "host": mock_host,
                "rest_port": port,
                # Never bind localhost mock traffic to the field control NIC.
                "source_address": "",
                "pose_hz": mock_pose_hz,
                "status_hz": mock_status_hz,
                "status_publish_hz": mock_status_publish_hz,
            })

        actions.append(
            Node(
                package="robot_bridge",
                executable="hi6_robot_node",
                name=f"hi6_{robot_id}",
                output="screen",
                parameters=[parameters],
            )
        )

        if debug:
            namespace = f"/robot/{robot_id}"
            actions.append(
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace=namespace,
                    name="state_publisher",
                    output="screen",
                    parameters=[{
                        "robot_description": robot_description,
                        "frame_prefix": f"{robot_id}/",
                        # This is only a maximum throttle.  Keep it above both
                        # field (20 Hz) and simulation (30 Hz) inputs so every
                        # received joint sample can reach RViz TF.
                        "publish_frequency": 100.0,
                    }],
                    remappings=[
                        ("joint_states", namespace + "/joint_states"),
                    ],
                )
            )
            xyz = robot_config.visualization_base_xyz_m
            rpy = robot_config.visualization_base_rpy_rad
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"{robot_id}_debug_base_tf",
                    output="screen",
                    arguments=[
                        "--x", str(xyz[0]),
                        "--y", str(xyz[1]),
                        "--z", str(xyz[2]),
                        "--roll", str(rpy[0]),
                        "--pitch", str(rpy[1]),
                        "--yaw", str(rpy[2]),
                        "--frame-id", "world",
                        "--child-frame-id", f"{robot_id}/base_link",
                    ],
                )
            )

    if with_unity:
        actions.append(
            Node(
                package="robot_bridge",
                executable="unity_adapter_node",
                name="unity_adapter",
                output="screen",
                parameters=[{
                    "profile": "sim",
                    "config_dir": config_dir,
                    "instances": ",".join(robot_ids),
                    "forward_unity_commands": False,
                    "use_hi6_instances": True,
                }],
            )
        )
        try:
            get_package_share_directory("ros_tcp_endpoint")
        except PackageNotFoundError:
            actions.append(LogInfo(msg=(
                "Unity pose topics are enabled, but ros_tcp_endpoint is not "
                "installed; RViz still works and Unity TCP will remain offline"
            )))
        else:
            actions.append(
                Node(
                    package="ros_tcp_endpoint",
                    executable="default_server_endpoint",
                    name="unity_tcp_endpoint",
                    output="screen",
                    parameters=[{
                        "ROS_IP": "0.0.0.0",
                        "ROS_TCP_PORT": 10000,
                    }],
                )
            )

    if debug:
        gui_available = bool(
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        )
        if gui_available:
            actions.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="hi6_debug_rviz",
                    output="screen",
                    arguments=["-d", str(rviz_config_path)],
                )
            )
        else:
            actions.append(LogInfo(msg=(
                "debug TF/RobotModel publishers enabled, but RViz was skipped "
                "because DISPLAY/WAYLAND_DISPLAY is not set"
            )))

    mode = "mock" if use_mock else "field"
    write_mode = "enabled" if allow_commands else "read-only"
    actions.insert(0, LogInfo(msg=f"Hi6 {mode}: {robot_ids} ({write_mode})"))
    if debug:
        actions.insert(1, LogInfo(msg=(
            "Hi6 RViz debug uses the YS080/HH050 candidate CAD model; "
            "joint signs/home offsets require field FAT"
        )))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "instances",
            default_value="",
            description=(
                "comma-separated instance ids; empty selects every enabled "
                "instance from config/hi6.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "config_dir",
            default_value="",
            description=(
                "config directory; empty uses package/repository discovery"
            ),
        ),
        DeclareLaunchArgument(
            "use_mock",
            default_value="true",
            description=(
                "launch localhost fake Hi6 controllers instead of field "
                "hardware"
            ),
        ),
        DeclareLaunchArgument(
            "mock_random_pose",
            default_value="false",
            choices=["true", "false"],
            description=(
                "make localhost mock poses change for visualization testing"
            ),
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description=(
                "launch robot_state_publisher, namespaced TF, and RViz when "
                "GUI is available"
            ),
        ),
        DeclareLaunchArgument(
            "robot_description_file",
            default_value="",
            description=(
                "optional URDF/xacro override; empty uses packaged "
                "YS080/HH050 debug model"
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="",
            description="optional RViz config override",
        ),
        DeclareLaunchArgument(
            "allow_commands",
            default_value="false",
            description="enable normal stop and playback-speed write services",
        ),
        DeclareLaunchArgument(
            "allow_speed_increase",
            default_value="false",
            description="allow less restrictive playback-speed changes",
        ),
        DeclareLaunchArgument(
            "allow_start",
            default_value="false",
            description="separately enable remote start service",
        ),
        DeclareLaunchArgument(
            "allow_unverified_start",
            default_value="false",
            description=(
                "accept missing protective-stop/current-fault readback for "
                "start"
            ),
        ),
        DeclareLaunchArgument(
            "with_unity",
            default_value="false",
            description="launch the read-only Unity visualization adapter",
        ),
        DeclareLaunchArgument(
            "mock_speed_readback_delay",
            default_value="0.0",
            description="test-only delay before mock rgen reports a speed PUT",
        ),
        DeclareLaunchArgument(
            "mock_stop_readback_delay",
            default_value="0.0",
            description="test-only delay before mock rgen reports a stop",
        ),
        OpaqueFunction(function=_launch_sessions),
    ])
