"""Safety and visualization contract for the PLC bring-up launch."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

from robot_bridge.config_loader import PlcBridgeConfig
from robot_bridge.mc_client import McConfig


PACKAGE_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_CONFIG = PACKAGE_DIR.parents[2] / "config"
LAUNCH_FILE = PACKAGE_DIR / "launch" / "plc_bringup.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "plc_bringup_launch_test", LAUNCH_FILE
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_field_profile_is_confirmed_endpoint_but_write_locked() -> None:
    config = PlcBridgeConfig.load(str(REPOSITORY_CONFIG), profile="field")

    assert config.connection.host == "192.168.10.30"
    assert config.connection.port == 9000
    assert config.connection.source_address == "192.168.10.61"
    assert config.connection.frame == "3E"
    assert config.connection.protocol == "binary"
    assert config.poll_hz == 20.0
    assert config.commissioned is False
    assert config.commands_enabled is False
    assert config.start_enabled is False


def test_uncommissioned_field_map_can_pass_read_only_preflight(
    monkeypatch,
) -> None:
    module = _load_launch_module()
    calls = []

    class _ReadOnlyClient:
        def __init__(self, connection):
            calls.append(("init", connection.host, connection.port))

        def connect(self):
            calls.append(("connect",))

        def read_words(self, device, count):
            calls.append(("read", device, count))
            return [0] * count

        def close(self):
            calls.append(("close",))

    instance = SimpleNamespace(
        registers=SimpleNamespace(read_head="D1000", read_words=21)
    )
    config = SimpleNamespace(
        commissioned=False,
        connection=McConfig(
            host="192.168.10.30",
            port=9000,
            source_address="192.168.10.61",
        ),
        enabled_instances=lambda: [instance],
    )
    monkeypatch.setattr(module, "McClient", _ReadOnlyClient)

    action = module._field_preflight(config)

    assert calls == [
        ("init", "192.168.10.30", 9000),
        ("connect",),
        ("read", "D1000", 21),
        ("close",),
    ]
    assert "UNCOMMISSIONED" in perform_substitutions(
        LaunchContext(), action.msg
    )


def test_debug_adds_joint_state_consumers_and_rviz(monkeypatch) -> None:
    module = _load_launch_module()
    monkeypatch.setenv("DISPLAY", ":0")
    instance = SimpleNamespace(
        visualization_base_xyz_m=[0.0, 0.0, 0.0],
        visualization_base_rpy_rad=[0.0, 0.0, 0.0],
    )
    config = SimpleNamespace(instance=lambda _robot_id: instance)

    actions = module._debug_actions(config, PACKAGE_DIR, ["loading"])
    nodes = [action for action in actions if isinstance(action, Node)]

    assert [node.node_executable for node in nodes] == [
        "robot_state_publisher",
        "static_transform_publisher",
        "rviz2",
    ]
    state_publisher = nodes[0]
    context = LaunchContext()
    remappings = [
        (
            perform_substitutions(context, list(source)),
            perform_substitutions(context, list(target)),
        )
        for source, target in state_publisher._Node__remappings
    ]
    assert ("joint_states", "/robot/loading/joint_states") in remappings
