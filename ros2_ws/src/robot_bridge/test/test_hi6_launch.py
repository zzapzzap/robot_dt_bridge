"""Focused launch-contract tests for direct Hi6 sessions."""

import importlib.util
import socket
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_DIR / "launch" / "hi6_bringup.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "hi6_bringup_test", LAUNCH_FILE
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_mock_session_clears_field_source_address(monkeypatch) -> None:
    """Local fake-controller traffic must not bind to the Jetson field NIC."""
    module = _load_launch_module()

    class _Robot:
        host = "192.168.250.21"
        rest_port = 8888
        pose_hz = 20.0
        status_hz = 5.0
        status_publish_hz = 20.0
        visualization_base_xyz_m = [0.0, 0.0, 0.0]
        visualization_base_rpy_rad = [0.0, 0.0, 0.0]

    class _Config:
        network = {"hosts": {"jetson": "192.168.250.10"}}

        def robot(self, _robot_id):
            return _Robot()

    monkeypatch.setattr(
        module.Hi6BridgeConfig,
        "load",
        classmethod(lambda _cls, _directory: _Config()),
    )
    monkeypatch.setattr(
        module,
        "_field_preflight",
        lambda *_args: pytest.fail("mock must not run field preflight"),
    )
    context = LaunchContext()
    context.launch_configurations.update({
        "instances": "loading",
        "config_dir": "",
        "use_mock": "true",
        "debug": "false",
        "allow_commands": "false",
        "allow_speed_increase": "false",
        "allow_start": "false",
        "allow_unverified_start": "false",
        "with_unity": "false",
        "mock_random_pose": "false",
        "mock_speed_readback_delay": "0",
        "mock_stop_readback_delay": "0",
    })

    node = next(
        action
        for action in module._launch_sessions(context)
        if isinstance(action, Node)
    )
    parameters = evaluate_parameters(context, node._Node__parameters)[0]
    assert parameters["host"] == "127.0.0.1"
    assert parameters["source_address"] == ""


def _command(context: LaunchContext, action: ExecuteProcess) -> list[str]:
    return [
        "".join(context.perform_substitution(part) for part in argument)
        for argument in action.cmd
    ]


def test_occupied_mock_port_is_rejected_before_session_construction() -> None:
    """A second sim cannot attach command nodes to an older mock listener."""
    module = _load_launch_module()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        port = listener.getsockname()[1]
        with pytest.raises(RuntimeError, match="already in use"):
            module._assert_mock_endpoints_available("127.0.0.1", [port])


def test_random_mock_pose_uses_30_hz_visualization_rates(
    monkeypatch,
) -> None:
    module = _load_launch_module()

    class _Robot:
        def __init__(self, robot_id: str) -> None:
            self.robot_id = robot_id

        pose_hz = 20.0
        status_hz = 5.0
        status_publish_hz = 20.0
        visualization_base_xyz_m = [0.0, 0.0, 0.0]
        visualization_base_rpy_rad = [0.0, 0.0, 0.0]

    loading = _Robot("loading")
    unloading = _Robot("unloading")

    class _Simulation:
        host = "127.0.0.2"
        port_base = 19888
        pose_hz = 31.0
        status_hz = 6.0
        status_publish_hz = 32.0

        @staticmethod
        def port_for_index(index: int) -> int:
            return 19888 + index

    class _Config:
        simulation = _Simulation()

        def enabled_instances(self):
            return [loading, unloading]

    monkeypatch.setattr(
        module.Hi6BridgeConfig,
        "load",
        classmethod(lambda _cls, _directory: _Config()),
    )
    context = LaunchContext()
    context.launch_configurations.update({
        "instances": "",
        "config_dir": "",
        "use_mock": "true",
        "debug": "true",
        "allow_commands": "false",
        "allow_speed_increase": "false",
        "allow_start": "false",
        "allow_unverified_start": "false",
        "with_unity": "true",
        "mock_random_pose": "true",
        "mock_speed_readback_delay": "0",
        "mock_stop_readback_delay": "0",
        "robot_description_file": "",
        "rviz_config_file": "",
    })

    actions = module._launch_sessions(context)
    mock_processes = [
        action for action in actions
        if isinstance(action, ExecuteProcess) and not isinstance(action, Node)
    ]
    assert len(mock_processes) == 2
    assert all("--random-pose" in _command(context, action)
               for action in mock_processes)
    commands = [_command(context, action) for action in mock_processes]
    assert all("127.0.0.2" in command for command in commands)
    assert any("19888" in command for command in commands)
    assert any("19889" in command for command in commands)

    robot_nodes = [
        action for action in actions
        if isinstance(action, Node)
        and action.node_executable == "hi6_robot_node"
    ]
    assert len(robot_nodes) == 2
    for node in robot_nodes:
        parameters = evaluate_parameters(context, node._Node__parameters)[0]
        assert parameters["pose_hz"] == 31.0
        assert parameters["status_hz"] == 6.0
        assert parameters["status_publish_hz"] == 32.0
        assert parameters["allow_commands"] is False
        assert parameters["allow_speed_increase"] is False
        assert parameters["allow_start"] is False
        assert parameters["allow_unverified_start"] is False

    state_publishers = [
        action for action in actions
        if isinstance(action, Node)
        and action.node_executable == "robot_state_publisher"
    ]
    assert len(state_publishers) == 2
    for node in state_publishers:
        parameters = evaluate_parameters(context, node._Node__parameters)[0]
        assert parameters["publish_frequency"] == 100.0

    unity_node = next(
        action for action in actions
        if isinstance(action, Node)
        and action.node_executable == "unity_adapter_node"
    )
    unity_parameters = evaluate_parameters(
        context, unity_node._Node__parameters
    )[0]
    assert unity_parameters["instances"] == "loading,unloading"
    assert unity_parameters["use_hi6_instances"] is True


def test_field_session_preserves_configured_20_5_20_rates(monkeypatch) -> None:
    module = _load_launch_module()

    class _Robot:
        host = "192.168.250.21"
        rest_port = 8888
        pose_hz = 20.0
        status_hz = 5.0
        status_publish_hz = 20.0
        visualization_base_xyz_m = [0.0, 0.0, 0.0]
        visualization_base_rpy_rad = [0.0, 0.0, 0.0]

    class _Config:
        network = {"hosts": {"jetson": "192.168.250.10"}}

        def robot(self, _robot_id):
            return _Robot()

    config = _Config()
    load_calls = []

    def load_config(_cls, directory):
        load_calls.append(directory)
        return config

    monkeypatch.setattr(
        module.Hi6BridgeConfig,
        "load",
        classmethod(load_config),
    )
    selected = []

    def preflight(_config, instance_ids, instances):
        selected.append((instance_ids, instances))
        return []

    monkeypatch.setattr(module, "_field_preflight", preflight)
    context = LaunchContext()
    context.launch_configurations.update({
        "instances": "loading,unloading",
        "config_dir": "",
        "use_mock": "false",
        "debug": "false",
        "allow_commands": "true",
        "allow_speed_increase": "true",
        "allow_start": "false",
        "allow_unverified_start": "false",
        "with_unity": "true",
        "mock_random_pose": "false",
        "mock_speed_readback_delay": "0",
        "mock_stop_readback_delay": "0",
    })

    robot_nodes = [
        action for action in module._launch_sessions(context)
        if isinstance(action, Node)
        and action.node_executable == "hi6_robot_node"
    ]
    assert len(robot_nodes) == 2
    assert load_calls == [None]
    assert selected and selected[0][0] == ["loading", "unloading"]
    for node in robot_nodes:
        parameters = evaluate_parameters(context, node._Node__parameters)[0]
        assert parameters["pose_hz"] == 20.0
        assert parameters["status_hz"] == 5.0
        assert parameters["status_publish_hz"] == 20.0
        assert parameters["host"] == "192.168.250.21"
        assert parameters["rest_port"] == 8888
        assert parameters["source_address"] == "192.168.250.10"


def test_field_preflight_failure_prevents_node_construction(
    monkeypatch,
) -> None:
    """No ROS node action may be constructed before field verification."""
    module = _load_launch_module()

    class _Robot:
        pose_hz = 20.0
        status_hz = 5.0
        status_publish_hz = 20.0
        visualization_base_xyz_m = [0.0, 0.0, 0.0]
        visualization_base_rpy_rad = [0.0, 0.0, 0.0]

    class _Config:
        def robot(self, _robot_id):
            return _Robot()

    monkeypatch.setattr(
        module.Hi6BridgeConfig,
        "load",
        classmethod(lambda _cls, _directory: _Config()),
    )

    def block_preflight(*_args):
        raise RuntimeError("preflight blocked")

    monkeypatch.setattr(module, "_field_preflight", block_preflight)
    monkeypatch.setattr(
        module,
        "Node",
        lambda *_args, **_kwargs: pytest.fail(
            "field Node was constructed before preflight"
        ),
    )
    context = LaunchContext()
    context.launch_configurations.update({
        "instances": "loading",
        "config_dir": "",
        "use_mock": "false",
        "debug": "false",
        "allow_commands": "true",
        "allow_speed_increase": "true",
        "allow_start": "false",
        "allow_unverified_start": "false",
        "with_unity": "false",
        "mock_random_pose": "false",
        "mock_speed_readback_delay": "0",
        "mock_stop_readback_delay": "0",
    })

    with pytest.raises(RuntimeError, match="preflight blocked"):
        module._launch_sessions(context)


def test_blank_selection_accepts_enabled_robots_compatibility_api() -> None:
    """The core works before and after the instances schema transition."""
    module = _load_launch_module()
    loading = SimpleNamespace(robot_id="loading")
    unloading = SimpleNamespace(robot_id="unloading")

    class _Config:
        def enabled_robots(self):
            return [loading, unloading]

    instance_ids, instances = module._select_instances(_Config(), "")

    assert instance_ids == ["loading", "unloading"]
    assert instances == [loading, unloading]


def test_field_preflight_verifies_exact_selected_instances(
    monkeypatch,
) -> None:
    """Every selected instance shares the bounded, source-bound preflight."""
    module = _load_launch_module()
    loading = SimpleNamespace(
        host="192.168.250.21",
        rest_port=8888,
        supported_api_versions=[5],
    )
    unloading = SimpleNamespace(
        host="192.168.250.22",
        rest_port=8888,
        supported_api_versions=[5],
    )
    config = SimpleNamespace(network={
        "subnet": "192.168.250.0/24",
        "hosts": {"jetson": "192.168.250.10"},
    })
    calls = []

    def preflight(endpoints, **kwargs):
        endpoints = tuple(endpoints)
        calls.append((endpoints, kwargs))
        return SimpleNamespace(
            configured_results=tuple(
                SimpleNamespace(
                    endpoint=endpoint,
                    status=module.Hi6ProbeStatus.VERIFIED,
                    api_version=5,
                    controller_version="60.34-00",
                )
                for endpoint in endpoints
            ),
            discovered_results=(),
        )

    monkeypatch.setattr(module, "_local_ipv4_is_assigned", lambda _ip: True)
    monkeypatch.setattr(module, "preflight_hi6_connections", preflight)

    actions = module._field_preflight(
        config,
        ["loading", "unloading"],
        [loading, unloading],
    )

    assert len(actions) == 2
    endpoints, kwargs = calls[0]
    assert [(item.host, item.port) for item in endpoints] == [
        ("192.168.250.21", 8888),
        ("192.168.250.22", 8888),
    ]
    assert kwargs["scan_subnet"] == "192.168.250.0/24"
    assert kwargs["source_address"] == "192.168.250.10"
    assert kwargs["supported_api_versions"] == [5]


def test_field_preflight_rejects_partial_selected_set(monkeypatch) -> None:
    """One healthy controller cannot start nodes for a partial field set."""
    module = _load_launch_module()
    instances = [
        SimpleNamespace(
            host="192.168.250.21",
            rest_port=8888,
            supported_api_versions=[5],
        ),
        SimpleNamespace(
            host="192.168.250.22",
            rest_port=8888,
            supported_api_versions=[5],
        ),
    ]
    config = SimpleNamespace(network={
        "subnet": "192.168.250.0/24",
        "hosts": {"jetson": "192.168.250.10"},
    })

    def preflight(endpoints, **_kwargs):
        endpoints = tuple(endpoints)
        return SimpleNamespace(
            configured_results=(
                SimpleNamespace(
                    endpoint=endpoints[0],
                    status=module.Hi6ProbeStatus.VERIFIED,
                    api_version=5,
                    controller_version="60.34-00",
                ),
                SimpleNamespace(
                    endpoint=endpoints[1],
                    status=module.Hi6ProbeStatus.UNREACHABLE,
                ),
            ),
            discovered_results=(),
        )

    monkeypatch.setattr(module, "_local_ipv4_is_assigned", lambda _ip: True)
    monkeypatch.setattr(module, "preflight_hi6_connections", preflight)

    with pytest.raises(RuntimeError, match="unloading=unreachable"):
        module._field_preflight(
            config,
            ["loading", "unloading"],
            instances,
        )
