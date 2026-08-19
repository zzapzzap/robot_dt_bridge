"""Unit tests for the photographed PLC command-register controls.

These tests construct the gateway without calling ``Node.__init__`` and use an
in-memory MC client.  They must never open a socket or write to a real PLC.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from builtin_interfaces.msg import Time

from robot_bridge.config_loader import PlcBridgeConfig
from robot_bridge.plc_gateway_node import PlcGatewayNode, _Runtime, _Snapshot
from robot_bridge_msgs import srv as bridge_services
from robot_bridge_msgs.msg import RobotStatus
from robot_bridge_msgs.srv import SetSpeedPercent, TriggerRobotAction


PACKAGE_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_CONFIG = PACKAGE_DIR.parents[2] / "config"


class _Logger:
    def info(self, *_args, **_kwargs) -> None:
        pass

    def warn(self, *_args, **_kwargs) -> None:
        pass

    def error(self, *_args, **_kwargs) -> None:
        pass


class _RecordingPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class _NoWaitEvent:
    """Event-shaped test double which records pulse time without sleeping."""

    def __init__(self) -> None:
        self.waits: list[float] = []

    def is_set(self) -> bool:
        return False

    def wait(self, timeout: float) -> bool:
        self.waits.append(float(timeout))
        return False


class _MemoryClient:
    """Small word-memory client used only by masked-write unit tests."""

    def __init__(self, control_word: int = 0) -> None:
        self.connected = True
        self.words = {"D1100": int(control_word) & 0xFFFF}
        self.word_writes: list[tuple[str, list[int]]] = []

    def read_words(self, device: str, count: int) -> list[int]:
        assert count == 1
        return [self.words[str(device)]]

    def write_words(self, device: str, values) -> None:
        copied = [int(value) & 0xFFFF for value in values]
        assert len(copied) == 1
        self.word_writes.append((str(device), copied))
        self.words[str(device)] = copied[0]


def _clock():
    return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))


def _gateway(
    *,
    profile: str = "sim",
    allow_field_control_writes: bool = False,
    control_word: int = 0,
) -> tuple[PlcGatewayNode, _Runtime, _MemoryClient]:
    """Build a socket-free gateway object with one fresh loading snapshot."""
    config = PlcBridgeConfig.load(str(REPOSITORY_CONFIG), profile=profile)
    instance = config.instance("loading")
    runtime = _Runtime(
        instance=instance,
        snapshot=_Snapshot(
            sequence=7,
            contact_monotonic_s=time.monotonic(),
            connection_state=RobotStatus.CONNECTION_CONNECTED,
            emergency_stop=False,
            control_word_raw=int(control_word) & 0xFFFF,
        ),
    )
    client = _MemoryClient(control_word)

    gateway = PlcGatewayNode.__new__(PlcGatewayNode)
    gateway.profile = profile
    gateway.allow_field_control_writes = allow_field_control_writes
    gateway.cfg = config
    gateway.allowed_speeds = tuple(config.allowed_speed_percent)
    gateway.stale_timeout_s = float(config.stale_timeout_ms) / 1000.0
    gateway.verify_timeout_s = float(config.verify_timeout_s)
    gateway.runtimes = {"loading": runtime}
    gateway.client = client
    gateway._transport_lock = threading.RLock()
    gateway._stop_generation_lock = threading.Lock()
    gateway._stop_generation = 0
    gateway._next_connect_monotonic_s = 0.0
    gateway._shutting_down = _NoWaitEvent()
    # Instance attributes deliberately replace Node methods so no rclpy
    # context, executor, timer, publisher, or socket is created.
    gateway._is_running = lambda: True
    gateway.get_clock = _clock
    gateway.get_logger = _Logger
    gateway._refresh_all = lambda: False
    return gateway, runtime, client


def test_50_percent_service_writes_only_exact_no9_to_no11_devices() -> None:
    """Map 50 percent to D1018 without touching either gap WORD."""
    gateway, _runtime, _client = _gateway()
    captured = []

    def capture(writes, *, stop_generation=None) -> bool:
        captured.append((list(writes), stop_generation))
        return True

    gateway._write_random_command = capture
    gateway._fresh_feedback = lambda _robot_id, *, refresh: True
    # Pretend the independent re-read matched all three command registers.
    gateway._verify = lambda *_args, **_kwargs: True

    request = SetSpeedPercent.Request()
    request.speed_percent = 50.0
    request.source = "unit-test"
    response = gateway.on_set_speed_percent(
        "loading", request, SetSpeedPercent.Response()
    )

    assert captured == [
        ([('D1016', 0), ('D1018', 50), ('D1020', 0)], 0)
    ]
    assert response.accepted is True
    assert response.controller_ack is True
    # A same-register echo is never promoted to robot actual-speed feedback.
    assert response.confirmed is False
    assert response.actual.actual_speed_valid is False
    assert response.error_code == "ACTUAL_FEEDBACK_UNAVAILABLE"


def test_field_speed_write_is_rejected_until_explicit_runtime_opt_in() -> None:
    """Keep field writes disabled unless the launch-time opt-in is true."""
    gateway, _runtime, _client = _gateway(
        profile="field", allow_field_control_writes=False
    )
    writes = []
    gateway._write_random_command = (
        lambda values, **_kwargs: writes.append(values)
    )

    request = SetSpeedPercent.Request()
    request.speed_percent = 50.0
    response = gateway.on_set_speed_percent(
        "loading", request, SetSpeedPercent.Response()
    )

    assert response.accepted is False
    assert response.controller_ack is False
    assert response.confirmed is False
    assert response.error_code == "FIELD_WRITE_OPT_IN_REQUIRED"
    assert writes == []


def test_speed_request_requires_write_access_to_all_three_registers() -> None:
    """Reject a request if clearing either unselected WORD is unauthorized."""
    gateway, runtime, _client = _gateway()
    controls = replace(
        runtime.instance.direct_controls,
        writable_controls=frozenset({"speed_50"}),
    )
    registers = replace(
        runtime.instance.registers,
        direct_controls=controls,
    )
    runtime.instance = replace(runtime.instance, registers=registers)
    writes = []
    gateway._write_random_command = (
        lambda values, **_kwargs: writes.append(values)
    )

    request = SetSpeedPercent.Request()
    request.speed_percent = 50.0
    response = gateway.on_set_speed_percent(
        "loading", request, SetSpeedPercent.Response()
    )

    assert response.accepted is False
    assert response.error_code == "CONTROL_NOT_WRITABLE"
    assert writes == []


def test_hold_masked_write_preserves_every_other_d1100_bit() -> None:
    """Change only Hold bit zero during a D1100 read-modify-write."""
    # bit 1 is deliberately ON, along with several unrelated high bits.
    before = 0xA55A
    gateway, runtime, client = _gateway(control_word=before)

    assert gateway._write_control_bit(runtime, "hold", True) == (True, True)
    asserted = before | 0x0001
    assert client.words["D1100"] == asserted
    assert client.words["D1100"] & 0x0002
    assert (client.words["D1100"] & 0xFFFE) == (before & 0xFFFE)

    assert gateway._write_control_bit(runtime, "hold", False) == (True, True)
    assert client.words["D1100"] == before
    assert client.word_writes == [
        ("D1100", [asserted]),
        ("D1100", [before]),
    ]


def test_trigger_action_asserts_then_clears_configured_bit() -> None:
    """Emit a bounded action pulse and restore the initial unrelated bits."""
    # Retain an unrelated bit throughout the fault-reset pulse.  Stop/E-stop
    # bit 1 starts clear, which is required by the service precondition.
    before = 1 << 9
    gateway, _runtime, client = _gateway(control_word=before)

    request = TriggerRobotAction.Request()
    request.action = request.ACTION_FAULT_RESET
    request.source = "unit-test"
    response = gateway.on_trigger_action(
        "loading", request, TriggerRobotAction.Response()
    )

    asserted = before | (1 << 2)
    assert client.word_writes == [
        ("D1100", [asserted]),
        ("D1100", [before]),
    ]
    assert client.words["D1100"] == before
    assert gateway._shutting_down.waits == [0.25]
    assert response.accepted is True
    assert response.controller_ack is True
    assert response.register_readback is True


def test_estop_has_no_write_service_or_trigger_action() -> None:
    """Expose no network E-stop action and reject its internal control name."""
    assert not hasattr(bridge_services, "SetEmergencyStop")
    assert not hasattr(bridge_services, "RequestEmergencyStop")
    assert not hasattr(
        TriggerRobotAction.Request, "ACTION_EMERGENCY_STOP"
    )

    gateway, runtime, _client = _gateway()
    assert gateway._direct_control_policy_error(
        runtime, "emergency_stop"
    )[0] == "ESTOP_READ_ONLY"


def test_control_state_keeps_raw_words_and_uses_exact_active_values() -> None:
    """Distinguish exact active values from arbitrary nonzero register data."""
    gateway, runtime, _client = _gateway()
    with runtime.lock:
        runtime.snapshot.speed_down_1_raw = 25
        runtime.snapshot.speed_down_2_raw = 1
        runtime.snapshot.speed_down_3_raw = 75
        runtime.snapshot.control_word_raw = 0b101011
        runtime.snapshot.hold = True
        runtime.snapshot.emergency_stop = True
        runtime.snapshot.fault_reset = False
        runtime.snapshot.device_home = True
        runtime.snapshot.robot_home = False
        runtime.snapshot.standby = True

    state = gateway._build_control_state("loading")

    assert state.fresh is True
    assert (
        state.slowdown_25_raw,
        state.slowdown_50_raw,
        state.slowdown_75_raw,
    ) == (25, 1, 75)
    # Nonzero is not enough: the raw word must equal the documented value.
    assert state.slowdown_25_active is True
    assert state.slowdown_50_active is False
    assert state.slowdown_75_active is True
    assert state.control_word_raw == 0b101011
    assert (
        state.hold,
        state.emergency_stop,
        state.fault_reset,
        state.device_home,
        state.robot_home,
        state.standby,
    ) == (True, True, False, True, False, True)


def test_controller_pose_and_visual_joint_state_are_decoupled() -> None:
    """Apply CAD calibration only to RViz here, in sim and field."""
    controller = [38.56, 136.25, -49.48, 0.17, -86.85, -50.68]
    expected_visual = [-38.56, 46.25, 49.48, -0.17, -86.85, 50.68]
    raw = [3856, 13625, -4948, 17, -8685, -5068]

    for profile in ("sim", "field"):
        gateway, runtime, _client = _gateway(profile=profile)
        runtime.pub_memory = _RecordingPublisher()
        runtime.pub_pose = _RecordingPublisher()
        runtime.pub_pose_unity = _RecordingPublisher()
        runtime.pub_joint = _RecordingPublisher()
        snapshot = _Snapshot(
            sequence=8,
            raw_axes=list(raw),
            degrees=list(controller),
        )

        gateway._publish_sample(runtime, snapshot, Time())

        controller_pose = runtime.pub_pose.messages[-1]
        unity_input = runtime.pub_pose_unity.messages[-1]
        joint_state = runtime.pub_joint.messages[-1]

        assert list(controller_pose.degrees) == controller
        assert list(controller_pose.raw) == raw
        assert unity_input is controller_pose
        assert [
            round(math.degrees(value), 2)
            for value in joint_state.position
        ] == expected_visual
