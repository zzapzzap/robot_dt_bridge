"""Runtime contract tests for split Hi6 pose and status scheduling."""

from __future__ import annotations

import threading
import time
from collections import Counter
from pathlib import Path

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_bridge import hi6_robot_node as node_module


class _RecordingClient:
    """In-memory replacement for the three persistent HTTP clients."""

    instances = []

    def __init__(self, _config) -> None:
        self.config = _config
        self.index = len(self.instances)
        self.calls = Counter()
        self.lock = threading.Lock()
        self.instances.append(self)

    def _record(self, name: str) -> None:
        with self.lock:
            self.calls[name] += 1

    def get_joint_positions(self, axis_count: int, *, mechinfo: int):
        self._record("pose")
        assert axis_count == 6
        assert mechinfo == 1
        return (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

    def get_api_version(self) -> int:
        self._record("api_version")
        return 5

    def get_status(self):
        self._record("status")
        time.sleep(0.04)
        return {
            "mode_code": 4,
            "motor_state_code": 0,
            "is_playback": False,
            "is_remote_mode": True,
            "playback_speed_percent": 50,
            "last_error_id": -1,
        }

    def get_motor_state(self) -> int:
        self._record("motor")
        time.sleep(0.04)
        return 0

    def get_emergency_stop(self) -> bool:
        self._record("emergency_stop")
        time.sleep(0.04)
        return False

    def get_playback_speed_percent(self) -> int:
        self._record("configured_speed")
        return 50

    def start(self):
        self._record("start")
        return {}

    def stop(self):
        self._record("stop")
        return {}

    def set_playback_speed_percent(self, percent: int):
        self._record(f"speed_{percent}")
        return {}

    def close(self) -> None:
        self._record("close")


class _RecordingPublisher:
    """Thread-safe publisher substitute retaining every emitted message."""

    def __init__(self) -> None:
        self.messages = []
        self.lock = threading.Lock()

    def publish(self, message) -> None:
        with self.lock:
            self.messages.append(message)


def _write_config(directory: Path) -> None:
    (directory / "hi6.yaml").write_text(
        """
defaults:
  pose_hz: 20
  status_hz: 5
  status_publish_hz: 20
  stale_timeout_ms: 500
robots:
  loading:
    host: 127.0.0.1
    allow_commands: true
    allow_speed_increase: true
    allow_start: true
    allow_unverified_start: true
""",
        encoding="utf-8",
    )
    (directory / "network.yaml").write_text(
        """
segments:
  hi6_control:
    hosts:
      jetson: 127.0.0.1
""",
        encoding="utf-8",
    )


def test_pose_and_cached_status_have_independent_rates(
    tmp_path: Path, monkeypatch
) -> None:
    """A slow three-GET status batch cannot block successful pose samples."""
    _write_config(tmp_path)
    monkeypatch.setenv("ROBOT_DT_CONFIG", str(tmp_path))
    _RecordingClient.instances = []
    monkeypatch.setattr(node_module, "Hi6Client", _RecordingClient)

    rclpy.init(args=[])
    node = None
    executor = None
    try:
        node = node_module.Hi6RobotNode()
        recorders = {
            "joint": _RecordingPublisher(),
            "pose": _RecordingPublisher(),
            "unity": _RecordingPublisher(),
            "status": _RecordingPublisher(),
        }
        node.pub_joint = recorders["joint"]
        node.pub_pose = recorders["pose"]
        node.pub_pose_unity = recorders["unity"]
        node.pub_status = recorders["status"]

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        deadline = time.monotonic() + 1.25
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert executor.shutdown(timeout_sec=3.0)
        executor = None

        assert len(_RecordingClient.instances) == 3
        pose_client, status_client, command_client = _RecordingClient.instances
        assert all(
            client.config.source_address == "127.0.0.1"
            for client in _RecordingClient.instances
        )
        pose_samples = pose_client.calls["pose"]
        status_samples = status_client.calls["status"]

        assert 20 <= pose_samples <= 30
        assert 4 <= status_samples <= 7
        assert status_client.calls["motor"] == status_samples
        assert status_client.calls["emergency_stop"] == status_samples
        assert status_client.calls["api_version"] == 1
        assert not command_client.calls

        assert len(recorders["joint"].messages) == pose_samples
        assert len(recorders["pose"].messages) == pose_samples
        assert len(recorders["unity"].messages) == pose_samples
        assert 20 <= len(recorders["status"].messages) <= 30

        for official, unity in zip(
            recorders["pose"].messages,
            recorders["unity"].messages,
        ):
            assert official is unity
            assert list(official.degrees) == [1, 2, 3, 4, 5, 6]

        statuses = recorders["status"].messages
        sequences = [message.sequence for message in statuses]
        assert sequences == sorted(sequences)
        # The final controller refresh may complete after the final 20 Hz
        # publisher tick while the executor is draining.
        assert status_samples - 1 <= max(sequences) <= status_samples
        assert len(set(sequences)) < len(sequences)

        repeated = {}
        for message in statuses:
            if message.sequence:
                repeated.setdefault(message.sequence, []).append(message)
        samples = next(
            values for values in repeated.values() if len(values) >= 2
        )
        assert samples[-1].age_sec > samples[0].age_sec
        assert all(message.fresh for message in samples)
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=3.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_unavailable_source_address_blocks_every_command_transport(
    tmp_path: Path, monkeypatch
) -> None:
    """Every command fails closed immediately before its write."""
    _write_config(tmp_path)
    monkeypatch.setenv("ROBOT_DT_CONFIG", str(tmp_path))
    _RecordingClient.instances = []
    monkeypatch.setattr(node_module, "Hi6Client", _RecordingClient)

    rclpy.init(args=[])
    node = None
    try:
        node = node_module.Hi6RobotNode()
        monkeypatch.setattr(
            node, "_command_source_is_available", lambda: False
        )

        stop = node.on_request_stop(
            node_module.RequestStop.Request(),
            node_module.RequestStop.Response(),
        )
        start = node.on_request_start(
            node_module.RequestStart.Request(),
            node_module.RequestStart.Response(),
        )
        speed_request = node_module.SetSpeedPercent.Request()
        speed_request.speed_percent = 25.0
        speed = node.on_set_speed_percent(
            speed_request,
            node_module.SetSpeedPercent.Response(),
        )

        for response in (stop, start, speed):
            assert response.accepted is False
            assert response.controller_ack is False
            assert response.confirmed is False
            assert response.error_code == "SOURCE_ADDRESS_UNAVAILABLE"

        command_calls = _RecordingClient.instances[2].calls
        assert command_calls["stop"] == 0
        assert command_calls["start"] == 0
        assert command_calls["speed_25"] == 0
        assert command_calls["close"] == 3
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_cached_status_freshness_ages_without_changing_sequence(
    monkeypatch,
) -> None:
    """Publishing cached state exposes age instead of inventing new samples."""
    class _ClockValue:
        def to_msg(self):
            return node_module.RobotStatus().header.stamp

    class _Clock:
        def now(self):
            return _ClockValue()

    class _Harness:
        _build_status_msg = node_module.Hi6RobotNode._build_status_msg
        _status_age_s = node_module.Hi6RobotNode._status_age_s
        _signal = staticmethod(node_module.Hi6RobotNode._signal)

    harness = _Harness()
    harness._snapshot_lock = threading.RLock()
    harness.snapshot = node_module._Snapshot(
        sequence=9,
        contact_wall_s=100.0,
        contact_monotonic_s=10.0,
        connection_state=node_module.RobotStatus.CONNECTION_CONNECTED,
        mode_code=4,
        motor_state_code=0,
        is_playback=False,
        is_remote_mode=True,
        speed_percent=50.0,
        emergency_stop=False,
    )
    harness.robot_id = "loading"
    harness.stale_timeout_s = 0.5
    harness.get_clock = _Clock

    now = {"value": 10.1}
    monkeypatch.setattr(
        node_module.time, "monotonic", lambda: now["value"]
    )
    fresh = harness._build_status_msg()
    now["value"] = 10.7
    stale = harness._build_status_msg()

    assert fresh.sequence == stale.sequence == 9
    assert fresh.age_sec == pytest.approx(0.1)
    assert stale.age_sec == pytest.approx(0.7)
    assert fresh.fresh is True
    assert stale.fresh is False
