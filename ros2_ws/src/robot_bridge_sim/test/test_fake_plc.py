"""Protocol-level tests for the functional MC 3E PLC simulator."""

from __future__ import annotations

import threading

import pytest

from robot_bridge.mc_client import McClient, McConfig, words_to_dword
from robot_bridge_sim.fake_plc import DEVICE_D, FakePlcState, make_server


class _ManualClock:
    """Deterministic monotonic clock for pose/hold state tests."""

    def __init__(self, initial: float = 0.0) -> None:
        self.value = float(initial)

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += float(seconds)


def _axis_raw(state: FakePlcState) -> list[int]:
    words = state.read(DEVICE_D, 1002, 12)
    return [
        words_to_dword(words[index], words[index + 1])
        for index in range(0, 12, 2)
    ]


@pytest.fixture
def binary_plc():
    state = FakePlcState()
    server = make_server("127.0.0.1", 0, state=state, quiet=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address[:2]
    client = McClient(
        McConfig(
            host=str(host),
            port=int(port),
            protocol="binary",
            connect_timeout_s=1.0,
            read_timeout_s=1.0,
        )
    )
    client.connect()
    try:
        yield client, state
    finally:
        client.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_sim_starts_at_requested_pose_with_hold_and_no_client_write() -> None:
    clock = _ManualClock()
    state = FakePlcState(monotonic_fn=clock)
    expected_raw = [3856, 13625, -4948, 17, -8685, -5068]
    directions = [1, 1, 1, 1, 1, 1]
    expected_visual_degrees = [
        38.56,
        136.25,
        -49.48,
        0.17,
        -86.85,
        -50.68,
    ]

    assert _axis_raw(state) == expected_raw
    assert [
        raw * 0.01 * direction
        for raw, direction in zip(expected_raw, directions)
    ] == pytest.approx(expected_visual_degrees)
    assert state.read(DEVICE_D, 1100, 1)[0] & 0x0001
    assert state.hold is True
    assert state.phase_seconds == pytest.approx(0.0)
    assert state.write_count == 0

    clock.advance(5.0)

    assert _axis_raw(state) == expected_raw
    assert state.phase_seconds == pytest.approx(0.0)
    assert state.write_count == 0


def test_clearing_direct_hold_resumes_pose_without_position_jump() -> None:
    clock = _ManualClock()
    state = FakePlcState(monotonic_fn=clock)
    held_raw = _axis_raw(state)

    # The gateway SetHold service uses a batch WORD write after its masked
    # D1100 read-modify-write.  At the same monotonic instant, clearing bit 0
    # must release motion without changing the pose sample discontinuously.
    state.apply_client_write(DEVICE_D, 1100, [0], protocol="binary")

    assert state.hold is False
    assert state.read(DEVICE_D, 1100, 1)[0] & 0x0001 == 0
    assert state.phase_seconds == pytest.approx(0.0)
    assert _axis_raw(state) == held_raw
    assert state.write_count == 1

    clock.advance(0.25)

    resumed_raw = _axis_raw(state)
    assert state.phase_seconds == pytest.approx(0.25)
    assert resumed_raw != held_raw


def test_random_word_write_preserves_noncontiguous_gap_words(binary_plc) -> None:
    client, state = binary_plc
    state.write(DEVICE_D, 1017, [0xA117])
    state.write(DEVICE_D, 1019, [0xA119])

    client.write_random_words(
        [
            ("D1016", 25),
            ("D1018", 50),
            ("D1020", 75),
        ]
    )

    assert client.read_words("D1016", 5) == [25, 0xA117, 50, 0xA119, 75]
    assert state.write_count == 1
    assert state.write_log == [
        {
            "protocol": "binary",
            "command": "random_word_write",
            "writes": [
                {"device_code": DEVICE_D, "address": 1016, "value": 25},
                {"device_code": DEVICE_D, "address": 1018, "value": 50},
                {"device_code": DEVICE_D, "address": 1020, "value": 75},
            ],
        }
    ]


def test_d1100_raw_word_supports_generic_read_modify_write(binary_plc) -> None:
    client, state = binary_plc

    # Pretend another controller owns bit 5, then assert fault-reset bit 2
    # using the exact read/modify/write sequence required for a shared word.
    client.write_random_words({"D1100": 1 << 5})
    before = client.read_words("D1100", 1)[0]
    client.write_random_words({"D1100": before | (1 << 2)})

    assert client.read_words("D1100", 1) == [(1 << 5) | (1 << 2)]
    assert state.write_count == 2


def test_legacy_d3000_speed_command_behavior_is_unchanged(binary_plc) -> None:
    client, state = binary_plc

    client.write_words("D3000", [0, 1, 0])

    assert state.speed_percent == 50
    assert client.read_words("D1016", 5) == [0, 0, 50, 0, 0]
