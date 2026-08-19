import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

import robot_bridge_sim.fake_hi6 as fake_hi6
from robot_bridge_sim.fake_hi6 import (
    RANDOM_POSE_BOUNDS_DEG,
    FakeHi6State,
    make_server,
)


class ManualClock:
    def __init__(self, initial=0.0):
        self.value = float(initial)

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += float(seconds)


def request_json(base_url, path, method="GET", body=None):
    raw = None if body is None else json.dumps(body).encode("utf-8")
    req = Request(base_url + path, data=raw, method=method)
    if raw is not None:
        req.add_header("Content-Type", "application/json; charset=utf-8")
    with urlopen(req, timeout=2.0) as response:
        return response.status, json.loads(response.read().decode("utf-8"))


def error_json(base_url, path, method="GET", body=None):
    with pytest.raises(HTTPError) as caught:
        request_json(base_url, path, method=method, body=body)
    error = caught.value
    return error.code, json.loads(error.read().decode("utf-8"))


@pytest.fixture
def controller():
    state = FakeHi6State(robot_id="loading")
    server = make_server("127.0.0.1", 0, state=state, quiet=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address[:2]
    try:
        yield f"http://{host}:{port}", state
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_read_only_endpoints_do_not_record_writes(controller):
    base_url, state = controller

    assert request_json(base_url, "/api_ver") == (200, 5)

    _, versions = request_json(base_url, "/versions/sysver")
    assert versions["modules"][0]["name"] == "com"
    assert versions["modules"][0]["ver"] == "60.34-00"

    _, pose = request_json(base_url, "/project/robot/po_cur?crd=2&mechinfo=1")
    assert pose["crd"] == "joint"
    assert pose["mechinfo"] == 1
    assert [pose[f"j{i}"] for i in range(1, 7)] == state.joints

    _, rgen = request_json(base_url, "/project/rgen")
    assert rgen["is_remote_mode"] == 1
    assert rgen["is_playback"] == 0
    assert rgen["auto_spd"] == 100

    _, op_cnd = request_json(base_url, "/project/control/op_cnd")
    assert op_cnd["_type"] == "CondGrp"
    assert op_cnd["playback_spd_rate"] == 100

    assert request_json(base_url, "/project/robot/motor_on_state")[1][
        "val"
    ] == 0
    assert request_json(base_url, "/project/robot/emergency_stop")[1][
        "val"
    ] == 0
    assert state.write_count == 0
    assert state.write_log == []


def test_persistent_mock_transport_disables_nagle_delay():
    """Keep localhost visualization polling above the requested 30 Hz."""
    assert fake_hi6.FakeHi6Handler.disable_nagle_algorithm is True


def test_random_pose_get_changes_smoothly_within_visual_bounds():
    clock = ManualClock(100.0)
    state = FakeHi6State(
        robot_id="loading",
        random_pose=True,
        random_seed=17,
        monotonic_fn=clock,
    )
    server = make_server("127.0.0.1", 0, state=state, quiet=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address[:2]
    base_url = f"http://{host}:{port}"

    try:
        assert state.is_playback is True
        assert state.write_count == 0
        assert state.write_log == []
        _, first = request_json(
            base_url,
            "/project/robot/po_cur?crd=2&mechinfo=1",
        )
        previous = [first[f"j{axis}"] for axis in range(1, 7)]
        all_samples = [previous]

        for _ in range(80):
            clock.advance(0.1)
            _, pose = request_json(
                base_url,
                "/project/robot/po_cur?crd=2&mechinfo=1",
            )
            current = [pose[f"j{axis}"] for axis in range(1, 7)]
            assert current == state.joints
            # The analytic trajectory has no request-to-request random jumps.
            assert max(abs(a - b) for a, b in zip(current, previous)) < 2.0
            all_samples.append(current)
            previous = current

        assert any(
            abs(all_samples[-1][axis] - all_samples[0][axis]) > 0.1
            for axis in range(6)
        )
        for sample in all_samples:
            for value, (low, high) in zip(sample, RANDOM_POSE_BOUNDS_DEG):
                assert low <= value <= high
        assert state.write_count == 0
        assert state.write_log == []
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_random_pose_is_deterministic_and_robot_defaults_are_distinct():
    clock_a = ManualClock(10.0)
    clock_b = ManualClock(10.0)
    loading_a = FakeHi6State(
        robot_id="loading",
        random_pose=True,
        monotonic_fn=clock_a,
    )
    loading_b = FakeHi6State(
        robot_id="loading",
        random_pose=True,
        monotonic_fn=clock_b,
    )
    unloading_clock = ManualClock(10.0)
    unloading = FakeHi6State(
        robot_id="unloading",
        random_pose=True,
        monotonic_fn=unloading_clock,
    )

    for delta in (0.0, 0.25, 3.0, 20.0):
        clock_a.advance(delta)
        clock_b.advance(delta)
        unloading_clock.advance(delta)
        assert loading_a.pose() == loading_b.pose()

    assert loading_a.pose() != unloading.pose()

    seeded_loading = FakeHi6State(
        robot_id="loading",
        random_pose=True,
        random_seed=1234,
        monotonic_fn=ManualClock(0.0),
    )
    seeded_unloading = FakeHi6State(
        robot_id="unloading",
        random_pose=True,
        random_seed=1234,
        monotonic_fn=ManualClock(0.0),
    )
    assert seeded_loading.pose() == seeded_unloading.pose()


def test_random_pose_starts_playing_without_a_write_and_pauses_smoothly():
    clock = ManualClock()
    state = FakeHi6State(
        robot_id="loading",
        random_pose=True,
        monotonic_fn=clock,
    )

    assert state.is_playback is True
    assert state.write_count == 0
    assert state.write_log == []

    clock.advance(2.0)
    moving_pose = state.pose()
    with state.lock:
        state.defer_stop_readback()
    stopped_pose = state.pose()
    assert stopped_pose == moving_pose

    clock.advance(30.0)
    assert state.pose() == stopped_pose

    with state.lock:
        state.start_playback()
    assert state.pose() == stopped_pose
    clock.advance(1.0)
    assert state.pose() != stopped_pose
    assert state.write_count == 0


def test_random_pose_speed_service_changes_visual_motion_rate_smoothly():
    """A 50% mock speed request halves pose phase without a discontinuity."""
    slow_clock = ManualClock()
    reference_clock = ManualClock()
    slow = FakeHi6State(
        robot_id="loading",
        random_pose=True,
        random_seed=99,
        monotonic_fn=slow_clock,
    )
    reference = FakeHi6State(
        robot_id="loading",
        random_pose=True,
        random_seed=99,
        monotonic_fn=reference_clock,
    )
    server = make_server("127.0.0.1", 0, state=slow, quiet=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address[:2]
    base_url = f"http://{host}:{port}"

    try:
        slow_clock.advance(1.0)
        reference_clock.advance(1.0)
        before = slow.pose()
        assert before == reference.pose()

        assert request_json(
            base_url,
            "/project/control/op_cnd",
            "PUT",
            {"playback_spd_rate": 50},
        ) == (200, {"_text": ""})
        assert slow.pose() == before

        # Two wall-clock seconds at 50% equal one phase second at 100%.
        slow_clock.advance(2.0)
        reference_clock.advance(1.0)
        assert slow.pose() == reference.pose()
        assert slow.rgen()["auto_spd"] == 50
        assert slow.write_count == 1
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_random_pose_cli_options_are_forwarded(monkeypatch):
    captured = {}

    def fake_serve(host, port, robot_id, **kwargs):
        captured.update(host=host, port=port, robot_id=robot_id, **kwargs)

    monkeypatch.setattr(fake_hi6, "serve", fake_serve)
    fake_hi6.main(
        [
            "--host",
            "0.0.0.0",
            "--port",
            "28888",
            "--robot-id",
            "unloading",
            "--random-pose",
            "--random-seed",
            "42",
        ]
    )

    assert captured["random_pose"] is True
    assert captured["random_seed"] == 42


def test_start_stop_and_speed_update_have_observable_readback(controller):
    base_url, state = controller

    assert request_json(base_url, "/project/robot/start", "POST", {}) == (
        200,
        {"_type": "JObject"},
    )
    assert request_json(base_url, "/project/rgen")[1]["is_playback"] == 1

    assert request_json(
        base_url,
        "/project/control/op_cnd",
        "PUT",
        {"playback_spd_rate": 50},
    ) == (200, {"_text": ""})
    assert request_json(base_url, "/project/control/op_cnd")[1][
        "playback_spd_rate"
    ] == 50
    assert request_json(base_url, "/project/rgen")[1]["auto_spd"] == 50

    assert request_json(base_url, "/project/robot/stop", "POST", {}) == (
        200,
        {"_type": "JObject"},
    )
    assert request_json(base_url, "/project/rgen")[1]["is_playback"] == 0
    assert state.write_count == 3
    assert [entry["path"] for entry in state.write_log] == [
        "/project/robot/start",
        "/project/control/op_cnd",
        "/project/robot/stop",
    ]


def test_speed_readback_delay_supports_confirmation_fault_injection(
    controller,
):
    base_url, state = controller
    with state.lock:
        state.speed_readback_delay_s = 0.1

    request_json(
        base_url,
        "/project/control/op_cnd",
        "PUT",
        {"playback_spd_rate": 50},
    )

    assert request_json(base_url, "/project/control/op_cnd")[1][
        "playback_spd_rate"
    ] == 50
    assert request_json(base_url, "/project/rgen")[1]["auto_spd"] == 100
    time.sleep(0.15)
    assert request_json(base_url, "/project/rgen")[1]["auto_spd"] == 50


def test_stop_readback_delay_supports_repeated_stop_tests(controller):
    base_url, state = controller
    with state.lock:
        state.is_playback = True
        state.stop_readback_delay_s = 0.1

    request_json(base_url, "/project/robot/stop", "POST", {})
    assert request_json(base_url, "/project/rgen")[1]["is_playback"] == 1
    time.sleep(0.15)
    assert request_json(base_url, "/project/rgen")[1]["is_playback"] == 0


@pytest.mark.parametrize("bad_value", [0, 101, "50", True, None])
def test_invalid_speed_is_rejected_atomically(controller, bad_value):
    base_url, state = controller

    status, body = error_json(
        base_url,
        "/project/control/op_cnd",
        "PUT",
        {"playback_spd_rate": bad_value},
    )
    assert status == 400
    assert body["_text"]
    assert request_json(base_url, "/project/control/op_cnd")[1][
        "playback_spd_rate"
    ] == 100
    assert state.write_count == 0


def test_start_requires_remote_mode(controller):
    base_url, state = controller
    with state.lock:
        state.remote_mode = False

    status, body = error_json(base_url, "/project/robot/start", "POST", {})
    assert status == 403
    assert body["error_code"] == -38500
    assert request_json(base_url, "/project/rgen")[1]["is_playback"] == 0
    assert state.write_count == 0


def test_unknown_endpoint_and_bad_json(controller):
    base_url, state = controller

    assert error_json(base_url, "/not-an-api")[0] == 404

    req = Request(
        base_url + "/project/control/op_cnd",
        data=b"not-json",
        method="PUT",
        headers={"Content-Type": "application/json"},
    )
    with pytest.raises(HTTPError) as caught:
        urlopen(req, timeout=2.0)
    assert caught.value.code == 400
    assert state.write_count == 0


def test_concurrent_reads_and_writes_keep_state_valid(controller):
    base_url, state = controller
    speeds = [25, 50, 75, 100] * 5

    def update(speed):
        return request_json(
            base_url,
            "/project/control/op_cnd",
            "PUT",
            {"playback_spd_rate": speed},
        )[0]

    def read(_):
        return request_json(base_url, "/project/rgen")[0]

    with ThreadPoolExecutor(max_workers=8) as pool:
        statuses = list(pool.map(update, speeds)) + list(
            pool.map(read, range(20))
        )

    assert statuses == [200] * 40
    assert request_json(base_url, "/project/control/op_cnd")[1][
        "playback_spd_rate"
    ] in {25, 50, 75, 100}
    assert state.write_count == len(speeds)
