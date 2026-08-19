"""
Small, dependency-free Hi6 Open API simulator.

The simulator implements only the endpoints used by the direct robot bridge.
It is deliberately an HTTP server rather than a ROS node so client code can be
tested before ROS or a physical controller is available.

Run it directly with::

    python3 -m robot_bridge_sim.fake_hi6 --port 18888 --robot-id loading

Add ``--random-pose`` for a smooth, bounded visualisation trajectory.  Its
default trajectory is stable and distinct per robot id; ``--random-seed`` can
override that seed when an exact trajectory needs to be reproduced.

The default state is remote mode, motor on, stopped, emergency stop released,
and playback speed 100 %.  Random-pose mode starts in playback state so its
motion and status agree.  Starting either mode never records a write.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlsplit


DEFAULT_JOINTS = [0.0, 90.0, 0.0, 0.0, -90.0, 0.0]

# Deliberately stay well inside the debug URDF joint limits.  These are visual
# motion envelopes in controller degrees, not physical-cell motion limits.
RANDOM_POSE_BOUNDS_DEG: Tuple[Tuple[float, float], ...] = (
    (-25.0, 25.0),
    (75.0, 105.0),
    (-20.0, 20.0),
    (-30.0, 30.0),
    (-105.0, -75.0),
    (-35.0, 35.0),
)


def _stable_unit(seed_material: bytes, label: str) -> float:
    """Return a process-independent value in [0, 1) for one profile field."""
    digest = hashlib.sha256(
        b"fake-hi6-random-pose-v1\0"
        + seed_material
        + b"\0"
        + label.encode("ascii")
    ).digest()
    return int.from_bytes(digest[:8], "big") / float(1 << 64)


def _random_pose_profile(
    robot_id: str,
    random_seed: Optional[int],
) -> Tuple[Tuple[float, float, float, float], ...]:
    """Build two smooth harmonic components for every joint axis."""
    if random_seed is None:
        seed_material = b"robot-id:" + robot_id.encode("utf-8")
    else:
        seed_material = f"seed:{int(random_seed)}".encode("ascii")

    profile = []
    for axis in range(6):
        # Slow periods make the movement easy to inspect in RViz and Unity.
        slow_hz = 0.025 + 0.025 * _stable_unit(
            seed_material, f"{axis}:slow-hz"
        )
        fast_hz = 0.060 + 0.060 * _stable_unit(
            seed_material, f"{axis}:fast-hz"
        )
        slow_phase = math.tau * _stable_unit(
            seed_material, f"{axis}:slow-phase"
        )
        fast_phase = math.tau * _stable_unit(
            seed_material, f"{axis}:fast-phase"
        )
        profile.append(
            (math.tau * slow_hz, slow_phase, math.tau * fast_hz, fast_phase)
        )
    return tuple(profile)


def _default_op_cnd() -> Dict[str, int]:
    return {
        "step_goback_max_spd": 200,
        "playback_mode": 1,
        "step_go_func_ex": 1,
        "robot_lock": 0,
        "playback_spd_rate": 100,
        "intp_base": 0,
        "ucrd_num": 0,
        "path_recov_confirm": 1,
        "func_reexe_on_trace": 1,
        "plc_mode": 1,
    }


@dataclass
class FakeHi6State:
    """Thread-safe state shared by all HTTP request handlers."""

    robot_id: str = "loading"
    api_version: int = 5
    system_version: str = "60.34-00"
    remote_mode: bool = True
    motor_state: int = 0  # Hi6 API: 0=on, 1=off, 2=busy
    emergency_stop: int = 0  # 0=released, 1=pressed
    is_playback: bool = False
    joints: List[float] = field(default_factory=lambda: list(DEFAULT_JOINTS))
    op_cnd: Dict[str, int] = field(default_factory=_default_op_cnd)
    speed_readback_delay_s: float = 0.0
    stop_readback_delay_s: float = 0.0
    reported_speed_percent: Optional[int] = None
    random_pose: bool = False
    random_seed: Optional[int] = None
    write_count: int = 0
    write_log: List[Dict[str, Any]] = field(default_factory=list)
    monotonic_fn: Callable[[], float] = field(
        default=time.monotonic,
        repr=False,
        compare=False,
    )
    lock: threading.RLock = field(default_factory=threading.RLock, repr=False)
    _speed_generation: int = field(default=0, repr=False)
    _stop_generation: int = field(default=0, repr=False)
    _motion_elapsed_s: float = field(default=0.0, init=False, repr=False)
    _motion_started_at_s: Optional[float] = field(
        default=None,
        init=False,
        repr=False,
    )
    _pose_profile: Tuple[Tuple[float, float, float, float], ...] = field(
        default=(),
        init=False,
        repr=False,
    )

    def __post_init__(self) -> None:
        if not self.random_pose:
            return

        self._pose_profile = _random_pose_profile(
            self.robot_id,
            self.random_seed,
        )
        # In visual simulation mode the moving pose should not be reported as
        # STOPPED.  This is initial fake state, not a controller write.
        self.is_playback = True
        self._motion_started_at_s = float(self.monotonic_fn())

    def _motion_time_locked(self) -> float:
        elapsed = self._motion_elapsed_s
        if self.is_playback and self._motion_started_at_s is not None:
            elapsed += max(
                0.0,
                float(self.monotonic_fn()) - self._motion_started_at_s,
            ) * (float(self.op_cnd["playback_spd_rate"]) / 100.0)
        return elapsed

    def _checkpoint_motion_locked(self) -> None:
        """Preserve pose phase before changing the simulated playback rate."""
        if not self.random_pose or not self.is_playback:
            return
        self._motion_elapsed_s = self._motion_time_locked()
        self._motion_started_at_s = float(self.monotonic_fn())

    def _random_joints_locked(self) -> List[float]:
        elapsed = self._motion_time_locked()
        joints = []
        for bounds, profile in zip(RANDOM_POSE_BOUNDS_DEG, self._pose_profile):
            low, high = bounds
            center = (low + high) / 2.0
            half_span = (high - low) / 2.0
            slow_w, slow_phase, fast_w, fast_phase = profile
            # The weights leave a fixed margin inside each bound.
            wave = 0.58 * math.sin(slow_w * elapsed + slow_phase)
            wave += 0.27 * math.sin(fast_w * elapsed + fast_phase)
            joints.append(center + half_span * wave)
        return joints

    def _pause_motion_locked(self) -> None:
        if self.random_pose and self.is_playback:
            self._motion_elapsed_s = self._motion_time_locked()
            self._motion_started_at_s = None
        self.is_playback = False

    def pose(self, mechinfo: int = 1) -> Dict[str, Any]:
        with self.lock:
            if self.random_pose:
                self.joints[:6] = self._random_joints_locked()
            result: Dict[str, Any] = {
                "_type": "Pose",
                "nsync": 0,
                "crd": "joint",
                "mechinfo": mechinfo,
            }
            result.update(
                {
                    f"j{i + 1}": float(v)
                    for i, v in enumerate(self.joints[:6])
                }
            )
            return result

    def rgen(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "_type": "JObject",
                "cur_mode": 4,
                "enable_state": int(self.motor_state) & 0xFF,
                "is_playback": int(self.is_playback),
                "is_remote_mode": int(self.remote_mode),
                "is_ext_start": 1,
                "is_ext_prog_sel": 1,
                "cur_prog_no": 1,
                "cur_step_no": 1,
                "cur_func_no": 0,
                "mov_prog_no": 1 if self.is_playback else 0,
                "mov_step_no": 1 if self.is_playback else 0,
                "mov_func_no": 0,
                "spd_lev": 1,
                "manual_spd_max": 250,
                "auto_spd": int(
                    self.reported_speed_percent
                    if self.reported_speed_percent is not None
                    else self.op_cnd["playback_spd_rate"]
                ),
                "jog_inch_status": 0,
                "step_execute_unit_status": 0,
                "cont_path": 1,
            }

    def operation_condition(self) -> Dict[str, Any]:
        with self.lock:
            return {"_type": "CondGrp", **copy.deepcopy(self.op_cnd)}

    def record_write(
        self,
        method: str,
        path: str,
        body: Dict[str, Any],
    ) -> None:
        """Record a successful write. Caller must hold ``lock``."""
        self.write_count += 1
        self.write_log.append(
            {
                "method": method,
                "path": path,
                "body": copy.deepcopy(body),
                "monotonic_s": time.monotonic(),
            }
        )

    def defer_speed_readback(self, percent: int) -> None:
        """Apply controller-reported speed now or after a test-only delay."""
        self._speed_generation += 1
        generation = self._speed_generation
        if self.speed_readback_delay_s <= 0.0:
            self.reported_speed_percent = percent
            return

        def apply_if_current() -> None:
            with self.lock:
                if generation == self._speed_generation:
                    self.reported_speed_percent = percent

        timer = threading.Timer(self.speed_readback_delay_s, apply_if_current)
        timer.daemon = True
        timer.start()

    def start_playback(self) -> None:
        """Apply start and cancel any delayed stop readback."""
        self._stop_generation += 1
        if self.random_pose and not self.is_playback:
            self._motion_started_at_s = float(self.monotonic_fn())
        self.is_playback = True

    def defer_stop_readback(self) -> None:
        """Apply stopped state now or after a test-only delay."""
        self._stop_generation += 1
        generation = self._stop_generation
        if self.stop_readback_delay_s <= 0.0:
            self._pause_motion_locked()
            return

        def apply_if_current() -> None:
            with self.lock:
                if generation == self._stop_generation:
                    self._pause_motion_locked()

        timer = threading.Timer(self.stop_readback_delay_s, apply_if_current)
        timer.daemon = True
        timer.start()


class FakeHi6Server(ThreadingHTTPServer):
    """HTTP server carrying one independent fake controller state."""

    allow_reuse_address = True
    daemon_threads = True

    def __init__(
        self,
        address: Tuple[str, int],
        state: FakeHi6State,
        quiet: bool = False,
    ) -> None:
        self.state = state
        self.quiet = quiet
        super().__init__(address, FakeHi6Handler)


class RequestBodyError(ValueError):
    pass


class FakeHi6Handler(BaseHTTPRequestHandler):
    """Subset of the Hi6 REST API used by the bridge."""

    protocol_version = "HTTP/1.1"
    server_version = "FakeHi6/0.1"
    max_body_bytes = 64 * 1024
    # Header and JSON body are separate writes.  Avoid the Linux delayed-ACK
    # interaction that otherwise stalls persistent localhost requests ~40 ms.
    disable_nagle_algorithm = True

    @property
    def hi6_server(self) -> FakeHi6Server:
        return self.server  # type: ignore[return-value]

    @property
    def state(self) -> FakeHi6State:
        return self.hi6_server.state

    def log_message(self, fmt: str, *args: Any) -> None:
        if not self.hi6_server.quiet:
            super().log_message(f"[{self.state.robot_id}] {fmt}", *args)

    def _send_json(self, status: int, body: Any) -> None:
        raw = json.dumps(
            body,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def _error(
        self,
        status: int,
        message: str,
        code: Optional[int] = None,
    ) -> None:
        body: Dict[str, Any] = {"_type": "JObject", "_text": message}
        if code is not None:
            body["error_code"] = code
        self._send_json(status, body)

    def _read_json_object(self) -> Dict[str, Any]:
        value = self.headers.get("Content-Length", "0")
        try:
            length = int(value)
        except ValueError as exc:
            raise RequestBodyError("invalid Content-Length") from exc
        if length < 0 or length > self.max_body_bytes:
            raise RequestBodyError("request body is too large")
        raw = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RequestBodyError(
                "request body must be valid UTF-8 JSON"
            ) from exc
        if not isinstance(body, dict):
            raise RequestBodyError("request body must be a JSON object")
        return body

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlsplit(self.path)
        path = parsed.path

        if path == "/api_ver":
            self._send_json(200, self.state.api_version)
            return

        if path == "/versions/sysver":
            self._send_json(
                200,
                {
                    "modules": [
                        {
                            "name": "com",
                            "ver": self.state.system_version,
                            "build-date": "Aug 17 2026",
                            "build-time": "00:00:00",
                            "commit-id": "fake-hi6",
                        },
                        {
                            "name": "tp",
                            "ver": self.state.system_version,
                            "build-date": "Aug 17 2026",
                            "build-time": "00:00:00",
                            "commit-id": "fake-hi6",
                        },
                    ]
                },
            )
            return

        if path == "/project/robot/po_cur":
            query = parse_qs(parsed.query)
            try:
                mechinfo = int(query.get("mechinfo", ["1"])[0])
            except ValueError:
                self._error(400, "mechinfo must be an integer")
                return
            self._send_json(200, self.state.pose(mechinfo))
            return

        if path == "/project/rgen":
            self._send_json(200, self.state.rgen())
            return

        if path == "/project/control/op_cnd":
            self._send_json(200, self.state.operation_condition())
            return

        if path == "/project/robot/motor_on_state":
            with self.state.lock:
                val = int(self.state.motor_state)
            self._send_json(200, {"_type": "JObject", "val": val})
            return

        if path == "/project/robot/emergency_stop":
            with self.state.lock:
                val = int(self.state.emergency_stop)
            self._send_json(200, {"_type": "JObject", "val": val})
            return

        self._error(404, f"unknown endpoint: {path}")

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlsplit(self.path)
        path = parsed.path
        try:
            body = self._read_json_object()
        except RequestBodyError as exc:
            self._error(400, str(exc))
            return

        if path == "/project/robot/start":
            with self.state.lock:
                if not self.state.remote_mode:
                    self._error(
                        403,
                        "controller is not in remote mode",
                        code=-38500,
                    )
                    return
                self.state.start_playback()
                self.state.record_write("POST", path, body)
            self._send_json(200, {"_type": "JObject"})
            return

        if path == "/project/robot/stop":
            with self.state.lock:
                self.state.defer_stop_readback()
                self.state.record_write("POST", path, body)
            self._send_json(200, {"_type": "JObject"})
            return

        self._error(404, f"unknown endpoint: {path}")

    def do_PUT(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlsplit(self.path)
        path = parsed.path
        try:
            body = self._read_json_object()
        except RequestBodyError as exc:
            self._error(400, str(exc))
            return

        if path != "/project/control/op_cnd":
            self._error(404, f"unknown endpoint: {path}")
            return

        try:
            updates = _validate_op_cnd(body)
        except ValueError as exc:
            self._error(400, str(exc))
            return

        with self.state.lock:
            if self.state.reported_speed_percent is None:
                self.state.reported_speed_percent = int(
                    self.state.op_cnd["playback_spd_rate"]
                )
            self.state._checkpoint_motion_locked()
            self.state.op_cnd.update(updates)
            self.state.defer_speed_readback(updates["playback_spd_rate"])
            self.state.record_write("PUT", path, body)
        self._send_json(200, {"_text": ""})


_OP_CND_RANGES: Dict[str, Tuple[int, int]] = {
    "playback_mode": (1, 2),
    "step_goback_max_spd": (10, 250),
    "step_go_func_ex": (0, 2),
    "func_reexe_on_trace": (0, 2),
    "path_recov_confirm": (0, 2),
    "playback_spd_rate": (1, 100),
    "robot_lock": (0, 1),
    "intp_base": (0, 1),
    "ucrd_num": (0, 20),
    "plc_mode": (0, 4),
}


def _validate_op_cnd(body: Dict[str, Any]) -> Dict[str, int]:
    """Validate a partial operation-condition update atomically."""
    if "playback_spd_rate" not in body:
        raise ValueError("playback_spd_rate is required")

    updates: Dict[str, int] = {}
    for key, value in body.items():
        if key == "_type" and value == "CondGrp":
            continue
        if key not in _OP_CND_RANGES:
            raise ValueError(f"unknown operation-condition field: {key}")
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{key} must be an integer")
        low, high = _OP_CND_RANGES[key]
        if not low <= value <= high:
            raise ValueError(f"{key} must be between {low} and {high}")
        updates[key] = value
    return updates


def make_server(
    host: str = "127.0.0.1",
    port: int = 18888,
    robot_id: str = "loading",
    *,
    state: Optional[FakeHi6State] = None,
    quiet: bool = False,
) -> FakeHi6Server:
    """Create, but do not start, a fake controller server."""
    controller = state or FakeHi6State(robot_id=robot_id)
    return FakeHi6Server((host, int(port)), controller, quiet=quiet)


def serve(
    host: str = "127.0.0.1",
    port: int = 18888,
    robot_id: str = "loading",
    *,
    quiet: bool = False,
    speed_readback_delay_s: float = 0.0,
    stop_readback_delay_s: float = 0.0,
    random_pose: bool = False,
    random_seed: Optional[int] = None,
) -> None:
    state = FakeHi6State(
        robot_id=robot_id,
        speed_readback_delay_s=max(0.0, speed_readback_delay_s),
        stop_readback_delay_s=max(0.0, stop_readback_delay_s),
        random_pose=random_pose,
        random_seed=random_seed,
    )
    with make_server(host, port, robot_id, state=state, quiet=quiet) as server:
        bound_host, bound_port = server.server_address[:2]
        print(
            f"[fake-hi6:{robot_id}] Open API server listening on "
            f"http://{bound_host}:{bound_port}"
        )
        try:
            server.serve_forever(poll_interval=0.1)
        except KeyboardInterrupt:
            pass


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description="Fake Hi6 REST/Open API controller"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18888)
    parser.add_argument("--robot-id", default="loading")
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="suppress per-request log lines",
    )
    parser.add_argument(
        "--speed-readback-delay",
        type=float,
        default=0.0,
        help="delay rgen auto_spd changes for confirmation/preemption tests",
    )
    parser.add_argument(
        "--stop-readback-delay",
        type=float,
        default=0.0,
        help="delay rgen stopped state for repeated-stop dispatch tests",
    )
    parser.add_argument(
        "--random-pose",
        action="store_true",
        help=(
            "publish a smooth bounded six-axis pose for RViz/Unity visual "
            "checks"
        ),
    )
    parser.add_argument(
        "--random-seed",
        type=int,
        default=None,
        help="override the deterministic robot-id-derived random-pose seed",
    )
    args = parser.parse_args(argv)
    serve(
        args.host,
        args.port,
        args.robot_id,
        quiet=args.quiet,
        speed_readback_delay_s=args.speed_readback_delay,
        stop_readback_delay_s=args.stop_readback_delay,
        random_pose=args.random_pose,
        random_seed=args.random_seed,
    )


if __name__ == "__main__":
    main()
