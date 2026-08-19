"""Unit tests for the Hi6 Open API client using only a local fake server."""

from __future__ import annotations

import json
import threading
import time
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Iterator

import pytest

from robot_bridge.hi6_client import (
    Hi6Client,
    Hi6Config,
    Hi6HttpError,
    Hi6ProtocolError,
    Hi6TimeoutError,
    Hi6ValidationError,
    MotorState,
)


class _FakeHi6Server(ThreadingHTTPServer):
    """HTTP server that records requests and accepted TCP connections."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self) -> None:
        super().__init__(("127.0.0.1", 0), _FakeHi6Handler)
        self.requests: list[tuple[str, str, Any]] = []
        self.connection_count = 0
        self.peer_addresses: list[str] = []
        self.close_next_silently = False
        self.fail_start = False
        self.malformed_api_version = False
        self.delay_api_version_s = 0.0

    def get_request(self):
        request, address = super().get_request()
        self.connection_count += 1
        self.peer_addresses.append(address[0])
        return request, address


class _FakeHi6Handler(BaseHTTPRequestHandler):
    """Minimal subset of the vendor API needed by the client tests."""

    protocol_version = "HTTP/1.1"

    def log_message(self, format: str, *args: Any) -> None:
        """Keep expected disconnects from polluting test output."""

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        self.server.requests.append(("GET", self.path, None))
        if self.path == "/api_ver":
            if self.server.delay_api_version_s:
                time.sleep(self.server.delay_api_version_s)
            if self.server.malformed_api_version:
                self._send_bytes(b"{not-json")
            else:
                self._send_json(5)
            return
        if self.path == "/versions/sysver":
            self._send_json(
                {"modules": [{"name": "com", "ver": "60.34-00"}]}
            )
            return
        if self.path == "/project/robot/po_cur?crd=2&mechinfo=1":
            self._send_json(
                {
                    "_type": "Pose",
                    "crd": "joint",
                    "mechinfo": 1,
                    "j1": 1,
                    "j2": 2.5,
                    "j3": -3,
                    "j4": 4,
                    "j5": -5.25,
                    "j6": 6,
                }
            )
            return
        if self.path == "/project/rgen":
            self._send_json(
                {
                    "cur_mode": 4,
                    "enable_state": 0x0100,
                    "is_playback": 1,
                    "is_remote_mode": 1,
                    "is_ext_start": 0,
                    "is_ext_prog_sel": 1,
                    "auto_spd": 50,
                    "cur_prog_no": 7,
                    "cur_step_no": 12,
                    "mov_prog_no": 7,
                    "mov_step_no": 13,
                    "eid_last_err": -1,
                }
            )
            return
        if self.path == "/project/robot/motor_on_state":
            self._send_json({"_type": "JObject", "val": 0})
            return
        if self.path == "/project/robot/emergency_stop":
            self._send_json({"_type": "JObject", "val": 1})
            return
        if self.path == "/project/control/op_cnd":
            self._send_json(
                {"_type": "CondGrp", "playback_spd_rate": 50}
            )
            return
        self._send_json({"message": "not found"}, status=404)

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        body = self._read_json()
        self.server.requests.append(("POST", self.path, body))
        if self.path == "/project/robot/start" and self.server.fail_start:
            self._send_json({"err_code": -38500}, status=403)
            return
        if self.path in ("/project/robot/start", "/project/robot/stop"):
            self._send_json({"_type": "JObject"})
            return
        self._send_json({"message": "not found"}, status=404)

    def do_PUT(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        body = self._read_json()
        self.server.requests.append(("PUT", self.path, body))
        if self.path == "/project/control/op_cnd":
            self._send_json({"_text": ""})
            return
        self._send_json({"message": "not found"}, status=404)

    def _read_json(self) -> Any:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length)
        return json.loads(raw.decode("utf-8")) if raw else None

    def _send_json(self, value: Any, status: int = 200) -> None:
        body = json.dumps(value, separators=(",", ":")).encode("utf-8")
        self._send_bytes(body, status=status)

    def _send_bytes(self, body: bytes, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return
        if self.server.close_next_silently:
            self.server.close_next_silently = False
            self.close_connection = True


@contextmanager
def _fake_server() -> Iterator[_FakeHi6Server]:
    server = _FakeHi6Server()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def _client(server: _FakeHi6Server, **kwargs: Any) -> Hi6Client:
    config = Hi6Config(host="127.0.0.1", port=server.server_port, **kwargs)
    return Hi6Client(config)


def test_monitoring_api_and_keep_alive() -> None:
    """Read APIs share one connection and normalize documented fields."""
    with _fake_server() as server, _client(server) as client:
        assert client.get_api_version() == 5
        assert client.get_system_version()["modules"][0]["name"] == "com"
        assert client.get_joint_positions() == (
            1.0, 2.5, -3.0, 4.0, -5.25, 6.0
        )
        assert client.get_motor_state() is MotorState.ON
        assert client.get_emergency_stop() is True
        assert client.get_playback_speed_percent() == 50
        status = client.get_status()

        assert status == {
            "mode_code": 4,
            "motor_state_code": 0,
            "is_playback": True,
            "is_remote_mode": True,
            "is_external_start": False,
            "is_external_program_select": True,
            "playback_speed_percent": 50,
            "current_program": 7,
            "current_step": 12,
            "moving_program": 7,
            "moving_step": 13,
            "last_error_id": -1,
        }
        assert server.connection_count == 1


def test_commands_are_single_requests_and_speed_update_is_partial() -> None:
    """Command bodies contain no unrelated operation-condition settings."""
    with _fake_server() as server, _client(server) as client:
        assert client.start() == {"_type": "JObject"}
        assert client.stop() == {"_type": "JObject"}
        assert client.set_playback_speed_percent(25) == {"_text": ""}

        assert server.requests == [
            ("POST", "/project/robot/start", {}),
            ("POST", "/project/robot/stop", {}),
            (
                "PUT",
                "/project/control/op_cnd",
                {"playback_spd_rate": 25},
            ),
        ]


def test_numeric_ipv4_source_address_is_used_for_the_http_connection() -> None:
    """Persistent sessions can be pinned to the control-network address."""
    with _fake_server() as server, _client(
        server, source_address="127.0.0.1"
    ) as client:
        assert client.get_api_version() == 5
        assert client.config.source_address == "127.0.0.1"
        assert client._connection is not None
        assert client._connection.source_address == ("127.0.0.1", 0)
        assert server.peer_addresses == ["127.0.0.1"]


@pytest.mark.parametrize("value", [None, "", "   "])
def test_empty_source_address_disables_explicit_binding(value: Any) -> None:
    """An omitted source remains available for localhost/mock operation."""
    assert Hi6Config(source_address=value).source_address is None


@pytest.mark.parametrize("value", [0, 101, True, 50.0, "50"])
def test_invalid_speed_is_rejected_before_network(value: Any) -> None:
    """Invalid percentages cannot reach the controller."""
    with _fake_server() as server, _client(server) as client:
        with pytest.raises(Hi6ValidationError):
            client.set_playback_speed_percent(value)
        assert server.requests == []


def test_http_error_exposes_status_and_json_body() -> None:
    """Controller rejection details remain available to the ROS adapter."""
    with _fake_server() as server, _client(server) as client:
        server.fail_start = True
        with pytest.raises(Hi6HttpError) as caught:
            client.start()

        error = caught.value
        assert error.status == 403
        assert error.method == "POST"
        assert error.path == "/project/robot/start"
        assert error.body == {"err_code": -38500}
        assert "403" in str(error)
        assert "-38500" in str(error)
        assert len(server.requests) == 1


def test_malformed_success_response_raises_protocol_error() -> None:
    """A successful HTTP status cannot hide an invalid response document."""
    with _fake_server() as server, _client(server) as client:
        server.malformed_api_version = True
        with pytest.raises(Hi6ProtocolError):
            client.get_api_version()


def test_stale_keep_alive_is_reconnected_once_for_get() -> None:
    """A safe read recovers once after the peer silently closes its socket."""
    with _fake_server() as server, _client(server) as client:
        server.close_next_silently = True
        assert client.get_api_version() == 5
        assert client.get_api_version() == 5
        assert server.connection_count == 2
        assert [request[0] for request in server.requests] == ["GET", "GET"]


def test_read_timeout_has_specific_exception() -> None:
    """A slow controller is distinguishable from an HTTP-level rejection."""
    with _fake_server() as server:
        server.delay_api_version_s = 0.2
        with _client(
            server,
            read_timeout_s=0.02,
            retry_safe_requests=False,
        ) as client:
            with pytest.raises(Hi6TimeoutError):
                client.get_api_version()


@pytest.mark.parametrize(
    "kwargs",
    [
        {"host": ""},
        {"host": "http://192.168.1.150"},
        {"port": 0},
        {"connect_timeout_s": 0},
        {"read_timeout_s": float("inf")},
        {"max_response_bytes": 0},
        {"source_address": "controller-nic.local"},
        {"source_address": "2001:db8::1"},
        {"source_address": "999.1.1.1"},
        {"source_address": 1234},
    ],
)
def test_invalid_config_is_rejected(kwargs: dict[str, Any]) -> None:
    """Unsafe or ambiguous connection configuration fails immediately."""
    with pytest.raises(Hi6ValidationError):
        Hi6Config(**kwargs)
