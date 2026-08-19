"""Tests for the bounded, read-only Hi6 connection preflight."""

from __future__ import annotations

import json
import threading
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Iterator

import pytest

import robot_bridge.hi6_connection as connection_module
from robot_bridge.hi6_client import Hi6ConnectionError
from robot_bridge.hi6_connection import (
    HI6_OPENAPI_PORT,
    Hi6ConnectionConfigError,
    Hi6Endpoint,
    Hi6PreflightStatus,
    Hi6ProbeStatus,
    preflight_hi6_connections,
    probe_hi6_endpoint,
)


class _VersionServer(ThreadingHTTPServer):
    """Minimal HTTP server that records every application request method."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self) -> None:
        super().__init__(("127.0.0.1", 0), _VersionHandler)
        self.requests: list[tuple[str, str]] = []
        self.connection_count = 0

    def get_request(self):
        """Count accepted sockets to verify use of one persistent session."""
        request, address = super().get_request()
        self.connection_count += 1
        return request, address


class _VersionHandler(BaseHTTPRequestHandler):
    """Serve only the two read-only identity resources."""

    protocol_version = "HTTP/1.1"

    def log_message(self, format: str, *args: Any) -> None:
        """Suppress local test-server access logging."""

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        """Return the documented version responses."""
        self.server.requests.append(("GET", self.path))
        if self.path == "/api_ver":
            self._send_json(5)
        elif self.path == "/versions/sysver":
            self._send_json(
                {"modules": [{"name": "com", "ver": "60.34-00"}]}
            )
        else:
            self._send_json({"message": "not found"}, status=404)

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        """Record any forbidden write attempt and reject it."""
        self.server.requests.append(("POST", self.path))
        self._send_json({"message": "write forbidden"}, status=405)

    def do_PUT(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        """Record any forbidden update attempt and reject it."""
        self.server.requests.append(("PUT", self.path))
        self._send_json({"message": "write forbidden"}, status=405)

    def _send_json(self, value: Any, status: int = 200) -> None:
        body = json.dumps(value, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)
        self.wfile.flush()


@contextmanager
def _version_server() -> Iterator[_VersionServer]:
    server = _VersionServer()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


class _StubClient:
    """Controllable read client for deterministic subnet tests."""

    def __init__(
        self,
        config: Any,
        plans: dict[str, dict[str, Any]],
        calls: list[tuple[str, str]],
        configs: list[Any],
    ) -> None:
        self.config = config
        self.plan = plans[config.host]
        self.calls = calls
        configs.append(config)

    def get_api_version(self) -> int:
        """Return or raise the planned API-version outcome."""
        self.calls.append((self.config.host, "GET /api_ver"))
        value = self.plan.get("api", 5)
        if isinstance(value, BaseException):
            raise value
        return value

    def get_system_version(self) -> dict[str, Any]:
        """Return the planned controller version document."""
        self.calls.append((self.config.host, "GET /versions/sysver"))
        return self.plan.get(
            "system",
            {"modules": [{"name": "com", "ver": "61.01-00"}]},
        )

    def close(self) -> None:
        """Record deterministic cleanup of the persistent session."""
        self.calls.append((self.config.host, "CLOSE"))


def _stub_factory(
    plans: dict[str, dict[str, Any]],
    calls: list[tuple[str, str]],
    configs: list[Any],
):
    """Create a Hi6Client-compatible factory backed by per-host plans."""
    return lambda config: _StubClient(config, plans, calls, configs)


def test_probe_uses_only_two_gets_on_one_persistent_connection() -> None:
    """Endpoint validation cannot issue a command or reconnect per GET."""
    with _version_server() as server:
        result = probe_hi6_endpoint(
            Hi6Endpoint("127.0.0.1", server.server_port)
        )

        assert result.status is Hi6ProbeStatus.VERIFIED
        assert result.api_version == 5
        assert result.controller_version == "60.34-00"
        assert server.requests == [
            ("GET", "/api_ver"),
            ("GET", "/versions/sysver"),
        ]
        assert server.connection_count == 1


def test_probe_passes_validated_source_address_to_client() -> None:
    """Identity GETs use the caller-selected local IPv4 source address."""
    calls: list[tuple[str, str]] = []
    configs: list[Any] = []
    endpoint = Hi6Endpoint("192.168.250.21")

    result = probe_hi6_endpoint(
        endpoint,
        source_address="192.168.250.10",
        client_factory=_stub_factory({endpoint.host: {}}, calls, configs),
    )

    assert result.status is Hi6ProbeStatus.VERIFIED
    assert len(configs) == 1
    assert configs[0].source_address == "192.168.250.10"


def test_tcp_port_check_binds_selected_source_address(monkeypatch) -> None:
    """The cheap TCP-8888 scan binds before connecting to a candidate."""
    calls: list[tuple[tuple[str, int], float, tuple[str, int]]] = []

    class _ConnectedSocket:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, traceback) -> None:
            return None

    def fake_create_connection(
        address: tuple[str, int],
        *,
        timeout: float,
        source_address: tuple[str, int],
    ) -> _ConnectedSocket:
        calls.append((address, timeout, source_address))
        return _ConnectedSocket()

    monkeypatch.setattr(
        connection_module.socket,
        "create_connection",
        fake_create_connection,
    )

    assert connection_module._tcp_port_open(
        "192.168.250.21",
        HI6_OPENAPI_PORT,
        0.25,
        "192.168.250.10",
    )
    assert calls == [
        (
            ("192.168.250.21", HI6_OPENAPI_PORT),
            0.25,
            ("192.168.250.10", 0),
        )
    ]


def test_configured_endpoint_wins_and_prevents_optional_scan() -> None:
    """A valid configured address is preferred without any subnet traffic."""
    with _version_server() as server:
        checker_calls: list[tuple[str, int, float]] = []

        def forbidden_checker(
            host: str,
            port: int,
            timeout: float,
            source_address: str | None,
        ) -> bool:
            checker_calls.append((host, port, timeout))
            raise AssertionError("configured success must not trigger a scan")

        result = preflight_hi6_connections(
            [Hi6Endpoint("127.0.0.1", server.server_port)],
            scan_subnet="127.0.0.0/30",
            port_checker=forbidden_checker,
        )

        assert result.status is Hi6PreflightStatus.CONFIGURED
        assert result.unique_candidate is not None
        assert result.unique_candidate.endpoint.port == server.server_port
        assert result.scanned_host_count == 0
        assert checker_calls == []


def test_multiple_discovered_controllers_are_ambiguous() -> None:
    """Discovery returns every verified endpoint and never assigns a role."""
    calls: list[tuple[str, str]] = []
    configs: list[Any] = []
    checked: list[tuple[str, int, float, str | None]] = []
    plans = {
        "192.168.250.1": {},
        "192.168.250.2": {},
    }

    def checker(
        host: str,
        port: int,
        timeout: float,
        source_address: str | None,
    ) -> bool:
        checked.append((host, port, timeout, source_address))
        return host in plans

    result = preflight_hi6_connections(
        scan_subnet="192.168.250.0/30",
        source_address="192.168.250.10",
        scan_workers=2,
        client_factory=_stub_factory(plans, calls, configs),
        port_checker=checker,
    )

    assert result.status is Hi6PreflightStatus.AMBIGUOUS
    assert result.unique_candidate is None
    assert [item.endpoint.host for item in result.candidates] == [
        "192.168.250.1",
        "192.168.250.2",
    ]
    assert result.scanned_subnet == "192.168.250.0/30"
    assert result.scanned_host_count == 2
    assert result.open_port_count == 2
    assert {host for host, _, _, _ in checked} == set(plans)
    assert all(port == HI6_OPENAPI_PORT for _, port, _, _ in checked)
    assert all(source == "192.168.250.10" for *_, source in checked)
    assert all(config.port == HI6_OPENAPI_PORT for config in configs)
    assert all(
        config.source_address == "192.168.250.10" for config in configs
    )
    assert all(config.retry_safe_requests is False for config in configs)
    assert {method for _, method in calls} == {
        "GET /api_ver",
        "GET /versions/sysver",
        "CLOSE",
    }


def test_only_verified_open_port_candidate_is_discovered() -> None:
    """An unrelated or incompatible service on 8888 is not a Hi6 candidate."""
    calls: list[tuple[str, str]] = []
    configs: list[Any] = []
    plans = {
        "192.168.250.1": {"api": 4},
        "192.168.250.2": {},
    }

    result = preflight_hi6_connections(
        scan_subnet="192.168.250.0/30",
        scan_workers=1,
        client_factory=_stub_factory(plans, calls, configs),
        port_checker=lambda host, port, timeout, source: True,
    )

    assert result.status is Hi6PreflightStatus.DISCOVERED
    assert result.unique_candidate is not None
    assert result.unique_candidate.endpoint.host == "192.168.250.2"
    assert [item.status for item in result.discovered_results] == [
        Hi6ProbeStatus.UNSUPPORTED_API,
        Hi6ProbeStatus.VERIFIED,
    ]
    assert ("192.168.250.1", "GET /versions/sysver") not in calls


def test_failed_configured_endpoint_can_fall_back_to_explicit_subnet() -> None:
    """An unreachable configured address does not disable explicit fallback."""
    calls: list[tuple[str, str]] = []
    configs: list[Any] = []
    plans = {
        "192.168.250.21": {
            "api": Hi6ConnectionError(
                "offline", method="GET", path="/api_ver"
            )
        },
        "192.168.250.2": {},
    }

    result = preflight_hi6_connections(
        [Hi6Endpoint("192.168.250.21")],
        scan_subnet="192.168.250.0/30",
        scan_workers=1,
        client_factory=_stub_factory(plans, calls, configs),
        port_checker=(
            lambda host, port, timeout, source: host.endswith(".2")
        ),
    )

    assert result.status is Hi6PreflightStatus.DISCOVERED
    assert result.configured_results[0].status is Hi6ProbeStatus.UNREACHABLE
    assert result.unique_candidate.endpoint.host == "192.168.250.2"


def test_no_subnet_means_no_scan() -> None:
    """There is no implicit interface or local-network discovery."""
    calls: list[tuple[str, str]] = []
    configs: list[Any] = []
    plans = {
        "192.168.250.21": {
            "api": Hi6ConnectionError(
                "offline", method="GET", path="/api_ver"
            )
        }
    }

    result = preflight_hi6_connections(
        [Hi6Endpoint("192.168.250.21")],
        client_factory=_stub_factory(plans, calls, configs),
        port_checker=lambda host, port, timeout, source: pytest.fail(
            "port checker must not run without scan_subnet"
        ),
    )

    assert result.status is Hi6PreflightStatus.NOT_FOUND
    assert result.scanned_subnet is None
    assert result.discovered_results == ()


def test_one_of_two_configured_controllers_is_partial() -> None:
    """One healthy robot cannot mark a two-robot connection set ready."""
    calls: list[tuple[str, str]] = []
    configs: list[Any] = []
    plans = {
        "192.168.250.21": {},
        "192.168.250.22": {
            "api": Hi6ConnectionError(
                "offline", method="GET", path="/api_ver"
            )
        },
    }

    result = preflight_hi6_connections(
        [
            Hi6Endpoint("192.168.250.21"),
            Hi6Endpoint("192.168.250.22"),
        ],
        client_factory=_stub_factory(plans, calls, configs),
    )

    assert result.status is Hi6PreflightStatus.PARTIAL
    assert [item.endpoint.host for item in result.candidates] == [
        "192.168.250.21"
    ]
    assert result.unique_candidate is not None


def test_missing_controller_module_is_not_verified() -> None:
    """A generic JSON service cannot pass only by copying the API version."""
    calls: list[tuple[str, str]] = []
    configs: list[Any] = []
    plans = {"192.168.250.21": {"system": {"modules": []}}}

    result = probe_hi6_endpoint(
        Hi6Endpoint("192.168.250.21"),
        client_factory=_stub_factory(plans, calls, configs),
    )

    assert result.status is Hi6ProbeStatus.INVALID_RESPONSE
    assert result.api_version == 5
    assert result.verified is False


def test_duplicate_configured_endpoints_are_probed_once() -> None:
    """Repeated configuration cannot create duplicate candidates or traffic."""
    calls: list[tuple[str, str]] = []
    configs: list[Any] = []
    plans = {"192.168.250.21": {}}
    endpoint = Hi6Endpoint("192.168.250.21")

    result = preflight_hi6_connections(
        [endpoint, endpoint],
        client_factory=_stub_factory(plans, calls, configs),
    )

    assert result.status is Hi6PreflightStatus.CONFIGURED
    assert len(result.configured_results) == 1
    assert calls.count((endpoint.host, "GET /api_ver")) == 1


@pytest.mark.parametrize(
    "subnet",
    [
        "192.168.0.0/23",
        "192.168.250.7/24",
        "2001:db8::/120",
        "8.8.8.0/24",
        "0.0.0.0/24",
        "",
    ],
)
def test_unsafe_or_ambiguous_scan_scope_is_rejected(subnet: str) -> None:
    """Only an exact, small, non-public IPv4 network may be scanned."""
    with pytest.raises(Hi6ConnectionConfigError):
        preflight_hi6_connections(scan_subnet=subnet)


@pytest.mark.parametrize("workers", [0, 33, True, 1.5])
def test_invalid_worker_count_is_rejected(workers: Any) -> None:
    """Concurrency is explicitly bounded to avoid bursty network traffic."""
    with pytest.raises(Hi6ConnectionConfigError):
        preflight_hi6_connections(scan_workers=workers)


@pytest.mark.parametrize(
    "source_address",
    ["robot-controller.local", "2001:db8::10", "999.1.2.3", 123],
)
def test_invalid_source_address_is_rejected(source_address: Any) -> None:
    """Source binding accepts only an optional numeric IPv4 address."""
    with pytest.raises(Hi6ConnectionConfigError):
        preflight_hi6_connections(source_address=source_address)
