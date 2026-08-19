"""
Small, dependency-free client for the HD Hyundai Hi6 Open API.

The controller speaks HTTP/1.1 on its Open API port.  A client instance owns
one persistent connection and serializes access to it, avoiding repeatedly
opening connections against the controller.  Safe GET requests are retried once
when a stale keep-alive connection is detected; command requests are never
retried implicitly.

Angular values returned by :meth:`Hi6Client.get_joint_positions` are degrees,
matching the controller API.  Conversion to ROS radians belongs at the ROS
adapter boundary.
"""

from __future__ import annotations

import http.client
import ipaddress
import json
import math
import socket
import threading
from dataclasses import dataclass, fields
from enum import IntEnum
from typing import Any, Mapping, Optional, Sequence, Tuple
from urllib.parse import urlencode


class Hi6Error(RuntimeError):
    """Base class for all errors reported by this module."""


class Hi6ValidationError(Hi6Error, ValueError):
    """A request was rejected locally before anything was sent."""


class Hi6TransportError(Hi6Error):
    """The controller was unreachable or the connection was interrupted."""

    def __init__(self, message: str, *, method: str, path: str):
        """Capture the failed request method and path."""
        self.method = method
        self.path = path
        super().__init__(f"{message} ({method} {path})")


class Hi6TimeoutError(Hi6TransportError, TimeoutError):
    """A connection or response timed out."""


class Hi6ConnectionError(Hi6TransportError, ConnectionError):
    """A non-timeout network or HTTP framing error occurred."""


class Hi6HttpError(Hi6Error):
    """The controller returned a non-successful HTTP status."""

    def __init__(
        self,
        *,
        method: str,
        path: str,
        status: int,
        reason: str,
        body: Any,
    ):
        """Capture the complete non-success response metadata."""
        self.method = method
        self.path = path
        self.status = status
        self.reason = reason
        self.body = body
        body_suffix = "" if body in (None, "", {}) else f"; body={body!r}"
        super().__init__(
            f"Hi6 HTTP {status} {reason} ({method} {path}){body_suffix}"
        )


class Hi6ProtocolError(Hi6Error):
    """The controller response did not match the documented JSON contract."""

    def __init__(
        self,
        message: str,
        *,
        method: str,
        path: str,
        body: Any = None,
    ):
        """Capture the request and invalid response value."""
        self.method = method
        self.path = path
        self.body = body
        super().__init__(f"{message} ({method} {path})")


class MotorState(IntEnum):
    """Values returned by ``/project/robot/motor_on_state``."""

    ON = 0
    OFF = 1
    BUSY = 2


@dataclass(frozen=True)
class Hi6Config:
    """Connection and response limits for a Hi6 Open API session."""

    host: str = "192.168.1.150"
    port: int = 8888
    source_address: Optional[str] = None
    connect_timeout_s: float = 3.0
    read_timeout_s: float = 2.0
    max_response_bytes: int = 1024 * 1024
    retry_safe_requests: bool = True
    user_agent: str = "robot-dt-bridge/0.1"

    def __post_init__(self) -> None:
        """Reject unusable networking and resource-limit values."""
        if not isinstance(self.host, str) or not self.host.strip():
            raise Hi6ValidationError("host must be a non-empty hostname or IP")
        if "://" in self.host:
            raise Hi6ValidationError("host must not include an URL scheme")
        if isinstance(self.port, bool) or not isinstance(self.port, int):
            raise Hi6ValidationError("port must be an integer")
        if not 1 <= self.port <= 65535:
            raise Hi6ValidationError("port must be in the range 1..65535")
        source_address = self.source_address
        if source_address is not None and not isinstance(source_address, str):
            raise Hi6ValidationError(
                "source_address must be a numeric IPv4 address or empty"
            )
        source_text = (source_address or "").strip()
        if source_text:
            try:
                normalized_source = str(ipaddress.IPv4Address(source_text))
            except ipaddress.AddressValueError as exc:
                raise Hi6ValidationError(
                    "source_address must be a numeric IPv4 address or empty"
                ) from exc
        else:
            normalized_source = None
        object.__setattr__(self, "source_address", normalized_source)
        _validate_positive_finite(self.connect_timeout_s, "connect_timeout_s")
        _validate_positive_finite(self.read_timeout_s, "read_timeout_s")
        if (
            isinstance(self.max_response_bytes, bool)
            or not isinstance(self.max_response_bytes, int)
            or self.max_response_bytes < 1
        ):
            raise Hi6ValidationError(
                "max_response_bytes must be a positive integer"
            )
        if not isinstance(self.retry_safe_requests, bool):
            raise Hi6ValidationError("retry_safe_requests must be boolean")
        if not isinstance(self.user_agent, str) or not self.user_agent.strip():
            raise Hi6ValidationError("user_agent must be a non-empty string")

    @classmethod
    def from_dict(cls, values: Mapping[str, Any]) -> "Hi6Config":
        """Build a configuration while ignoring unrelated YAML keys."""
        known = {item.name for item in fields(cls)}
        selected = {
            key: value for key, value in values.items() if key in known
        }
        return cls(**selected)


class Hi6Client:
    """Thread-safe, persistent HTTP client for one Hi6 controller."""

    def __init__(self, config: Optional[Hi6Config] = None):
        """Create a lazy session without making a network call."""
        self.config = config or Hi6Config()
        self._connection: Optional[http.client.HTTPConnection] = None
        self._lock = threading.RLock()

    @property
    def connected(self) -> bool:
        """Whether the client currently has an open TCP socket."""
        with self._lock:
            return (
                self._connection is not None
                and self._connection.sock is not None
            )

    def close(self) -> None:
        """Close the persistent connection; the next call reconnects lazily."""
        with self._lock:
            self._close_unlocked()

    def __enter__(self) -> "Hi6Client":
        """Return this client as a context-managed session."""
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        """Close the persistent session when leaving a context."""
        self.close()

    # ------------------------------------------------------------- version API
    def get_api_version(self) -> int:
        """Return the controller Open API schema version."""
        path = "/api_ver"
        value = self._request_json("GET", path)
        return _expect_int(value, "API version", "GET", path)

    def get_system_version(self) -> dict[str, Any]:
        """Return controller and teaching-pendant module version metadata."""
        path = "/versions/sysver"
        value = self._request_json("GET", path)
        result = _expect_object(value, "system version", "GET", path)
        if not isinstance(result.get("modules"), list):
            raise Hi6ProtocolError(
                "system version response has no modules array",
                method="GET",
                path=path,
                body=result,
            )
        return result

    # --------------------------------------------------------------- state API
    def get_pose(
        self,
        *,
        crd: Optional[int] = None,
        mechinfo: Optional[int] = None,
        task_no: Optional[int] = None,
        ucrd_no: Optional[int] = None,
    ) -> dict[str, Any]:
        """Return the documented raw pose object for a coordinate system."""
        if task_no is not None:
            _validate_int_range(task_no, "task_no", 0, 7)
        if crd is not None:
            _validate_integer(crd, "crd")
        if mechinfo is not None:
            _validate_integer(mechinfo, "mechinfo")
        if ucrd_no is not None:
            _validate_integer(ucrd_no, "ucrd_no")

        query = _query_string(
            (
                ("task_no", task_no),
                ("crd", crd),
                ("ucrd_no", ucrd_no),
                ("mechinfo", mechinfo),
            )
        )
        path = "/project/robot/po_cur" + query
        value = self._request_json("GET", path)
        return _expect_object(value, "pose", "GET", path)

    def get_joint_pose(
        self,
        *,
        mechinfo: int = 1,
        task_no: Optional[int] = None,
    ) -> dict[str, Any]:
        """Return the raw joint-coordinate pose object (angles are degrees)."""
        return self.get_pose(crd=2, mechinfo=mechinfo, task_no=task_no)

    def get_joint_positions(
        self,
        axis_count: int = 6,
        *,
        mechinfo: int = 1,
        task_no: Optional[int] = None,
    ) -> Tuple[float, ...]:
        """Return ``j1`` through ``jN`` in degrees, in deterministic order."""
        _validate_int_range(axis_count, "axis_count", 1, 99)
        pose = self.get_joint_pose(mechinfo=mechinfo, task_no=task_no)
        coordinate = pose.get("crd")
        if coordinate is not None and coordinate != "joint":
            raise Hi6ProtocolError(
                f"expected joint pose, got crd={coordinate!r}",
                method="GET",
                path="/project/robot/po_cur",
                body=pose,
            )

        values = []
        for number in range(1, axis_count + 1):
            key = f"j{number}"
            value = pose.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise Hi6ProtocolError(
                    f"joint pose has no numeric {key}",
                    method="GET",
                    path="/project/robot/po_cur",
                    body=pose,
                )
            value = float(value)
            if not math.isfinite(value):
                raise Hi6ProtocolError(
                    f"joint pose contains non-finite {key}",
                    method="GET",
                    path="/project/robot/po_cur",
                    body=pose,
                )
            values.append(value)
        return tuple(values)

    def get_rgen(self) -> dict[str, Any]:
        """Return the complete, controller-native ``rgen`` status object."""
        path = "/project/rgen"
        value = self._request_json("GET", path)
        return _expect_object(value, "rgen status", "GET", path)

    def get_status(self) -> dict[str, Any]:
        """
        Return a compact normalized view of the common ``rgen`` fields.

        Keys are ``mode_code``, ``motor_state_code``, ``is_playback``,
        ``is_remote_mode``, ``is_external_start``,
        ``is_external_program_select``, ``playback_speed_percent``,
        ``current_program``, ``current_step``, ``moving_program``,
        ``moving_step``, and ``last_error_id``.  Optional controller fields are
        represented as ``None`` rather than silently assuming a safe state.
        Use :meth:`get_rgen` when the full vendor response is required.
        """
        path = "/project/rgen"
        raw = self.get_rgen()
        enable_state = _required_int_field(raw, "enable_state", "GET", path)
        return {
            "mode_code": _required_int_field(raw, "cur_mode", "GET", path),
            "motor_state_code": enable_state & 0xFF,
            "is_playback": _required_flag_field(
                raw, "is_playback", "GET", path
            ),
            "is_remote_mode": _required_flag_field(
                raw, "is_remote_mode", "GET", path
            ),
            "is_external_start": _optional_flag_field(
                raw, "is_ext_start", "GET", path
            ),
            "is_external_program_select": _optional_flag_field(
                raw, "is_ext_prog_sel", "GET", path
            ),
            "playback_speed_percent": _required_int_field(
                raw, "auto_spd", "GET", path
            ),
            "current_program": _optional_int_field(
                raw, "cur_prog_no", "GET", path
            ),
            "current_step": _optional_int_field(
                raw, "cur_step_no", "GET", path
            ),
            "moving_program": _optional_int_field(
                raw, "mov_prog_no", "GET", path
            ),
            "moving_step": _optional_int_field(
                raw, "mov_step_no", "GET", path
            ),
            "last_error_id": _optional_int_field(
                raw, "eid_last_err", "GET", path
            ),
        }

    def get_motor_state(self) -> MotorState:
        """Return the motor state as ``ON``, ``OFF``, or ``BUSY``."""
        path = "/project/robot/motor_on_state"
        value = self._request_json("GET", path)
        result = _expect_object(value, "motor state", "GET", path)
        state = _required_int_field(result, "val", "GET", path)
        try:
            return MotorState(state)
        except ValueError as exc:
            raise Hi6ProtocolError(
                f"unknown motor state {state}",
                method="GET",
                path=path,
                body=result,
            ) from exc

    def get_emergency_stop(self) -> bool:
        """Return whether the controller reports an emergency stop pressed."""
        path = "/project/robot/emergency_stop"
        value = self._request_json("GET", path)
        result = _expect_object(value, "emergency stop state", "GET", path)
        return _required_flag_field(result, "val", "GET", path)

    # ----------------------------------------------------------- control API
    def start(self) -> dict[str, Any]:
        """Request remote program start; never retry it automatically."""
        return self._command("/project/robot/start")

    def stop(self) -> dict[str, Any]:
        """Request normal external stop; this is not safety-rated."""
        return self._command("/project/robot/stop")

    def get_operation_condition(self) -> dict[str, Any]:
        """Return the complete controller operation-condition object."""
        path = "/project/control/op_cnd"
        value = self._request_json("GET", path)
        return _expect_object(value, "operation condition", "GET", path)

    def get_playback_speed_percent(self) -> int:
        """Return the configured automatic playback speed percentage."""
        path = "/project/control/op_cnd"
        result = self.get_operation_condition()
        percent = _required_int_field(
            result, "playback_spd_rate", "GET", path
        )
        if not 1 <= percent <= 100:
            raise Hi6ProtocolError(
                f"invalid playback_spd_rate {percent}",
                method="GET",
                path=path,
                body=result,
            )
        return percent

    def set_playback_speed_percent(self, percent: int) -> dict[str, Any]:
        """
        Set automatic playback speed to 1..100 percent.

        Hi6 documents partial updates for ``op_cnd``.  Sending only
        ``playback_spd_rate`` avoids overwriting unrelated teaching-pendant
        configuration.  The returned object acknowledges only the HTTP/API
        request; callers should read the value back before reporting confirmed
        physical state.
        """
        _validate_int_range(percent, "percent", 1, 100)
        path = "/project/control/op_cnd"
        value = self._request_json(
            "PUT", path, payload={"playback_spd_rate": percent}
        )
        return _expect_object(value, "operation-condition update", "PUT", path)

    # --------------------------------------------------------- HTTP internals
    def _command(self, path: str) -> dict[str, Any]:
        value = self._request_json("POST", path, payload={})
        return _expect_object(value, "command response", "POST", path)

    def _request_json(
        self,
        method: str,
        path: str,
        *,
        payload: Optional[Mapping[str, Any]] = None,
    ) -> Any:
        if not path.startswith("/"):
            raise Hi6ValidationError("request path must start with '/'")
        method = method.upper()
        safe_retry = method == "GET" and self.config.retry_safe_requests
        retries = 1 if safe_retry else 0

        with self._lock:
            for attempt in range(retries + 1):
                try:
                    return self._request_once(method, path, payload)
                except Hi6TransportError:
                    self._close_unlocked()
                    if attempt >= retries:
                        raise
            raise AssertionError("unreachable request retry state")

    def _request_once(
        self,
        method: str,
        path: str,
        payload: Optional[Mapping[str, Any]],
    ) -> Any:
        connection = self._get_connection_unlocked()
        headers = {
            "Accept": "application/json",
            "Connection": "keep-alive",
            "User-Agent": self.config.user_agent,
        }
        body: Optional[bytes] = None
        if payload is not None:
            body = json.dumps(
                payload, ensure_ascii=False, separators=(",", ":")
            ).encode("utf-8")
            headers["Content-Type"] = "application/json; charset=utf-8"

        try:
            connection.request(method, path, body=body, headers=headers)
            if connection.sock is not None:
                connection.sock.settimeout(self.config.read_timeout_s)
            response = connection.getresponse()
            raw_body = self._read_response(response, method, path)
        except (socket.timeout, TimeoutError) as exc:
            raise Hi6TimeoutError(
                "Hi6 request timed out", method=method, path=path
            ) from exc
        except (OSError, http.client.HTTPException) as exc:
            raise Hi6ConnectionError(
                f"Hi6 connection failed: {exc}", method=method, path=path
            ) from exc

        if not 200 <= response.status < 300:
            error_body = _decode_error_body(raw_body)
            raise Hi6HttpError(
                method=method,
                path=path,
                status=response.status,
                reason=response.reason or "",
                body=error_body,
            )

        if not raw_body:
            return {}
        try:
            return json.loads(raw_body.decode("utf-8-sig"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise Hi6ProtocolError(
                "response is not valid UTF-8 JSON",
                method=method,
                path=path,
                body=raw_body,
            ) from exc

    def _get_connection_unlocked(self) -> http.client.HTTPConnection:
        if self._connection is None:
            self._connection = http.client.HTTPConnection(
                self.config.host,
                self.config.port,
                timeout=self.config.connect_timeout_s,
                source_address=(self.config.source_address, 0)
                if self.config.source_address is not None
                else None,
            )
        return self._connection

    def _read_response(
        self,
        response: http.client.HTTPResponse,
        method: str,
        path: str,
    ) -> bytes:
        content_length = response.getheader("Content-Length")
        if content_length is not None:
            try:
                declared_length = int(content_length)
            except ValueError as exc:
                self._close_unlocked()
                raise Hi6ProtocolError(
                    "invalid Content-Length response header",
                    method=method,
                    path=path,
                    body=content_length,
                ) from exc
            if declared_length > self.config.max_response_bytes:
                self._close_unlocked()
                raise Hi6ProtocolError(
                    "response exceeds max_response_bytes",
                    method=method,
                    path=path,
                    body=declared_length,
                )

        raw_body = response.read(self.config.max_response_bytes + 1)
        response.close()
        if len(raw_body) > self.config.max_response_bytes:
            self._close_unlocked()
            raise Hi6ProtocolError(
                "response exceeds max_response_bytes",
                method=method,
                path=path,
                body=len(raw_body),
            )
        return raw_body

    def _close_unlocked(self) -> None:
        connection, self._connection = self._connection, None
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass


def _decode_error_body(raw_body: bytes) -> Any:
    if not raw_body:
        return None
    try:
        return json.loads(raw_body.decode("utf-8-sig"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return raw_body.decode("utf-8", errors="replace")


def _expect_object(
    value: Any,
    label: str,
    method: str,
    path: str,
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise Hi6ProtocolError(
            f"{label} response is not a JSON object",
            method=method,
            path=path,
            body=value,
        )
    return value


def _expect_int(value: Any, label: str, method: str, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise Hi6ProtocolError(
            f"{label} is not an integer",
            method=method,
            path=path,
            body=value,
        )
    return value


def _required_int_field(
    value: Mapping[str, Any],
    key: str,
    method: str,
    path: str,
) -> int:
    if key not in value:
        raise Hi6ProtocolError(
            f"response is missing integer field {key!r}",
            method=method,
            path=path,
            body=value,
        )
    return _expect_int(value[key], key, method, path)


def _optional_int_field(
    value: Mapping[str, Any],
    key: str,
    method: str,
    path: str,
) -> Optional[int]:
    if key not in value:
        return None
    return _expect_int(value[key], key, method, path)


def _required_flag_field(
    value: Mapping[str, Any],
    key: str,
    method: str,
    path: str,
) -> bool:
    if key not in value:
        raise Hi6ProtocolError(
            f"response is missing flag field {key!r}",
            method=method,
            path=path,
            body=value,
        )
    return _as_flag(value[key], key, method, path, value)


def _optional_flag_field(
    value: Mapping[str, Any],
    key: str,
    method: str,
    path: str,
) -> Optional[bool]:
    if key not in value:
        return None
    return _as_flag(value[key], key, method, path, value)


def _as_flag(
    value: Any,
    label: str,
    method: str,
    path: str,
    body: Any,
) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    raise Hi6ProtocolError(
        f"{label} is not a 0/1 flag",
        method=method,
        path=path,
        body=body,
    )


def _query_string(items: Sequence[Tuple[str, Optional[int]]]) -> str:
    values = [(key, value) for key, value in items if value is not None]
    return "?" + urlencode(values) if values else ""


def _validate_integer(value: Any, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise Hi6ValidationError(f"{name} must be an integer")


def _validate_int_range(
    value: Any,
    name: str,
    minimum: int,
    maximum: int,
) -> None:
    _validate_integer(value, name)
    if not minimum <= value <= maximum:
        raise Hi6ValidationError(
            f"{name} must be in the range {minimum}..{maximum}"
        )


def _validate_positive_finite(value: Any, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Hi6ValidationError(f"{name} must be a number")
    if not math.isfinite(float(value)) or value <= 0:
        raise Hi6ValidationError(f"{name} must be positive and finite")
