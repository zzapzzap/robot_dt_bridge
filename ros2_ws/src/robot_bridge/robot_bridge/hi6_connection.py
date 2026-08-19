"""
Read-only connection preflight for explicitly scoped Hi6 endpoints.

Hi6 does not publish an official controller-discovery protocol.  This module
therefore validates configured endpoints first and performs an optional,
bounded TCP-8888 search only when a caller supplies the exact IPv4 subnet.
Application-level validation consists solely of ``GET /api_ver`` followed by
``GET /versions/sysver`` through :class:`Hi6Client`.

No interface enumeration, subnet inference, port-range scan, POST, or PUT is
implemented here.  A caller must also resolve multiple discovered controllers;
the library deliberately reports that result as ambiguous.
"""

from __future__ import annotations

import ipaddress
import math
import socket
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from enum import Enum
from typing import (
    Any,
    Callable,
    Iterable,
    Mapping,
    Optional,
    Protocol,
    Sequence,
)

from .hi6_client import (
    Hi6Client,
    Hi6Config,
    Hi6ConnectionError,
    Hi6Error,
    Hi6HttpError,
    Hi6ProtocolError,
    Hi6TimeoutError,
    Hi6ValidationError,
)


__all__ = [
    "HI6_OPENAPI_PORT",
    "Hi6ConnectionConfigError",
    "Hi6Endpoint",
    "Hi6PreflightResult",
    "Hi6PreflightStatus",
    "Hi6ProbeResult",
    "Hi6ProbeStatus",
    "preflight_hi6_connections",
    "probe_hi6_endpoint",
]


HI6_OPENAPI_PORT = 8888
MAX_DISCOVERY_ADDRESSES = 256
MAX_DISCOVERY_WORKERS = 32


class Hi6ConnectionConfigError(ValueError):
    """A preflight request would exceed the deliberately narrow scan scope."""


class Hi6ProbeStatus(str, Enum):
    """Outcome of validating one endpoint with read-only OpenAPI calls."""

    VERIFIED = "verified"
    UNREACHABLE = "unreachable"
    HTTP_ERROR = "http_error"
    UNSUPPORTED_API = "unsupported_api"
    INVALID_RESPONSE = "invalid_response"


class Hi6PreflightStatus(str, Enum):
    """Overall outcome after configured-first preflight and optional search."""

    CONFIGURED = "configured"
    PARTIAL = "partial"
    DISCOVERED = "discovered"
    AMBIGUOUS = "ambiguous"
    NOT_FOUND = "not_found"


@dataclass(frozen=True, order=True)
class Hi6Endpoint:
    """One explicitly configured controller host and OpenAPI port."""

    host: str
    port: int = HI6_OPENAPI_PORT

    def __post_init__(self) -> None:
        """Normalize the host and reuse the OpenAPI client's validation."""
        if not isinstance(self.host, str):
            raise Hi6ConnectionConfigError("endpoint host must be a string")
        host = self.host.strip()
        try:
            Hi6Config(host=host, port=self.port)
        except Hi6ValidationError as exc:
            raise Hi6ConnectionConfigError(str(exc)) from exc
        object.__setattr__(self, "host", host)


@dataclass(frozen=True)
class Hi6ProbeResult:
    """Read-only identity result for one potential Hi6 endpoint."""

    endpoint: Hi6Endpoint
    status: Hi6ProbeStatus
    api_version: Optional[int] = None
    controller_version: Optional[str] = None
    error: Optional[str] = None

    @property
    def verified(self) -> bool:
        """Return whether both version endpoints matched the Hi6 contract."""
        return self.status is Hi6ProbeStatus.VERIFIED


@dataclass(frozen=True)
class Hi6PreflightResult:
    """Configured and discovered results without assigning controller roles."""

    status: Hi6PreflightStatus
    configured_results: tuple[Hi6ProbeResult, ...]
    discovered_results: tuple[Hi6ProbeResult, ...] = ()
    scanned_subnet: Optional[str] = None
    scanned_host_count: int = 0
    open_port_count: int = 0

    @property
    def candidates(self) -> tuple[Hi6ProbeResult, ...]:
        """Return every verified endpoint once, regardless of its source."""
        unique: dict[Hi6Endpoint, Hi6ProbeResult] = {}
        for result in self.configured_results + self.discovered_results:
            if result.verified:
                unique.setdefault(result.endpoint, result)
        return tuple(unique.values())

    @property
    def unique_candidate(self) -> Optional[Hi6ProbeResult]:
        """Return the only verified candidate, never choosing among several."""
        candidates = self.candidates
        return candidates[0] if len(candidates) == 1 else None


class _ReadableHi6Client(Protocol):
    """Small client surface required by the read-only verifier."""

    def get_api_version(self) -> int:
        """Return the OpenAPI schema version."""

    def get_system_version(self) -> dict[str, Any]:
        """Return the controller module version document."""

    def close(self) -> None:
        """Close the persistent HTTP session."""


ClientFactory = Callable[[Hi6Config], _ReadableHi6Client]
PortChecker = Callable[[str, int, float, Optional[str]], bool]


def probe_hi6_endpoint(
    endpoint: Hi6Endpoint,
    *,
    source_address: Optional[str] = None,
    supported_api_versions: Sequence[int] = (5,),
    connect_timeout_s: float = 0.5,
    read_timeout_s: float = 1.0,
    client_factory: ClientFactory = Hi6Client,
) -> Hi6ProbeResult:
    """Verify one endpoint using only the two documented version GET calls."""
    if not isinstance(endpoint, Hi6Endpoint):
        raise Hi6ConnectionConfigError("endpoint must be a Hi6Endpoint")
    supported = _validate_supported_versions(supported_api_versions)
    _validate_timeout(connect_timeout_s, "connect_timeout_s")
    _validate_timeout(read_timeout_s, "read_timeout_s")

    try:
        config = Hi6Config(
            host=endpoint.host,
            port=endpoint.port,
            source_address=source_address,
            connect_timeout_s=float(connect_timeout_s),
            read_timeout_s=float(read_timeout_s),
            max_response_bytes=64 * 1024,
            retry_safe_requests=False,
            user_agent="robot-dt-bridge-preflight/0.1",
        )
    except Hi6ValidationError as exc:
        raise Hi6ConnectionConfigError(str(exc)) from exc
    client = client_factory(config)
    try:
        api_version = client.get_api_version()
        if api_version not in supported:
            return Hi6ProbeResult(
                endpoint=endpoint,
                status=Hi6ProbeStatus.UNSUPPORTED_API,
                api_version=api_version,
                error=(
                    f"OpenAPI schema {api_version} is not in the allowed set "
                    f"{supported}"
                ),
            )

        system_version = client.get_system_version()
        controller_version = _extract_controller_version(system_version)
        if controller_version is None:
            return Hi6ProbeResult(
                endpoint=endpoint,
                status=Hi6ProbeStatus.INVALID_RESPONSE,
                api_version=api_version,
                error="system version has no non-empty 'com' module version",
            )
        return Hi6ProbeResult(
            endpoint=endpoint,
            status=Hi6ProbeStatus.VERIFIED,
            api_version=api_version,
            controller_version=controller_version,
        )
    except (Hi6TimeoutError, Hi6ConnectionError) as exc:
        return Hi6ProbeResult(
            endpoint=endpoint,
            status=Hi6ProbeStatus.UNREACHABLE,
            error=str(exc),
        )
    except Hi6HttpError as exc:
        return Hi6ProbeResult(
            endpoint=endpoint,
            status=Hi6ProbeStatus.HTTP_ERROR,
            error=str(exc),
        )
    except Hi6ProtocolError as exc:
        return Hi6ProbeResult(
            endpoint=endpoint,
            status=Hi6ProbeStatus.INVALID_RESPONSE,
            error=str(exc),
        )
    except Hi6Error as exc:
        return Hi6ProbeResult(
            endpoint=endpoint,
            status=Hi6ProbeStatus.INVALID_RESPONSE,
            error=str(exc),
        )
    finally:
        client.close()


def preflight_hi6_connections(
    configured_endpoints: Iterable[Hi6Endpoint] = (),
    *,
    scan_subnet: Optional[str] = None,
    source_address: Optional[str] = None,
    supported_api_versions: Sequence[int] = (5,),
    connect_timeout_s: float = 0.25,
    read_timeout_s: float = 1.0,
    scan_workers: int = 8,
    client_factory: ClientFactory = Hi6Client,
    port_checker: Optional[PortChecker] = None,
) -> Hi6PreflightResult:
    """
    Validate configured endpoints, then optionally search one bounded subnet.

    A subnet search occurs unless every configured endpoint verifies, and only
    when ``scan_subnet`` was explicitly supplied.  The subnet must be private,
    link-local, or loopback IPv4 and contain at most 256 total addresses.  Only
    TCP port 8888 is checked.  When ``source_address`` is supplied, the port
    checks and subsequent identity GETs both bind to that validated local
    IPv4 address.  Discovery never assigns a controller role automatically.
    """
    supported = _validate_supported_versions(supported_api_versions)
    _validate_timeout(connect_timeout_s, "connect_timeout_s")
    _validate_timeout(read_timeout_s, "read_timeout_s")
    workers = _validate_workers(scan_workers)
    network = _validate_scan_subnet(scan_subnet)
    try:
        source_address = Hi6Config(
            source_address=source_address
        ).source_address
    except Hi6ValidationError as exc:
        raise Hi6ConnectionConfigError(str(exc)) from exc
    endpoints = _deduplicate_endpoints(configured_endpoints)

    configured_results = tuple(
        probe_hi6_endpoint(
            endpoint,
            source_address=source_address,
            supported_api_versions=supported,
            connect_timeout_s=connect_timeout_s,
            read_timeout_s=read_timeout_s,
            client_factory=client_factory,
        )
        for endpoint in endpoints
    )
    configured_verified = tuple(
        result for result in configured_results if result.verified
    )
    if (
        configured_results
        and len(configured_verified) == len(configured_results)
    ):
        return Hi6PreflightResult(
            status=Hi6PreflightStatus.CONFIGURED,
            configured_results=configured_results,
        )

    if network is None:
        return Hi6PreflightResult(
            status=(
                Hi6PreflightStatus.PARTIAL
                if configured_verified
                else Hi6PreflightStatus.NOT_FOUND
            ),
            configured_results=configured_results,
        )

    hosts = tuple(str(address) for address in network.hosts())
    checker = port_checker or _tcp_port_open
    open_hosts = _find_openapi_hosts(
        hosts,
        timeout_s=float(connect_timeout_s),
        workers=workers,
        source_address=source_address,
        checker=checker,
    )
    discovered_results = tuple(
        probe_hi6_endpoint(
            Hi6Endpoint(host, HI6_OPENAPI_PORT),
            source_address=source_address,
            supported_api_versions=supported,
            connect_timeout_s=connect_timeout_s,
            read_timeout_s=read_timeout_s,
            client_factory=client_factory,
        )
        for host in open_hosts
    )
    verified_endpoints = {
        result.endpoint
        for result in configured_results + discovered_results
        if result.verified
    }
    if len(verified_endpoints) > 1:
        status = Hi6PreflightStatus.AMBIGUOUS
    elif len(verified_endpoints) == 1:
        status = (
            Hi6PreflightStatus.PARTIAL
            if configured_verified
            else Hi6PreflightStatus.DISCOVERED
        )
    else:
        status = Hi6PreflightStatus.NOT_FOUND

    return Hi6PreflightResult(
        status=status,
        configured_results=configured_results,
        discovered_results=discovered_results,
        scanned_subnet=str(network),
        scanned_host_count=len(hosts),
        open_port_count=len(open_hosts),
    )


def _deduplicate_endpoints(
    values: Iterable[Hi6Endpoint],
) -> tuple[Hi6Endpoint, ...]:
    try:
        endpoints = tuple(values)
    except TypeError as exc:
        raise Hi6ConnectionConfigError(
            "configured_endpoints must be iterable"
        ) from exc
    if any(not isinstance(endpoint, Hi6Endpoint) for endpoint in endpoints):
        raise Hi6ConnectionConfigError(
            "configured_endpoints must contain only Hi6Endpoint values"
        )
    return tuple(dict.fromkeys(endpoints))


def _validate_supported_versions(values: Sequence[int]) -> tuple[int, ...]:
    try:
        versions = tuple(values)
    except TypeError as exc:
        raise Hi6ConnectionConfigError(
            "supported_api_versions must be a non-empty sequence"
        ) from exc
    if not versions or any(
        isinstance(value, bool) or not isinstance(value, int) or value < 1
        for value in versions
    ):
        raise Hi6ConnectionConfigError(
            "supported_api_versions must contain positive integers"
        )
    return tuple(dict.fromkeys(versions))


def _validate_timeout(value: float, name: str) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or value <= 0
    ):
        raise Hi6ConnectionConfigError(f"{name} must be positive and finite")


def _validate_workers(value: int) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 1 <= value <= MAX_DISCOVERY_WORKERS
    ):
        raise Hi6ConnectionConfigError(
            f"scan_workers must be in the range 1..{MAX_DISCOVERY_WORKERS}"
        )
    return value


def _validate_scan_subnet(
    value: Optional[str],
) -> Optional[ipaddress.IPv4Network]:
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise Hi6ConnectionConfigError(
            "scan_subnet must be an explicit IPv4 network string"
        )
    try:
        network = ipaddress.ip_network(value.strip(), strict=True)
    except ValueError as exc:
        raise Hi6ConnectionConfigError(
            f"invalid scan_subnet {value!r}: {exc}"
        ) from exc
    if not isinstance(network, ipaddress.IPv4Network):
        raise Hi6ConnectionConfigError("scan_subnet must be IPv4")
    if network.num_addresses > MAX_DISCOVERY_ADDRESSES:
        raise Hi6ConnectionConfigError(
            "scan_subnet is too broad; use an IPv4 /24 or narrower network"
        )
    if not (
        network.is_private or network.is_link_local or network.is_loopback
    ):
        raise Hi6ConnectionConfigError(
            "scan_subnet must be private, link-local, or loopback"
        )
    if network.is_multicast or network.network_address.is_unspecified:
        raise Hi6ConnectionConfigError(
            "scan_subnet is not a usable host network"
        )
    return network


def _extract_controller_version(
    value: Mapping[str, Any],
) -> Optional[str]:
    modules = value.get("modules")
    if not isinstance(modules, list):
        return None
    for module in modules:
        if not isinstance(module, Mapping) or module.get("name") != "com":
            continue
        version = module.get("ver")
        if isinstance(version, str) and version.strip():
            return version.strip()
    return None


def _find_openapi_hosts(
    hosts: Sequence[str],
    *,
    timeout_s: float,
    workers: int,
    source_address: Optional[str],
    checker: PortChecker,
) -> tuple[str, ...]:
    if not hosts:
        return ()

    def check(host: str) -> bool:
        return bool(
            checker(host, HI6_OPENAPI_PORT, timeout_s, source_address)
        )

    with ThreadPoolExecutor(max_workers=min(workers, len(hosts))) as executor:
        results = executor.map(check, hosts)
        return tuple(host for host, is_open in zip(hosts, results) if is_open)


def _tcp_port_open(
    host: str,
    port: int,
    timeout_s: float,
    source_address: Optional[str],
) -> bool:
    try:
        source = (source_address, 0) if source_address is not None else None
        with socket.create_connection(
            (host, port),
            timeout=timeout_s,
            source_address=source,
        ):
            return True
    except OSError:
        return False
