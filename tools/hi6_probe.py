#!/usr/bin/env python3
"""
Read-only capability probe for one HD Hyundai Hi6 controller.

This tool never sends POST or PUT requests.  It can therefore be used before
the ROS node is enabled to prove the controller address, Open API version,
joint ordering, and the status fields needed by the bridge.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "robot_bridge"))

from robot_bridge.config_loader import Hi6Config as Hi6BridgeConfig  # noqa: E402
from robot_bridge.hi6_client import (  # noqa: E402
    Hi6Client,
    Hi6Config as Hi6HttpConfig,
    Hi6Error,
)


def _configured_instance(robot_id: str):
    """Load one resolved instance through the canonical config parser."""
    try:
        return Hi6BridgeConfig.load(str(ROOT / "config")).instance(robot_id)
    except (FileNotFoundError, KeyError):
        return None


def _system_version_summary(version: dict) -> str:
    modules = version.get("modules") or []
    values = []
    for module in modules:
        if not isinstance(module, dict):
            continue
        name = module.get("name") or module.get("module") or "module"
        value = module.get("version") or module.get("ver") or "?"
        values.append(f"{name}={value}")
    return ", ".join(values) if values else json.dumps(version, ensure_ascii=False)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Hi6 Open API read-only capability probe"
    )
    parser.add_argument(
        "--robot",
        default="loading",
        help="robot id in config/hi6.yaml (default: loading)",
    )
    parser.add_argument("--host", help="override controller IP or hostname")
    parser.add_argument("--port", type=int, help="override Open API port")
    parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="connect/read timeout seconds",
    )
    arguments = parser.parse_args()

    configured = _configured_instance(arguments.robot)
    host = arguments.host or (
        configured.host if configured is not None else None
    )
    port = arguments.port or (
        configured.rest_port if configured is not None else 8888
    )
    if not host:
        parser.error(
            "--host is required when the instance is absent from config/hi6.yaml"
        )

    connect_timeout = float(
        arguments.timeout
        if arguments.timeout is not None
        else (
            configured.connect_timeout_s if configured is not None else 1.0
        )
    )
    read_timeout = float(
        arguments.timeout
        if arguments.timeout is not None
        else configured.read_timeout_s if configured is not None else 1.0
    )
    client = Hi6Client(
        Hi6HttpConfig(
            host=str(host),
            port=port,
            connect_timeout_s=connect_timeout,
            read_timeout_s=read_timeout,
        )
    )

    print(f"Hi6 read-only probe: {arguments.robot} at {host}:{port}")
    print("No controller write requests will be sent.")
    try:
        api_version = client.get_api_version()
        print(f"  OK  Open API version: {api_version}")
        supported_versions = set(
            configured.supported_api_versions
            if configured is not None
            else [5]
        )
        if api_version not in supported_versions:
            print(
                "  FAIL  Open API schema is not in the reviewed allowlist: "
                f"{sorted(supported_versions)}",
                file=sys.stderr,
            )
            return 1

        system_version = client.get_system_version()
        print(f"  OK  System version: {_system_version_summary(system_version)}")

        joints = client.get_joint_positions(6, mechinfo=1)
        formatted = ", ".join(
            f"J{index}={value:.3f} deg"
            for index, value in enumerate(joints, start=1)
        )
        print(f"  OK  Joint pose: {formatted}")

        status = client.get_status()
        motor_state = client.get_motor_state()
        print(
            "  OK  Status: "
            f"remote={status['is_remote_mode']} "
            f"motor_code={int(motor_state)} "
            f"playback={status['is_playback']} "
            f"reported_speed={status['playback_speed_percent']}% "
            f"last_error={status['last_error_id']}"
        )

        emergency_stop = client.get_emergency_stop()
        print(f"  OK  Emergency-stop readback: active={emergency_stop}")

        configured_speed = client.get_playback_speed_percent()
        print(f"  OK  Configured playback speed: {configured_speed}%")
    except Hi6Error as error:
        print(f"  FAIL  {error}", file=sys.stderr)
        print(
            "Check controller model/firmware, Open API enablement, REMOTE/user-LAN "
            "settings, cable, IP, and TCP port 8888.",
            file=sys.stderr,
        )
        return 1
    finally:
        client.close()

    print("PASS: all read-only endpoints required by the initial bridge responded.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
