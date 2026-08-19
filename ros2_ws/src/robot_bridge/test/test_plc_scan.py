"""Regression tests for non-disruptive known-host PLC scanning."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def _load_scan_module():
    root = Path(__file__).resolve().parents[4]
    path = root / "tools" / "plc_scan.py"
    spec = importlib.util.spec_from_file_location("plc_scan_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_known_host_identifies_without_sacrificial_tcp_connection(
    monkeypatch,
) -> None:
    scan = _load_scan_module()
    captured = []

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("known-host scan must not pre-open the PLC port")

    def probe(targets):
        captured.extend(targets)
        return []

    monkeypatch.setattr(scan, "tcp_open", fail_if_called)
    monkeypatch.setattr(scan, "probe_all", probe)
    monkeypatch.setattr(scan, "report", lambda _hits: 0)
    monkeypatch.setattr(
        sys,
        "argv",
        ["plc_scan.py", "--host", "192.168.10.30", "--ports", "9000,9001"],
    )

    assert scan.main() == 0
    assert captured == [
        ("192.168.10.30", 9000),
        ("192.168.10.30", 9001),
    ]


def test_field_ports_are_in_default_candidate_list() -> None:
    scan = _load_scan_module()
    assert 9000 in scan.CANDIDATE_PORTS
    assert 9001 in scan.CANDIDATE_PORTS
