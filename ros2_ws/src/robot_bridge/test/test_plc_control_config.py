"""Strict configuration contract for photographed PLC rows No.9~17."""

from __future__ import annotations

import copy
import shutil
from pathlib import Path
from typing import Any, Callable

import pytest
import yaml

from robot_bridge.config_loader import BridgeConfig, PlcBridgeConfig


PACKAGE_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_CONFIG = PACKAGE_DIR.parents[2] / "config"


def _mutated_config(
    tmp_path: Path,
    mutate: Callable[[dict[str, Any]], None],
) -> Path:
    """Copy the two strict config documents and mutate only robots.yaml."""
    shutil.copy2(REPOSITORY_CONFIG / "plc.yaml", tmp_path / "plc.yaml")
    document = yaml.safe_load(
        (REPOSITORY_CONFIG / "robots.yaml").read_text(encoding="utf-8")
    )
    mutate(document)
    (tmp_path / "robots.yaml").write_text(
        yaml.safe_dump(document, sort_keys=False),
        encoding="utf-8",
    )
    return tmp_path


def _direct(document: dict[str, Any]) -> dict[str, Any]:
    return document["robots"][0]["plc_map"]["direct_controls"]


@pytest.mark.parametrize("profile", ["field", "sim"])
def test_no9_to_no17_contract_is_exact_and_not_actual_feedback(
    profile: str,
) -> None:
    config = PlcBridgeConfig.load(str(REPOSITORY_CONFIG), profile=profile)
    instance = config.instance("loading")
    controls = instance.direct_controls

    assert {
        name: (control.device, control.active_value)
        for name, control in controls.speed.items()
    } == {
        "speed_25": ("D1016", 25),
        "speed_50": ("D1018", 50),
        "speed_75": ("D1020", 75),
    }
    assert controls.control_word_device == "D1100"
    assert controls.control_bits == {
        "hold": 0,
        "emergency_stop": 1,
        "fault_reset": 2,
        "device_home": 3,
        "robot_home": 4,
        "standby": 5,
    }
    assert controls.writable_controls == frozenset(
        {
            "speed_25",
            "speed_50",
            "speed_75",
            "hold",
            "fault_reset",
            "device_home",
            "robot_home",
            "standby",
        }
    )
    assert "emergency_stop" not in controls.writable_controls
    assert controls.action_pulse_seconds == 0.25

    assert instance.command_map_verified is True
    assert instance.actual_feedback_available is False
    assert instance.ack_available is False
    # Compatibility aliases retain their API but no longer conflate the two.
    assert instance.map_verified is True
    assert instance.feedback_available is False


def test_legacy_sim_blocks_remain_separate_from_direct_controls() -> None:
    instance = PlcBridgeConfig.load(
        str(REPOSITORY_CONFIG), profile="sim"
    ).instance("loading")

    assert instance.motion_command.device == "D2000"
    assert instance.speed_command.device == "D3000"
    assert {item.device for item in instance.direct_controls.speed.values()} == {
        "D1016",
        "D1018",
        "D1020",
    }


@pytest.mark.parametrize(
    ("name", "field", "bad_value"),
    [
        ("speed_25", "device", "D1015"),
        ("speed_50", "device", "D1019"),
        ("speed_75", "device", "D1021"),
        ("speed_25", "active_value", 24),
        ("speed_50", "active_value", 25),
        ("speed_75", "active_value", 50),
    ],
)
def test_speed_contract_rejects_wrong_address_or_value(
    tmp_path: Path,
    name: str,
    field: str,
    bad_value: Any,
) -> None:
    directory = _mutated_config(
        tmp_path,
        lambda document: _direct(document)["speed"][name].__setitem__(
            field, bad_value
        ),
    )

    with pytest.raises(ValueError, match="No.9~11"):
        PlcBridgeConfig.load(str(directory), profile="field")


def test_duplicate_speed_devices_are_rejected_before_use(tmp_path: Path) -> None:
    def mutate(document: dict[str, Any]) -> None:
        _direct(document)["speed"]["speed_50"]["device"] = "D1016"

    directory = _mutated_config(tmp_path, mutate)

    with pytest.raises(ValueError, match="device는 서로 달라야"):
        PlcBridgeConfig.load(str(directory), profile="field")


@pytest.mark.parametrize(
    ("field", "bad_value"),
    [
        ("device", "D1101"),
        ("bits", {"hold": 1, "emergency_stop": 0, "fault_reset": 2,
                  "device_home": 3, "robot_home": 4, "standby": 5}),
    ],
)
def test_control_word_rejects_wrong_device_or_bits(
    tmp_path: Path,
    field: str,
    bad_value: Any,
) -> None:
    directory = _mutated_config(
        tmp_path,
        lambda document: _direct(document)["control_word"].__setitem__(
            field, copy.deepcopy(bad_value)
        ),
    )

    with pytest.raises(ValueError, match="No.12~17"):
        PlcBridgeConfig.load(str(directory), profile="field")


def test_emergency_stop_can_never_enter_write_allowlist(tmp_path: Path) -> None:
    def mutate(document: dict[str, Any]) -> None:
        _direct(document)["writable"]["field"].append("emergency_stop")

    directory = _mutated_config(tmp_path, mutate)

    with pytest.raises(ValueError, match="쓰기 금지"):
        PlcBridgeConfig.load(str(directory), profile="field")


def test_profile_selects_only_its_write_allowlist(tmp_path: Path) -> None:
    def mutate(document: dict[str, Any]) -> None:
        _direct(document)["writable"]["field"] = ["speed_50", "hold"]

    directory = _mutated_config(tmp_path, mutate)

    field_controls = PlcBridgeConfig.load(
        str(directory), profile="field"
    ).instance("loading").direct_controls
    sim_controls = PlcBridgeConfig.load(
        str(directory), profile="sim"
    ).instance("loading").direct_controls

    assert field_controls.writable_controls == frozenset(
        {"speed_50", "hold"}
    )
    assert "standby" in sim_controls.writable_controls


@pytest.mark.parametrize("bad_value", [0.0, 0.09, 5.01, float("nan")])
def test_action_pulse_must_be_finite_and_in_safe_config_range(
    tmp_path: Path,
    bad_value: float,
) -> None:
    directory = _mutated_config(
        tmp_path,
        lambda document: _direct(document).__setitem__(
            "action_pulse_seconds", bad_value
        ),
    )

    with pytest.raises(ValueError, match="action_pulse_seconds"):
        PlcBridgeConfig.load(str(directory), profile="field")


def test_unknown_direct_control_key_is_rejected(tmp_path: Path) -> None:
    directory = _mutated_config(
        tmp_path,
        lambda document: _direct(document).__setitem__("unsafe", True),
    )

    with pytest.raises(ValueError, match="알 수 없는 항목: unsafe"):
        PlcBridgeConfig.load(str(directory), profile="field")


def test_axis_raw_uses_two_decimal_places() -> None:
    """PLC/controller coordinates retain their original signs."""
    instance = PlcBridgeConfig.load(
        str(REPOSITORY_CONFIG), profile="field"
    ).instance("loading")

    degrees, clamped = instance.to_degrees(
        [-12354, 100, 100, 100, 100, 100]
    )

    assert instance.scale == [0.01] * 6
    assert instance.dir == [1, 1, 1, 1, 1, 1]
    assert degrees == pytest.approx([-123.54, 1.0, 1.0, 1.0, 1.0, 1.0])
    assert clamped is False

    visual, visual_clamped = instance.to_visual_degrees(degrees)
    assert instance.visual_dir == [-1, 1, -1, -1, 1, -1]
    assert instance.visual_offset == [0.0, -90.0, 0.0, 0.0, 0.0, 0.0]
    assert visual == pytest.approx([123.54, -89.0, -1.0, -1.0, 1.0, -1.0])
    assert visual_clamped is False


def test_sim_axis_direction_preserves_input_signs() -> None:
    """Simulation publishes controller axes unchanged and maps only visuals."""
    instance = PlcBridgeConfig.load(
        str(REPOSITORY_CONFIG), profile="sim"
    ).instance("loading")

    degrees, clamped = instance.to_degrees(
        [3856, 13625, -4948, 17, -8685, -5068]
    )

    assert instance.scale == [0.01] * 6
    assert instance.dir == [1, 1, 1, 1, 1, 1]
    assert degrees == pytest.approx(
        [38.56, 136.25, -49.48, 0.17, -86.85, -50.68]
    )
    assert clamped is False

    visual, visual_clamped = instance.to_visual_degrees(degrees)
    assert visual == pytest.approx(
        [-38.56, 46.25, 49.48, -0.17, -86.85, 50.68]
    )
    assert visual_clamped is False


@pytest.mark.parametrize(
    "profile",
    ["field", "sim"],
)
def test_legacy_loader_exposes_controller_and_visual_transforms(
    profile: str,
) -> None:
    instance = BridgeConfig.load(
        str(REPOSITORY_CONFIG), profile=profile
    ).robot("loading")

    assert instance.dir == [1, 1, 1, 1, 1, 1]
    assert instance.visual_dir == [-1, 1, -1, -1, 1, -1]
    assert instance.visual_offset == [0.0, -90.0, 0.0, 0.0, 0.0, 0.0]


def test_flat_legacy_robot_config_defaults_visual_transform_to_identity(
    tmp_path: Path,
) -> None:
    def mutate(document: dict[str, Any]) -> None:
        robot = document["robots"][0]
        robot["dir"] = [1, 1, 1, 1, 1, 1]
        robot.pop("visual_dir")
        robot.pop("visual_offset")

    directory = _mutated_config(tmp_path, mutate)
    instance = PlcBridgeConfig.load(
        str(directory), profile="field"
    ).instance("loading")

    assert instance.visual_dir == [1, 1, 1, 1, 1, 1]
    assert instance.visual_offset == [0.0] * 6
    assert instance.to_visual_degrees([1, 2, 3, 4, 5, 6]) == (
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        False,
    )
