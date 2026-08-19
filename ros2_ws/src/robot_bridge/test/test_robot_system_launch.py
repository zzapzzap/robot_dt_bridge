"""Tests for the deliberately small robot_system launch interface."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.utilities import perform_substitutions


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_DIR / "launch" / "robot_system.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "robot_system_launch_test", LAUNCH_FILE
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _include_arguments(actions) -> dict[str, str]:
    include = next(
        action for action in actions
        if isinstance(action, IncludeLaunchDescription)
    )
    return dict(include.launch_arguments)


def test_top_level_exposes_only_strict_sim_and_debug() -> None:
    module = _load_launch_module()
    description = module.generate_launch_description()
    arguments = [
        entity for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]

    assert [argument.name for argument in arguments] == ["sim", "debug"]
    assert all(argument.choices == ["true", "false"] for argument in arguments)
    defaults = {
        argument.name: perform_substitutions(
            LaunchContext(), argument.default_value
        )
        for argument in arguments
    }
    assert defaults == {"sim": "true", "debug": "true"}


@pytest.mark.parametrize(
    ("sim", "debug", "bad_name"),
    [
        ("yes", "false", "sim"),
        ("false", "1", "debug"),
        ("TRUE", "false", "sim"),
    ],
)
def test_top_level_rejects_non_literal_booleans(
    monkeypatch, sim: str, debug: str, bad_name: str
) -> None:
    module = _load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda _package: str(PACKAGE_DIR),
    )
    context = LaunchContext()
    context.launch_configurations.update({"sim": sim, "debug": debug})

    with pytest.raises(ValueError, match=bad_name):
        module._include_plc_launch(context)


def test_sim_delegates_to_plc_sim_with_unity(
    monkeypatch,
) -> None:
    module = _load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda _package: str(PACKAGE_DIR),
    )

    context = LaunchContext()
    context.launch_configurations.update({"sim": "true", "debug": "true"})

    actions = module._include_plc_launch(context)
    source = actions[0].launch_description_source
    source.get_launch_description(context)
    assert "plc_bringup.launch.py" in source.location
    assert _include_arguments(actions) == {
        "profile": "sim",
        "debug": "true",
        "with_unity": "true",
    }


def test_field_delegates_preflight_and_preserves_command_policy(
    monkeypatch,
) -> None:
    module = _load_launch_module()
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda _package: str(PACKAGE_DIR),
    )
    context = LaunchContext()
    context.launch_configurations.update({"sim": "false", "debug": "false"})

    actions = module._include_plc_launch(context)
    assert len(actions) == 1
    assert _include_arguments(actions) == {
        "profile": "field",
        "debug": "false",
        "with_unity": "true",
    }
