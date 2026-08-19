from pathlib import Path

import pytest

from robot_bridge.config_loader import Hi6Config


def _write_config(directory: Path, hi6_text: str) -> None:
    (directory / "network.yaml").write_text(
        "segments:\n"
        "  hi6_control:\n"
        "    subnet: 192.168.250.0/24\n"
        "    hosts:\n"
        "      jetson: 192.168.250.10\n",
        encoding="utf-8",
    )
    (directory / "hi6.yaml").write_text(hi6_text, encoding="utf-8")


def _schema(
    instances: str,
    *,
    model: str = "",
    simulation: str = "",
) -> str:
    return f"""
model:
  name: test_hi6
{model}
simulation:
  host: 127.0.0.1
  port_base: 18888
{simulation}
instances:
{instances}
"""


def test_hi6_config_does_not_require_legacy_plc_file(tmp_path: Path) -> None:
    _write_config(
        tmp_path,
        _schema(
            """  loading:
    host: 192.168.250.21"""
        ),
    )

    assert not (tmp_path / "plc.yaml").exists()
    config = Hi6Config.load(str(tmp_path))
    instance = config.instance("loading")
    assert instance.rest_port == 8888
    assert instance.pose_hz == 20.0
    assert instance.status_hz == 5.0
    assert instance.status_publish_hz == 20.0
    assert config.simulation.host == "127.0.0.1"
    assert config.simulation.port_for_index(0) == 18888
    assert config.simulation.pose_hz == 30.0
    assert config.simulation.status_hz == 5.0
    assert config.simulation.status_publish_hz == 30.0


def test_model_is_shared_by_instances_and_aliases_remain_available(
    tmp_path: Path,
) -> None:
    _write_config(
        tmp_path,
        _schema(
            """  loading:
    host: 192.168.250.21
  unloading:
    host: 192.168.250.22
    enabled: false
    visualization_base_xyz_m: [0, 1.5, 0]
    visualization_base_rpy_rad: [0, 0, 3.1415926536]""",
            model="""  rest_port: 8888
  stream_port: 49000
  connect_timeout_s: 1.0
  read_timeout_s: 2.0
  pose_hz: 10
  status_hz: 5
  status_publish_hz: 20
  stale_timeout_ms: 500
  verify_timeout_s: 3
  allowed_speed_percent: [100, 25, 50, 50]
  supported_api_versions: [5]
  axis_names: [J1, J2, J3, J4, J5, J6]
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]""",
        ),
    )

    config = Hi6Config.load(str(tmp_path))
    loading = config.instance("loading")
    unloading = config.instance("unloading")

    assert loading.model is config.model
    assert unloading.model is config.model
    assert loading.host == "192.168.250.21"
    assert unloading.host == "192.168.250.22"
    assert loading.rest_port == unloading.rest_port == 8888
    assert loading.pose_hz == unloading.pose_hz == 10
    assert loading.allowed_speed_percent == [25, 50, 100]
    assert loading.supported_api_versions == [5]
    assert loading.axis_names == ["J1", "J2", "J3", "J4", "J5", "J6"]
    assert loading.joint_names == [
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6",
    ]
    assert unloading.visualization_base_xyz_m == [0.0, 1.5, 0.0]
    assert unloading.visualization_base_rpy_rad == [0.0, 0.0, 3.1415926536]
    assert [item.robot_id for item in config.enabled_instances()] == [
        "loading"
    ]
    assert config.enabled_robots() == config.enabled_instances()
    assert config.robot("loading") is config.instance("loading")
    assert config.robots is config.instances
    assert config.network["subnet"] == "192.168.250.0/24"


@pytest.mark.parametrize(
    "bad_line",
    [
        "  rest_port: 70000",
        "  pose_hz: 0",
        "  status_hz: 0",
        "  status_publish_hz: 0",
        "  allowed_speed_percent: [0, 50]",
        "  supported_api_versions: [0]",
        "  axis_names: [J1, J2]",
        "  joint_names: [joint_1, joint_2]",
    ],
)
def test_hi6_rejects_unsafe_model_values(
    tmp_path: Path,
    bad_line: str,
) -> None:
    _write_config(
        tmp_path,
        _schema(
            """  loading:
    host: 192.168.250.21""",
            model=bad_line,
        ),
    )

    with pytest.raises(ValueError):
        Hi6Config.load(str(tmp_path))


@pytest.mark.parametrize(
    "bad_line",
    [
        "  host: 192.168.250.21",
        "  port_base: 0",
        "  pose_hz: 0",
        "  status_hz: 0",
        "  status_publish_hz: 0",
    ],
)
def test_hi6_rejects_unsafe_simulation_values(
    tmp_path: Path,
    bad_line: str,
) -> None:
    _write_config(
        tmp_path,
        _schema(
            """  loading:
    host: 192.168.250.21""",
            simulation=bad_line,
        ),
    )

    with pytest.raises(ValueError):
        Hi6Config.load(str(tmp_path))


@pytest.mark.parametrize(
    "forbidden_line",
    [
        "    rest_port: 18888",
        "    joint_names: [a, b, c, d, e, f]",
        "    allow_commands: true",
        "    allow_start: true",
        "    unexpected: value",
    ],
)
def test_production_instance_rejects_model_policy_and_unknown_fields(
    tmp_path: Path,
    forbidden_line: str,
) -> None:
    _write_config(
        tmp_path,
        _schema(
            f"""  loading:
    host: 192.168.250.21
{forbidden_line}"""
        ),
    )

    with pytest.raises(ValueError, match="model/policy"):
        Hi6Config.load(str(tmp_path))


def test_production_instances_are_always_fail_closed_by_default(
    tmp_path: Path,
) -> None:
    _write_config(
        tmp_path,
        _schema(
            """  loading:
    host: 192.168.250.21"""
        ),
    )

    instance = Hi6Config.load(str(tmp_path)).instance("loading")
    assert instance.allow_commands is False
    assert instance.allow_speed_increase is False
    assert instance.allow_start is False
    assert instance.allow_unverified_start is False


def test_quoted_false_enabled_remains_false(tmp_path: Path) -> None:
    _write_config(
        tmp_path,
        _schema(
            """  loading:
    enabled: "false"
    host: 192.168.250.21"""
        ),
    )

    config = Hi6Config.load(str(tmp_path))
    assert config.instance("loading").enabled is False
    assert config.enabled_instances() == []


def test_invalid_instance_boolean_is_rejected(tmp_path: Path) -> None:
    _write_config(
        tmp_path,
        _schema(
            """  loading:
    enabled: definitely
    host: 192.168.250.21"""
        ),
    )

    with pytest.raises(ValueError, match="enabled"):
        Hi6Config.load(str(tmp_path))


def test_duplicate_enabled_instance_endpoints_are_rejected(
    tmp_path: Path,
) -> None:
    _write_config(
        tmp_path,
        _schema(
            """  loading:
    host: " 192.168.250.21 "
  unloading:
    host: 192.168.250.21"""
        ),
    )

    with pytest.raises(ValueError, match="duplicate enabled Hi6 endpoint"):
        Hi6Config.load(str(tmp_path))


def test_simulation_port_range_covers_every_instance(tmp_path: Path) -> None:
    _write_config(
        tmp_path,
        _schema(
            """  loading:
    host: 192.168.250.21
  unloading:
    host: 192.168.250.22""",
            simulation="  port_base: 65535",
        ),
    )

    with pytest.raises(ValueError, match="65535"):
        Hi6Config.load(str(tmp_path))


def test_new_and_legacy_schema_cannot_be_mixed(tmp_path: Path) -> None:
    _write_config(
        tmp_path,
        _schema(
            """  loading:
    host: 192.168.250.21"""
        ) + "\nrobots: {}\n",
    )

    with pytest.raises(ValueError, match="섞을 수 없습니다"):
        Hi6Config.load(str(tmp_path))


def test_legacy_schema_remains_readable_for_existing_internal_tests(
    tmp_path: Path,
) -> None:
    _write_config(
        tmp_path,
        """
defaults:
  pose_hz: 10
  allow_commands: false
robots:
  loading:
    host: 192.168.250.21
    allow_commands: true
  unloading:
    host: 192.168.250.22
    rest_port: 18888
    enabled: false
""",
    )

    config = Hi6Config.load(str(tmp_path))
    assert config.instance("loading").pose_hz == 10
    assert config.instance("loading").allow_commands is True
    assert config.instance("unloading").rest_port == 18888
    assert config.instance("unloading").enabled is False


def test_explicit_config_directory_does_not_fallback(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="config_dir"):
        Hi6Config.load(str(tmp_path))
