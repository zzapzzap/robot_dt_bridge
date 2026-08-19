"""config/*.yaml 을 읽어 노드들이 공유하는 설정 객체로 만든다.

설정 파일 위치는 다음 순서로 찾는다.
  1) ROS 파라미터 `config_dir`
  2) 환경변수 `ROBOT_DT_CONFIG`
  3) 패키지 share/config
  4) 저장소 루트의 config/   (소스 트리에서 바로 실행할 때)
"""

from __future__ import annotations

import ipaddress
import math
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, FrozenSet, List, Mapping, Optional, Sequence, Tuple

import yaml

from .mc_client import McConfig, parse_device


def find_config_dir(
    explicit: Optional[str] = None,
    required_file: str = "plc.yaml",
) -> Path:
    candidates: List[Path] = []
    if explicit:
        directory = Path(explicit)
        if (directory / required_file).is_file():
            return directory
        raise FileNotFoundError(
            f"명시적 config_dir에 {required_file} 이 없습니다: {directory}"
        )
    environment_dir = os.environ.get("ROBOT_DT_CONFIG")
    if environment_dir:
        directory = Path(environment_dir)
        if (directory / required_file).is_file():
            return directory
        raise FileNotFoundError(
            f"ROBOT_DT_CONFIG에 {required_file} 이 없습니다: {directory}"
        )
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory("robot_bridge")) / "config")
    except Exception:                      # ament 미설치 환경(단독 테스트)
        pass
    here = Path(__file__).resolve()
    for up in here.parents:
        candidates.append(up / "config")
    for c in candidates:
        if (c / required_file).is_file():
            return c
    raise FileNotFoundError(
        f"{required_file} 을 찾을 수 없습니다. ROBOT_DT_CONFIG 환경변수로 "
        "config 디렉터리를 지정하세요."
    )


def _load(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


# ---------------------------------------------------------------- 로봇 정의
@dataclass
class AxisMap:
    name: str
    offset: int
    type: str            # "word" | "dword"


@dataclass
class RobotDef:
    id: str
    label: str
    enabled: bool
    axis_names: List[str]
    calibrated: bool
    scale: List[float]
    dir: List[int]
    offset: List[float]
    visual_dir: List[int]
    visual_offset: List[float]
    limits_min: List[float]
    limits_max: List[float]
    topics: Dict[str, str]

    @classmethod
    def from_dict(cls, d: dict, profile: str = "field") -> "RobotDef":
        lim = d.get("limits_deg") or {}
        n = len(d.get("axis_names", [])) or 6
        directions = _profile_directions(
            d.get("dir", [1] * n),
            profile,
            f"robot {d.get('id', '<unknown>')}.dir",
        )
        visual_directions = _profile_directions(
            d.get("visual_dir", [1] * n),
            profile,
            f"robot {d.get('id', '<unknown>')}.visual_dir",
        )
        visual_offsets = _profile_offsets(
            d.get("visual_offset", [0.0] * n),
            profile,
            f"robot {d.get('id', '<unknown>')}.visual_offset",
        )
        return cls(
            id=d["id"],
            label=d.get("label", d["id"]),
            enabled=bool(d.get("enabled", True)),
            axis_names=list(d.get("axis_names", [f"J{i+1}" for i in range(n)])),
            calibrated=bool(d.get("calibrated", False)),
            scale=[float(x) for x in d.get("scale", [0.001] * n)],
            dir=directions,
            offset=[float(x) for x in d.get("offset", [0.0] * n)],
            visual_dir=visual_directions,
            visual_offset=visual_offsets,
            limits_min=[float(x) for x in lim.get("min", [-360.0] * n)],
            limits_max=[float(x) for x in lim.get("max", [360.0] * n)],
            topics=dict(d.get("topics", {})),
        )

    def to_degrees(self, raw: List[int]) -> tuple[List[float], bool]:
        """raw → degree 변환. (각도, 범위초과여부) 를 돌려준다."""
        out: List[float] = []
        clamped = False
        for i, r in enumerate(raw):
            deg = r * self.scale[i] * self.dir[i] + self.offset[i]
            lo, hi = self.limits_min[i], self.limits_max[i]
            if deg < lo:
                deg, clamped = lo, True
            elif deg > hi:
                deg, clamped = hi, True
            out.append(deg)
        return out, clamped

    def to_visual_degrees(
        self,
        controller_degrees: Sequence[float],
    ) -> tuple[List[float], bool]:
        """Controller axis coordinates -> CAD/URDF joint coordinates."""
        return _to_visual_degrees(
            controller_degrees,
            self.visual_dir,
            self.visual_offset,
            self.limits_min,
            self.limits_max,
            f"robot '{self.id}'",
        )


# ------------------------------------------------------- Hi6 direct connection
_HI6_ROOT_KEYS = {"model", "simulation", "instances"}
_HI6_MODEL_KEYS = {
    "name",
    "rest_port",
    "stream_port",
    "connect_timeout_s",
    "read_timeout_s",
    "pose_hz",
    "status_hz",
    "status_publish_hz",
    "stale_timeout_ms",
    "verify_timeout_s",
    "allowed_speed_percent",
    "supported_api_versions",
    "axis_names",
    "joint_names",
}
_HI6_SIMULATION_KEYS = {
    "host",
    "port_base",
    "pose_hz",
    "status_hz",
    "status_publish_hz",
}
_HI6_INSTANCE_KEYS = {
    "enabled",
    "host",
    "visualization_base_xyz_m",
    "visualization_base_rpy_rad",
}
_HI6_LEGACY_POLICY_KEYS = {
    "allow_commands",
    "allow_speed_increase",
    "allow_start",
    "allow_unverified_start",
}


@dataclass(frozen=True)
class Hi6ModelConfig:
    """Configuration shared by every identical Hi6 controller instance."""

    name: str
    axis_names: List[str]
    joint_names: List[str]
    rest_port: int
    stream_port: int
    connect_timeout_s: float
    read_timeout_s: float
    pose_hz: float
    status_hz: float
    status_publish_hz: float
    stale_timeout_ms: int
    verify_timeout_s: float
    allowed_speed_percent: List[int]
    supported_api_versions: List[int]


@dataclass(frozen=True)
class Hi6SimulationConfig:
    """Loopback mock endpoint and rates for pose and service-flow tests."""

    host: str
    port_base: int
    pose_hz: float
    status_hz: float
    status_publish_hz: float

    def port_for_index(self, index: int) -> int:
        if index < 0:
            raise ValueError("simulation instance index must be non-negative")
        port = self.port_base + index
        if port > 65535:
            raise ValueError("simulation instance port exceeds 65535")
        return port


@dataclass(frozen=True)
class Hi6InstanceConfig:
    """One physical role using the shared Hi6 model configuration."""

    robot_id: str
    enabled: bool
    host: str
    visualization_base_xyz_m: List[float]
    visualization_base_rpy_rad: List[float]
    model: Hi6ModelConfig
    # Compatibility for legacy test/config files only.  The production
    # model/simulation/instances schema cannot set command policy per robot.
    allow_commands: bool = False
    allow_speed_increase: bool = False
    allow_start: bool = False
    allow_unverified_start: bool = False

    @property
    def axis_names(self) -> List[str]:
        return self.model.axis_names

    @property
    def joint_names(self) -> List[str]:
        return self.model.joint_names

    @property
    def rest_port(self) -> int:
        return self.model.rest_port

    @property
    def stream_port(self) -> int:
        return self.model.stream_port

    @property
    def connect_timeout_s(self) -> float:
        return self.model.connect_timeout_s

    @property
    def read_timeout_s(self) -> float:
        return self.model.read_timeout_s

    @property
    def pose_hz(self) -> float:
        return self.model.pose_hz

    @property
    def status_hz(self) -> float:
        return self.model.status_hz

    @property
    def status_publish_hz(self) -> float:
        return self.model.status_publish_hz

    @property
    def stale_timeout_ms(self) -> int:
        return self.model.stale_timeout_ms

    @property
    def verify_timeout_s(self) -> float:
        return self.model.verify_timeout_s

    @property
    def allowed_speed_percent(self) -> List[int]:
        return self.model.allowed_speed_percent

    @property
    def supported_api_versions(self) -> List[int]:
        return self.model.supported_api_versions


# Kept while downstream code migrates from the old robot terminology.
Hi6RobotConfig = Hi6InstanceConfig


@dataclass(frozen=True)
class Hi6Config:
    """Validated same-model controller instances and isolated network data."""

    config_dir: Path
    model: Hi6ModelConfig
    simulation: Hi6SimulationConfig
    instances: Dict[str, Hi6InstanceConfig]
    network: Dict[str, Any]

    @classmethod
    def load(cls, config_dir: Optional[str] = None) -> "Hi6Config":
        directory = find_config_dir(config_dir, required_file="hi6.yaml")
        path = directory / "hi6.yaml"
        if not path.is_file():
            raise FileNotFoundError(f"Hi6 설정 파일이 없습니다: {path}")

        data = _load(path)
        if not isinstance(data, dict):
            raise ValueError("hi6.yaml 최상위 값은 mapping 이어야 합니다")
        network_path = directory / "network.yaml"
        network_doc = _load(network_path) if network_path.is_file() else {}
        network = ((network_doc.get("segments") or {}).get("hi6_control") or {})

        uses_new_schema = bool(_HI6_ROOT_KEYS.intersection(data))
        uses_legacy_schema = "defaults" in data or "robots" in data
        if uses_new_schema and uses_legacy_schema:
            raise ValueError(
                "hi6.yaml 에 model/instances 와 defaults/robots 를 섞을 수 없습니다"
            )
        if uses_legacy_schema:
            model, simulation, instances = _parse_legacy_hi6(data)
        else:
            model, simulation, instances = _parse_hi6(data)

        _validate_unique_enabled_hi6_endpoints(instances)
        if simulation.port_base + len(instances) - 1 > 65535:
            raise ValueError(
                "simulation.port_base 에 instance 수를 더하면 65535를 넘습니다"
            )
        return cls(
            config_dir=directory,
            model=model,
            simulation=simulation,
            instances=instances,
            network=dict(network),
        )

    @property
    def robots(self) -> Dict[str, Hi6InstanceConfig]:
        """Compatibility alias for callers not yet renamed to instances."""
        return self.instances

    def instance(self, robot_id: str) -> Hi6InstanceConfig:
        try:
            return self.instances[robot_id]
        except KeyError as exc:
            raise KeyError(
                f"hi6.yaml 에 instance id '{robot_id}' 가 없습니다. "
                f"사용 가능: {list(self.instances)}"
            ) from exc

    def robot(self, robot_id: str) -> Hi6InstanceConfig:
        """Compatibility alias for :meth:`instance`."""
        return self.instance(robot_id)

    def enabled_instances(self) -> List[Hi6InstanceConfig]:
        return [item for item in self.instances.values() if item.enabled]

    def enabled_robots(self) -> List[Hi6InstanceConfig]:
        """Compatibility alias for :meth:`enabled_instances`."""
        return self.enabled_instances()


def _parse_hi6(data: dict) -> tuple[
    Hi6ModelConfig,
    Hi6SimulationConfig,
    Dict[str, Hi6InstanceConfig],
]:
    unknown_root = set(data) - _HI6_ROOT_KEYS
    if unknown_root:
        raise ValueError(
            "hi6.yaml 에 알 수 없는 항목이 있습니다: "
            + ", ".join(sorted(unknown_root))
        )
    missing_root = _HI6_ROOT_KEYS - set(data)
    if missing_root:
        raise ValueError(
            "hi6.yaml 필수 항목이 없습니다: "
            + ", ".join(sorted(missing_root))
        )

    raw_model = _required_mapping(data.get("model"), "model")
    unknown_model = set(raw_model) - _HI6_MODEL_KEYS
    if unknown_model:
        raise ValueError(
            "hi6.yaml model 에 알 수 없는 항목이 있습니다: "
            + ", ".join(sorted(unknown_model))
        )
    model = _parse_hi6_model(raw_model, "model")

    raw_simulation = _required_mapping(data.get("simulation"), "simulation")
    unknown_simulation = set(raw_simulation) - _HI6_SIMULATION_KEYS
    if unknown_simulation:
        raise ValueError(
            "hi6.yaml simulation 에 알 수 없는 항목이 있습니다: "
            + ", ".join(sorted(unknown_simulation))
        )
    simulation = _parse_hi6_simulation(raw_simulation)

    raw_instances = _required_mapping(data.get("instances"), "instances")
    if not raw_instances:
        raise ValueError("hi6.yaml 의 instances 항목이 비어 있습니다")
    instances: Dict[str, Hi6InstanceConfig] = {}
    for robot_id, raw_value in raw_instances.items():
        path = f"instances.{robot_id}"
        values = _required_mapping(raw_value, path)
        unknown_instance = set(values) - _HI6_INSTANCE_KEYS
        if unknown_instance:
            raise ValueError(
                f"hi6.yaml {path} 에 model/policy 또는 알 수 없는 항목이 "
                "있습니다: " + ", ".join(sorted(unknown_instance))
            )
        instances[str(robot_id)] = _parse_hi6_instance(
            str(robot_id), values, model, path
        )
    return model, simulation, instances


def _parse_legacy_hi6(data: dict) -> tuple[
    Hi6ModelConfig,
    Hi6SimulationConfig,
    Dict[str, Hi6InstanceConfig],
]:
    """Read old defaults/robots files without weakening the new schema."""
    defaults = dict(data.get("defaults") or {})
    raw_robots = _required_mapping(data.get("robots"), "robots")
    if not raw_robots:
        raise ValueError("hi6.yaml 의 robots 항목이 비어 있습니다")

    base_model_values = {
        key: value for key, value in defaults.items()
        if key in _HI6_MODEL_KEYS
    }
    model = _parse_hi6_model(base_model_values, "defaults")
    simulation = _parse_hi6_simulation({})
    instances: Dict[str, Hi6InstanceConfig] = {}
    for robot_id, raw_value in raw_robots.items():
        path = f"robots.{robot_id}"
        overrides = _required_mapping(raw_value, path)
        merged = dict(defaults)
        merged.update(overrides)
        per_instance_model = _parse_hi6_model(
            {key: value for key, value in merged.items()
             if key in _HI6_MODEL_KEYS},
            path,
        )
        instance_values = {
            key: value for key, value in merged.items()
            if key in _HI6_INSTANCE_KEYS
        }
        instance = _parse_hi6_instance(
            str(robot_id), instance_values, per_instance_model, path
        )
        instances[str(robot_id)] = Hi6InstanceConfig(
            robot_id=instance.robot_id,
            enabled=instance.enabled,
            host=instance.host,
            visualization_base_xyz_m=instance.visualization_base_xyz_m,
            visualization_base_rpy_rad=instance.visualization_base_rpy_rad,
            model=instance.model,
            **{
                key: _strict_bool(merged.get(key, False), f"{path}.{key}")
                for key in _HI6_LEGACY_POLICY_KEYS
            },
        )
    return model, simulation, instances


def _parse_hi6_model(values: dict, path: str) -> Hi6ModelConfig:
    name = str(values.get("name", "hi6_6axis")).strip()
    if not name:
        raise ValueError(f"{path}.name 이 비어 있습니다")
    allowed = sorted({int(value) for value in values.get(
        "allowed_speed_percent", [25, 50, 75, 100]
    )})
    if not allowed or any(value < 1 or value > 100 for value in allowed):
        raise ValueError(f"{path}.allowed_speed_percent 는 1~100 이어야 합니다")
    api_versions = sorted({int(value) for value in values.get(
        "supported_api_versions", [5]
    )})
    if not api_versions or any(value < 1 for value in api_versions):
        raise ValueError(f"{path}.supported_api_versions 는 양수여야 합니다")
    axis_names = _six_unique_names(
        values.get("axis_names", [f"J{index}" for index in range(1, 7)]),
        f"{path}.axis_names",
    )
    joint_names = _six_unique_names(
        values.get(
            "joint_names", [f"joint_{index}" for index in range(1, 7)]
        ),
        f"{path}.joint_names",
    )
    config = Hi6ModelConfig(
        name=name,
        axis_names=axis_names,
        joint_names=joint_names,
        rest_port=int(values.get("rest_port", 8888)),
        stream_port=int(values.get("stream_port", 49000)),
        connect_timeout_s=float(values.get("connect_timeout_s", 1.0)),
        read_timeout_s=float(values.get("read_timeout_s", 1.0)),
        pose_hz=float(values.get("pose_hz", 20.0)),
        status_hz=float(values.get("status_hz", 5.0)),
        status_publish_hz=float(values.get("status_publish_hz", 20.0)),
        stale_timeout_ms=int(values.get("stale_timeout_ms", 500)),
        verify_timeout_s=float(values.get("verify_timeout_s", 3.0)),
        allowed_speed_percent=allowed,
        supported_api_versions=api_versions,
    )
    _validate_hi6_model(config, path)
    return config


def _parse_hi6_simulation(values: dict) -> Hi6SimulationConfig:
    host = str(values.get("host", "127.0.0.1")).strip()
    if not host:
        raise ValueError("simulation.host 가 비어 있습니다")
    try:
        simulation_address = ipaddress.ip_address(host)
    except ValueError as exc:
        raise ValueError(
            "simulation.host 는 IPv4 loopback 주소여야 합니다"
        ) from exc
    if not isinstance(simulation_address, ipaddress.IPv4Address) or not (
        simulation_address.is_loopback
    ):
        raise ValueError("simulation.host 는 IPv4 loopback 주소여야 합니다")
    config = Hi6SimulationConfig(
        host=host,
        port_base=int(values.get("port_base", 18888)),
        pose_hz=float(values.get("pose_hz", 30.0)),
        status_hz=float(values.get("status_hz", 5.0)),
        status_publish_hz=float(values.get("status_publish_hz", 30.0)),
    )
    if config.port_base < 1 or config.port_base > 65535:
        raise ValueError("simulation.port_base 는 1~65535 이어야 합니다")
    if (
        config.pose_hz <= 0
        or config.status_hz <= 0
        or config.status_publish_hz <= 0
    ):
        raise ValueError(
            "simulation pose_hz/status_hz/status_publish_hz 는 0보다 커야 합니다"
        )
    return config


def _parse_hi6_instance(
    robot_id: str,
    values: dict,
    model: Hi6ModelConfig,
    path: str,
) -> Hi6InstanceConfig:
    host = str(values.get("host", "")).strip()
    if not host:
        raise ValueError(f"hi6.yaml {path}.host 가 비어 있습니다")
    return Hi6InstanceConfig(
        robot_id=robot_id,
        enabled=_strict_bool(values.get("enabled", True), f"{path}.enabled"),
        host=host,
        visualization_base_xyz_m=_float_vector3(
            values.get("visualization_base_xyz_m", [0.0, 0.0, 0.0]),
            f"{path}.visualization_base_xyz_m",
        ),
        visualization_base_rpy_rad=_float_vector3(
            values.get("visualization_base_rpy_rad", [0.0, 0.0, 0.0]),
            f"{path}.visualization_base_rpy_rad",
        ),
        model=model,
    )


def _validate_hi6_model(config: Hi6ModelConfig, path: str) -> None:
    for name, port in (
        ("rest_port", config.rest_port),
        ("stream_port", config.stream_port),
    ):
        if port < 1 or port > 65535:
            raise ValueError(f"{path}.{name} 는 1~65535 이어야 합니다")
    if config.connect_timeout_s <= 0 or config.read_timeout_s <= 0:
        raise ValueError(f"{path} timeout 은 0보다 커야 합니다")
    if (
        config.pose_hz <= 0
        or config.status_hz <= 0
        or config.status_publish_hz <= 0
    ):
        raise ValueError(
            f"{path} pose_hz/status_hz/status_publish_hz 는 0보다 커야 합니다"
        )
    if config.stale_timeout_ms <= 0 or config.verify_timeout_s <= 0:
        raise ValueError(f"{path} stale/verify timeout 은 0보다 커야 합니다")


def _validate_unique_enabled_hi6_endpoints(
    instances: Dict[str, Hi6InstanceConfig],
) -> None:
    endpoints: Dict[tuple[str, int], str] = {}
    for instance in instances.values():
        if not instance.enabled:
            continue
        endpoint = (instance.host, instance.rest_port)
        existing = endpoints.get(endpoint)
        if existing is not None:
            raise ValueError(
                "duplicate enabled Hi6 endpoint "
                f"{instance.host}:{instance.rest_port}: "
                f"instances.{existing} and instances.{instance.robot_id}"
            )
        endpoints[endpoint] = instance.robot_id


def _required_mapping(value: Any, path: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"hi6.yaml {path} 는 mapping 이어야 합니다")
    return dict(value)


def _six_unique_names(value: Any, path: str) -> List[str]:
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{path} 는 고유한 6개 이름이어야 합니다")
    names = [str(item).strip() for item in value]
    if len(names) != 6 or any(not item for item in names) or len(set(names)) != 6:
        raise ValueError(f"{path} 는 고유한 6개 이름이어야 합니다")
    return names


def _strict_bool(value: Any, path: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "yes", "on", "1"):
            return True
        if normalized in ("false", "no", "off", "0"):
            return False
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    raise ValueError(f"{path} 는 true 또는 false 여야 합니다")


def _float_vector3(value: Any, path: str) -> List[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{path} 는 유한한 숫자 3개여야 합니다")
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path} 는 유한한 숫자 3개여야 합니다") from exc
    if any(not math.isfinite(item) for item in result):
        raise ValueError(f"{path} 는 유한한 숫자 3개여야 합니다")
    return result


# ---------------------------------------------------------- PLC gateway config
_PLC_ROOT_KEYS = {"schema_version", "profiles", "safety"}
_PLC_PROFILE_KEYS = {
    "commissioned",
    "commands_enabled",
    "start_enabled",
    "poll_hz",
    "status_publish_hz",
    "stale_timeout_ms",
    "verify_timeout_s",
    "allowed_speed_percent",
    "connection",
}
_PLC_CONNECTION_KEYS = {
    "host",
    "port",
    "source_address",
    "frame",
    "protocol",
    "network_no",
    "pc_no",
    "io_no",
    "station_no",
    "monitor_timer_250ms",
    "connect_timeout_s",
    "read_timeout_s",
    "reconnect_backoff_s",
}
_PLC_SAFETY_KEYS = {"stale_timeout_ms", "fail_safe_state"}
_ROBOTS_ROOT_KEYS = {"schema_version", "robots", "workers", "unity"}
_PLC_ROBOT_KEYS = {
    "id",
    "label",
    "enabled",
    "calibrated",
    "axis_names",
    "scale",
    "dir",
    "offset",
    "visual_dir",
    "visual_offset",
    "limits_deg",
    "plc_map",
}
_PLC_MAP_KEYS = {
    "map_status",
    "availability",
    "read",
    "status_word",
    "direct_controls",
    "legacy_sim_commands",
}
_PLC_AVAILABILITY_KEYS = {
    "command_map_verified",
    "actual_feedback_available",
    "ack_available",
}
_PLC_READ_KEYS = {"head", "words", "axes", "feedback"}
_PLC_FEEDBACK_KEYS = {
    "operation_state",
    "run",
    "speed_down_1",
    "speed_down_2",
    "speed_down_3",
}
_PLC_STATUS_WORD_KEYS = {"device", "bits"}
_PLC_STATUS_BITS = {
    "hold",
    "emergency_stop",
    "fault_reset",
    "device_home",
    "robot_home",
    "standby",
}
_PLC_COMMANDS_KEYS = {"motion", "speed"}
_PLC_MOTION_FIELDS = {"run", "hold", "stop"}
_PLC_SPEED_FIELDS = {"speed_down_1", "speed_down_2", "speed_down_3"}
_PLC_DIRECT_CONTROL_KEYS = {
    "speed",
    "control_word",
    "writable",
    "action_pulse_seconds",
}
_PLC_DIRECT_SPEED_KEYS = {"speed_25", "speed_50", "speed_75"}
_PLC_DIRECT_SPEED_ENTRY_KEYS = {"device", "active_value"}
_PLC_DIRECT_CONTROL_WORD_KEYS = {"device", "bits"}
_PLC_WRITABLE_PROFILES = {"field", "sim"}
_PLC_DIRECTION_PROFILES = {"field", "sim"}
_PLC_WRITABLE_CONTROLS = {
    "speed_25",
    "speed_50",
    "speed_75",
    "hold",
    "fault_reset",
    "device_home",
    "robot_home",
    "standby",
}
_PLC_EXPECTED_SPEED_CONTROLS = {
    "speed_25": ("D1016", 25, "speed_down_1"),
    "speed_50": ("D1018", 50, "speed_down_2"),
    "speed_75": ("D1020", 75, "speed_down_3"),
}
_PLC_EXPECTED_CONTROL_WORD = "D1100"
_PLC_EXPECTED_CONTROL_BITS = {
    "hold": 0,
    "emergency_stop": 1,
    "fault_reset": 2,
    "device_home": 3,
    "robot_home": 4,
    "standby": 5,
}
_PLC_ACTION_PULSE_MIN_SECONDS = 0.1
_PLC_ACTION_PULSE_MAX_SECONDS = 5.0
_WORD_DEVICE_CODES = {
    # Keep this list intentionally narrower than mc_client.DEVICE_CODES.  These
    # mappings are word-register contracts, not bit-device contracts.
    0xA8,  # D
    0xB4,  # W
    0xAF,  # R
    0xB0,  # ZR
}
_ROS_INSTANCE_ID = re.compile(r"^[a-z][a-z0-9_]*$")


def _profile_directions(value: Any, profile: str, path: str) -> List[int]:
    """Resolve six axis signs for a field/sim profile.

    A legacy flat list remains supported and applies to both profiles.  The
    repository config uses an explicit mapping so a simulator-only visual
    change cannot silently alter the commissioned field transform.
    """

    selected = value
    if isinstance(value, Mapping):
        profiles = _strict_mapping(value, path)
        _require_exact_keys(
            profiles,
            _PLC_DIRECTION_PROFILES,
            _PLC_DIRECTION_PROFILES,
            path,
        )
        if profile not in profiles:
            raise ValueError(f"{path}에 profile '{profile}'이 없습니다")
        selected = profiles[profile]
    directions = [
        _strict_int(item, path)
        for item in _strict_sequence(selected, path)
    ]
    if len(directions) != 6 or any(item not in (-1, 1) for item in directions):
        raise ValueError(f"{path}는 -1 또는 1인 값 6개여야 합니다")
    return directions


def _profile_offsets(value: Any, profile: str, path: str) -> List[float]:
    """Resolve six visual zero offsets for a field/sim profile.

    A flat list remains supported for legacy/single-transform files.
    """

    selected = value
    if isinstance(value, Mapping):
        profiles = _strict_mapping(value, path)
        _require_exact_keys(
            profiles,
            _PLC_DIRECTION_PROFILES,
            _PLC_DIRECTION_PROFILES,
            path,
        )
        if profile not in profiles:
            raise ValueError(f"{path}에 profile '{profile}'이 없습니다")
        selected = profiles[profile]
    return _six_finite_numbers(selected, path)


def _to_visual_degrees(
    controller_degrees: Sequence[float],
    directions: Sequence[int],
    offsets: Sequence[float],
    controller_limits_min: Sequence[float],
    controller_limits_max: Sequence[float],
    path: str,
) -> tuple[List[float], bool]:
    """Apply a coordinate transform and clamp against transformed limits."""

    if len(controller_degrees) != 6:
        raise ValueError(f"{path} controller 축은 정확히 6개여야 합니다")
    try:
        controller = [float(value) for value in controller_degrees]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path} controller 축은 유한한 숫자여야 합니다") from exc
    if any(not math.isfinite(value) for value in controller):
        raise ValueError(f"{path} controller 축은 유한한 숫자여야 합니다")

    out: List[float] = []
    clamped = False
    for index, value in enumerate(controller):
        direction = directions[index]
        offset = offsets[index]
        degrees = value * direction + offset

        endpoint_a = controller_limits_min[index] * direction + offset
        endpoint_b = controller_limits_max[index] * direction + offset
        low, high = min(endpoint_a, endpoint_b), max(endpoint_a, endpoint_b)
        if degrees < low:
            degrees, clamped = low, True
        elif degrees > high:
            degrees, clamped = high, True
        out.append(degrees)
    return out, clamped


@dataclass(frozen=True)
class PlcCommandBlock:
    """One contiguous PLC command block."""

    device: str
    words: int
    fields: Dict[str, int]


@dataclass(frozen=True)
class PlcWordControl:
    """One directly writable word device and its active scalar value."""

    device: str
    active_value: int


@dataclass(frozen=True)
class PlcDirectControls:
    """Verified No.9~17 command/register map for one selected profile.

    ``writable_controls`` is only a configuration allowlist.  It does not
    authorize field writes by itself; the gateway must additionally require
    its explicit runtime write gate.
    """

    speed: Dict[str, PlcWordControl]
    control_word_device: str
    control_bits: Dict[str, int]
    writable_controls: FrozenSet[str]
    action_pulse_seconds: float


@dataclass(frozen=True)
class PlcRegisterMap:
    """Readback and command map belonging to exactly one robot instance."""

    map_status: str
    read_head: str
    read_words: int
    axes: List[AxisMap]
    operation_state_offset: int
    run_feedback_offset: int
    speed_feedback_offsets: Dict[str, int]
    status_word: str
    status_bits: Dict[str, int]
    direct_controls: PlcDirectControls
    motion_command: PlcCommandBlock
    speed_command: PlcCommandBlock
    actual_feedback_available: bool
    ack_available: bool
    command_map_verified: bool

    @property
    def feedback_available(self) -> bool:
        """Compatibility alias; this means actual robot feedback only."""
        return self.actual_feedback_available

    @property
    def map_verified(self) -> bool:
        """Compatibility alias for the No.9~17 command/register map."""
        return self.command_map_verified


@dataclass(frozen=True)
class PlcRobotInstance:
    """One physical robot represented by an isolated PLC register map."""

    robot_id: str
    label: str
    enabled: bool
    calibrated: bool
    axis_names: List[str]
    scale: List[float]
    dir: List[int]
    offset: List[float]
    visual_dir: List[int]
    visual_offset: List[float]
    limits_min: List[float]
    limits_max: List[float]
    topics: Dict[str, str]
    registers: Optional[PlcRegisterMap]

    @property
    def id(self) -> str:
        """Compatibility alias used by the existing Unity adapter."""
        return self.robot_id

    def to_degrees(self, raw: Sequence[int]) -> tuple[List[float], bool]:
        if len(raw) != 6:
            raise ValueError(
                f"robot '{self.robot_id}' raw 축은 정확히 6개여야 합니다"
            )
        out: List[float] = []
        clamped = False
        for index, value in enumerate(raw):
            degrees = (
                int(value) * self.scale[index] * self.dir[index]
                + self.offset[index]
            )
            low = self.limits_min[index]
            high = self.limits_max[index]
            if degrees < low:
                degrees, clamped = low, True
            elif degrees > high:
                degrees, clamped = high, True
            out.append(degrees)
        return out, clamped

    def to_visual_degrees(
        self,
        controller_degrees: Sequence[float],
    ) -> tuple[List[float], bool]:
        """Controller axis coordinates -> CAD/URDF joint coordinates."""
        return _to_visual_degrees(
            controller_degrees,
            self.visual_dir,
            self.visual_offset,
            self.limits_min,
            self.limits_max,
            f"robot '{self.robot_id}'",
        )

    def _require_registers(self) -> PlcRegisterMap:
        if self.registers is None:
            raise ValueError(
                f"robot '{self.robot_id}' 의 PLC register map이 아직 없습니다"
            )
        return self.registers

    @property
    def read_head(self) -> str:
        return self._require_registers().read_head

    @property
    def read_words(self) -> int:
        return self._require_registers().read_words

    @property
    def axes(self) -> List[AxisMap]:
        return self._require_registers().axes

    @property
    def operation_state_offset(self) -> int:
        return self._require_registers().operation_state_offset

    @property
    def run_feedback_offset(self) -> int:
        return self._require_registers().run_feedback_offset

    @property
    def speed_feedback_offsets(self) -> Dict[str, int]:
        return self._require_registers().speed_feedback_offsets

    @property
    def status_word(self) -> str:
        return self._require_registers().status_word

    @property
    def status_bits(self) -> Dict[str, int]:
        return self._require_registers().status_bits

    @property
    def direct_controls(self) -> PlcDirectControls:
        return self._require_registers().direct_controls

    @property
    def motion_command(self) -> PlcCommandBlock:
        return self._require_registers().motion_command

    @property
    def speed_command(self) -> PlcCommandBlock:
        return self._require_registers().speed_command

    @property
    def feedback_available(self) -> bool:
        return self.actual_feedback_available

    @property
    def actual_feedback_available(self) -> bool:
        return bool(
            self.registers and self.registers.actual_feedback_available
        )

    @property
    def ack_available(self) -> bool:
        return bool(self.registers and self.registers.ack_available)

    @property
    def map_verified(self) -> bool:
        return self.command_map_verified

    @property
    def command_map_verified(self) -> bool:
        return bool(self.registers and self.registers.command_map_verified)


@dataclass(frozen=True)
class PlcBridgeConfig:
    """Strict, fail-closed PLC gateway configuration.

    One profile selects one Mitsubishi endpoint.  Every enabled robot owns a
    distinct register range on that endpoint; direct robot-controller network
    settings intentionally do not exist in this model.
    """

    config_dir: Path
    profile: str
    connection: McConfig
    commissioned: bool
    commands_enabled: bool
    start_enabled: bool
    poll_hz: float
    status_publish_hz: float
    stale_timeout_ms: int
    verify_timeout_s: float
    allowed_speed_percent: List[int]
    instances: Dict[str, PlcRobotInstance]
    safety: Dict[str, Any]
    workers: Dict[str, Any]
    unity: Dict[str, Any]
    network: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def load(
        cls,
        config_dir: Optional[str] = None,
        profile: str = "sim",
    ) -> "PlcBridgeConfig":
        directory = find_config_dir(config_dir, required_file="plc.yaml")
        plc = _load(directory / "plc.yaml")
        robots_doc = _load(directory / "robots.yaml")
        network = (
            _load(directory / "network.yaml")
            if (directory / "network.yaml").is_file()
            else {}
        )
        _require_exact_keys(
            plc,
            _PLC_ROOT_KEYS,
            _PLC_ROOT_KEYS,
            "plc.yaml",
        )
        if int(plc["schema_version"]) != 1:
            raise ValueError("plc.yaml schema_version은 1이어야 합니다")
        profiles = _strict_mapping(plc["profiles"], "plc.yaml profiles")
        if profile not in profiles:
            raise KeyError(
                f"plc.yaml 에 profile '{profile}' 이 없습니다. "
                f"사용 가능: {list(profiles)}"
            )
        raw_profile = _strict_mapping(
            profiles[profile], f"plc.yaml profiles.{profile}"
        )
        _require_exact_keys(
            raw_profile,
            _PLC_PROFILE_KEYS,
            _PLC_PROFILE_KEYS,
            f"plc.yaml profiles.{profile}",
        )
        connection_values = _strict_mapping(
            raw_profile["connection"],
            f"plc.yaml profiles.{profile}.connection",
        )
        _require_exact_keys(
            connection_values,
            _PLC_CONNECTION_KEYS,
            _PLC_CONNECTION_KEYS,
            f"plc.yaml profiles.{profile}.connection",
        )
        connection = McConfig.from_dict(connection_values, strict=True)
        connection.validate()

        commissioned = _strict_bool(
            raw_profile["commissioned"],
            f"profiles.{profile}.commissioned",
        )
        commands_enabled = _strict_bool(
            raw_profile["commands_enabled"],
            f"profiles.{profile}.commands_enabled",
        )
        start_enabled = _strict_bool(
            raw_profile["start_enabled"],
            f"profiles.{profile}.start_enabled",
        )
        if commands_enabled and not commissioned:
            raise ValueError(
                f"profile '{profile}': commissioned=false이면 "
                "commands_enabled=true일 수 없습니다"
            )
        if start_enabled and not commands_enabled:
            raise ValueError(
                f"profile '{profile}': commands_enabled=false이면 "
                "start_enabled=true일 수 없습니다"
            )
        if profile == "field" and 5000 <= connection.port <= 5009:
            raise ValueError(
                "field MC port 5000~5009는 제공된 QnU/L 매뉴얼의 "
                "예약 범위이므로 사용할 수 없습니다"
            )
        _validate_plc_endpoint(connection, profile)

        poll_hz = _positive_float(raw_profile["poll_hz"], "poll_hz")
        status_publish_hz = _positive_float(
            raw_profile["status_publish_hz"], "status_publish_hz"
        )
        stale_timeout_ms = _positive_int(
            raw_profile["stale_timeout_ms"], "stale_timeout_ms"
        )
        verify_timeout_s = _positive_float(
            raw_profile["verify_timeout_s"], "verify_timeout_s"
        )
        allowed_speed_percent = sorted(
            {
                _strict_int(value, "allowed_speed_percent")
                for value in _strict_sequence(
                    raw_profile["allowed_speed_percent"],
                    "allowed_speed_percent",
                )
            }
        )
        if allowed_speed_percent != [25, 50, 75, 100]:
            raise ValueError(
                "allowed_speed_percent는 현재 PLC 3단 맵에 맞춰 "
                "[25, 50, 75, 100]이어야 합니다"
            )

        safety = _strict_mapping(plc["safety"], "plc.yaml safety")
        _require_exact_keys(
            safety,
            _PLC_SAFETY_KEYS,
            _PLC_SAFETY_KEYS,
            "plc.yaml safety",
        )
        safety_stale_timeout_ms = _positive_int(
            safety["stale_timeout_ms"],
            "safety.stale_timeout_ms",
        )
        if safety_stale_timeout_ms != stale_timeout_ms:
            raise ValueError(
                "profile stale_timeout_ms와 safety.stale_timeout_ms가 "
                "일치해야 합니다"
            )
        if str(safety["fail_safe_state"]).strip().lower() != "unknown":
            raise ValueError(
                "Ethernet 단절 때 PLC 명령을 보장할 수 없으므로 "
                "safety.fail_safe_state는 unknown이어야 합니다"
            )

        instances, workers, unity = _parse_plc_robots(robots_doc, profile)
        _validate_enabled_plc_ranges(instances)
        return cls(
            config_dir=directory,
            profile=profile,
            connection=connection,
            commissioned=commissioned,
            commands_enabled=commands_enabled,
            start_enabled=start_enabled,
            poll_hz=poll_hz,
            status_publish_hz=status_publish_hz,
            stale_timeout_ms=stale_timeout_ms,
            verify_timeout_s=verify_timeout_s,
            allowed_speed_percent=allowed_speed_percent,
            instances=instances,
            safety=safety,
            workers=workers,
            unity=unity,
            network=network,
        )

    @property
    def robots(self) -> Dict[str, PlcRobotInstance]:
        """Compatibility alias for instance-oriented callers."""
        return self.instances

    def instance(self, robot_id: str) -> PlcRobotInstance:
        try:
            return self.instances[robot_id]
        except KeyError as exc:
            raise KeyError(
                f"robots.yaml 에 robot id '{robot_id}' 가 없습니다. "
                f"사용 가능: {list(self.instances)}"
            ) from exc

    def robot(self, robot_id: str) -> PlcRobotInstance:
        return self.instance(robot_id)

    def enabled_instances(self) -> List[PlcRobotInstance]:
        return [item for item in self.instances.values() if item.enabled]

    def enabled_robots(self) -> List[PlcRobotInstance]:
        return self.enabled_instances()


def _parse_plc_robots(
    document: dict,
    profile: str,
) -> tuple[Dict[str, PlcRobotInstance], Dict[str, Any], Dict[str, Any]]:
    _require_exact_keys(
        document,
        _ROBOTS_ROOT_KEYS,
        _ROBOTS_ROOT_KEYS,
        "robots.yaml",
    )
    if int(document["schema_version"]) != 1:
        raise ValueError("robots.yaml schema_version은 1이어야 합니다")
    raw_robots = _strict_sequence(document["robots"], "robots.yaml robots")
    if not raw_robots:
        raise ValueError("robots.yaml robots가 비어 있습니다")
    instances: Dict[str, PlcRobotInstance] = {}
    for index, raw_value in enumerate(raw_robots):
        path = f"robots.yaml robots[{index}]"
        values = _strict_mapping(raw_value, path)
        legacy_keys = _PLC_ROBOT_KEYS - {"visual_dir", "visual_offset"}
        _require_exact_keys(values, _PLC_ROBOT_KEYS, legacy_keys, path)
        visual_keys = {"visual_dir", "visual_offset"} & set(values)
        if visual_keys and visual_keys != {"visual_dir", "visual_offset"}:
            missing = {"visual_dir", "visual_offset"} - visual_keys
            raise ValueError(
                f"{path} 필수 항목 누락: {', '.join(sorted(missing))}"
            )
        # Before visual transforms were introduced, the only supported shape
        # was a flat controller direction list. Keep those files readable with
        # an identity visual transform; profile-aware configs must be explicit.
        if not visual_keys and isinstance(values["dir"], Mapping):
            raise ValueError(
                f"{path} 필수 항목 누락: visual_dir, visual_offset"
            )
        robot_id = str(values["id"]).strip()
        if not _ROS_INSTANCE_ID.fullmatch(robot_id):
            raise ValueError(
                f"{path}.id는 소문자로 시작하는 영문/숫자/_ ROS 이름이어야 합니다"
            )
        if robot_id in instances:
            raise ValueError(f"중복 robot id: {robot_id}")
        axis_names = _six_unique_names(values["axis_names"], f"{path}.axis_names")
        scale = _six_finite_numbers(values["scale"], f"{path}.scale")
        directions = _profile_directions(values["dir"], profile, f"{path}.dir")
        offsets = _six_finite_numbers(values["offset"], f"{path}.offset")
        visual_directions = _profile_directions(
            values.get("visual_dir", [1] * 6),
            profile,
            f"{path}.visual_dir",
        )
        visual_offsets = _profile_offsets(
            values.get("visual_offset", [0.0] * 6),
            profile,
            f"{path}.visual_offset",
        )
        limits = _strict_mapping(values["limits_deg"], f"{path}.limits_deg")
        _require_exact_keys(
            limits,
            {"min", "max"},
            {"min", "max"},
            f"{path}.limits_deg",
        )
        limits_min = _six_finite_numbers(limits["min"], f"{path}.limits_deg.min")
        limits_max = _six_finite_numbers(limits["max"], f"{path}.limits_deg.max")
        if any(low >= high for low, high in zip(limits_min, limits_max)):
            raise ValueError(f"{path}.limits_deg는 모든 축에서 min < max여야 합니다")

        enabled = _strict_bool(values["enabled"], f"{path}.enabled")
        registers = None
        if values["plc_map"] is not None:
            registers = _parse_plc_register_map(
                _strict_mapping(values["plc_map"], f"{path}.plc_map"),
                axis_names,
                profile,
                f"{path}.plc_map",
            )
        if enabled and registers is None:
            raise ValueError(f"enabled robot '{robot_id}' 에 plc_map이 없습니다")

        namespace = f"/robot/{robot_id}"
        topics = {
            "memory": namespace + "/memory",
            "pose": namespace + "/pose",
            "unity_pose_raw": namespace + "/cmd_degs_raw",
            "unity_pose": namespace + "/cmd_degs",
            "joint_states": namespace + "/joint_states",
            "status": namespace + "/status",
            "state": namespace + "/state",
            "command": namespace + "/command",
            "mode": namespace + "/mode",
        }
        instances[robot_id] = PlcRobotInstance(
            robot_id=robot_id,
            label=str(values["label"]).strip(),
            enabled=enabled,
            calibrated=_strict_bool(values["calibrated"], f"{path}.calibrated"),
            axis_names=axis_names,
            scale=scale,
            dir=directions,
            offset=offsets,
            visual_dir=visual_directions,
            visual_offset=visual_offsets,
            limits_min=limits_min,
            limits_max=limits_max,
            topics=topics,
            registers=registers,
        )
    return (
        instances,
        _strict_mapping(document["workers"], "robots.yaml workers"),
        _strict_mapping(document["unity"], "robots.yaml unity"),
    )


def _parse_plc_register_map(
    values: dict,
    axis_names: List[str],
    profile: str,
    path: str,
) -> PlcRegisterMap:
    _require_exact_keys(values, _PLC_MAP_KEYS, _PLC_MAP_KEYS, path)
    map_status = str(values["map_status"]).strip().lower()
    if map_status not in ("sample_unverified", "verified"):
        raise ValueError(
            f"{path}.map_status는 sample_unverified 또는 verified여야 합니다"
        )
    raw_availability = _strict_mapping(
        values["availability"], f"{path}.availability"
    )
    _require_exact_keys(
        raw_availability,
        {"field", "sim"},
        {"field", "sim"},
        f"{path}.availability",
    )
    for availability_profile, availability_value in raw_availability.items():
        availability_path = f"{path}.availability.{availability_profile}"
        availability_mapping = _strict_mapping(
            availability_value, availability_path
        )
        _require_exact_keys(
            availability_mapping,
            _PLC_AVAILABILITY_KEYS,
            _PLC_AVAILABILITY_KEYS,
            availability_path,
        )
        for key in _PLC_AVAILABILITY_KEYS:
            _strict_bool(
                availability_mapping[key], f"{availability_path}.{key}"
            )
    if profile not in raw_availability:
        raise ValueError(
            f"{path}.availability에 profile '{profile}' 항목이 없습니다"
        )
    availability = _strict_mapping(
        raw_availability[profile], f"{path}.availability.{profile}"
    )
    _require_exact_keys(
        availability,
        _PLC_AVAILABILITY_KEYS,
        _PLC_AVAILABILITY_KEYS,
        f"{path}.availability.{profile}",
    )
    command_map_verified = _strict_bool(
        availability["command_map_verified"],
        f"{path}.availability.{profile}.command_map_verified",
    )
    actual_feedback_available = _strict_bool(
        availability["actual_feedback_available"],
        f"{path}.availability.{profile}.actual_feedback_available",
    )
    ack_available = _strict_bool(
        availability["ack_available"],
        f"{path}.availability.{profile}.ack_available",
    )
    # The photographed No.9~17 command/register map and an independent robot
    # actual-feedback/ACK channel are deliberately separate capabilities.
    # Do not infer either one from the other here.

    read = _strict_mapping(values["read"], f"{path}.read")
    _require_exact_keys(read, _PLC_READ_KEYS, _PLC_READ_KEYS, f"{path}.read")
    read_head = _validate_word_device(read["head"], f"{path}.read.head")
    read_words = _word_count(read["words"], f"{path}.read.words")
    _validate_device_span(read_head, read_words, f"{path}.read")

    raw_axes = _strict_sequence(read["axes"], f"{path}.read.axes")
    if len(raw_axes) != 6:
        raise ValueError(f"{path}.read.axes는 정확히 6개여야 합니다")
    axes: List[AxisMap] = []
    occupied_offsets: Dict[int, str] = {}
    for index, raw_axis in enumerate(raw_axes):
        axis_path = f"{path}.read.axes[{index}]"
        axis = _parse_offset_value(raw_axis, axis_path, require_name=True)
        if axis.name != axis_names[index]:
            raise ValueError(
                f"{axis_path}.name '{axis.name}' 이 axis_names[{index}] "
                f"'{axis_names[index]}' 과 다릅니다"
            )
        _validate_offset_bounds(axis, read_words, axis_path)
        for offset in _offset_span(axis):
            if offset in occupied_offsets:
                raise ValueError(
                    f"{axis_path} offset {offset}가 {occupied_offsets[offset]}와 겹칩니다"
                )
            occupied_offsets[offset] = axis_path
        axes.append(axis)

    feedback = _strict_mapping(read["feedback"], f"{path}.read.feedback")
    _require_exact_keys(
        feedback,
        _PLC_FEEDBACK_KEYS,
        _PLC_FEEDBACK_KEYS,
        f"{path}.read.feedback",
    )
    feedback_maps: Dict[str, AxisMap] = {}
    for name in sorted(_PLC_FEEDBACK_KEYS):
        item_path = f"{path}.read.feedback.{name}"
        item = _parse_offset_value(feedback[name], item_path, name=name)
        _validate_offset_bounds(item, read_words, item_path)
        for offset in _offset_span(item):
            if offset in occupied_offsets:
                raise ValueError(
                    f"{item_path} offset {offset}가 {occupied_offsets[offset]}와 겹칩니다"
                )
            occupied_offsets[offset] = item_path
        feedback_maps[name] = item

    status_word = _strict_mapping(values["status_word"], f"{path}.status_word")
    _require_exact_keys(
        status_word,
        _PLC_STATUS_WORD_KEYS,
        _PLC_STATUS_WORD_KEYS,
        f"{path}.status_word",
    )
    status_device = _validate_word_device(
        status_word["device"], f"{path}.status_word.device"
    )
    _validate_device_span(status_device, 1, f"{path}.status_word")
    raw_bits = _strict_mapping(status_word["bits"], f"{path}.status_word.bits")
    _require_exact_keys(
        raw_bits,
        _PLC_STATUS_BITS,
        _PLC_STATUS_BITS,
        f"{path}.status_word.bits",
    )
    status_bits = {
        name: _strict_int(value, f"{path}.status_word.bits.{name}")
        for name, value in raw_bits.items()
    }
    if any(bit < 0 or bit > 15 for bit in status_bits.values()):
        raise ValueError(f"{path}.status_word.bits는 0~15여야 합니다")
    if len(set(status_bits.values())) != len(status_bits):
        raise ValueError(f"{path}.status_word.bits는 서로 달라야 합니다")

    direct_controls = _parse_plc_direct_controls(
        values["direct_controls"],
        profile=profile,
        read_head=read_head,
        speed_feedback_offsets={
            name: feedback_maps[name].offset
            for name in ("speed_down_1", "speed_down_2", "speed_down_3")
        },
        status_device=status_device,
        status_bits=status_bits,
        path=f"{path}.direct_controls",
    )

    commands = _strict_mapping(
        values["legacy_sim_commands"], f"{path}.legacy_sim_commands"
    )
    _require_exact_keys(
        commands,
        _PLC_COMMANDS_KEYS,
        _PLC_COMMANDS_KEYS,
        f"{path}.legacy_sim_commands",
    )
    motion = _parse_plc_command_block(
        commands["motion"],
        _PLC_MOTION_FIELDS,
        f"{path}.legacy_sim_commands.motion",
    )
    speed = _parse_plc_command_block(
        commands["speed"],
        _PLC_SPEED_FIELDS,
        f"{path}.legacy_sim_commands.speed",
    )

    result = PlcRegisterMap(
        map_status=map_status,
        read_head=read_head,
        read_words=read_words,
        axes=axes,
        operation_state_offset=feedback_maps["operation_state"].offset,
        run_feedback_offset=feedback_maps["run"].offset,
        speed_feedback_offsets={
            name: feedback_maps[name].offset
            for name in ("speed_down_1", "speed_down_2", "speed_down_3")
        },
        status_word=status_device,
        status_bits=status_bits,
        direct_controls=direct_controls,
        motion_command=motion,
        speed_command=speed,
        actual_feedback_available=actual_feedback_available,
        ack_available=ack_available,
        command_map_verified=command_map_verified,
    )
    _validate_one_plc_map_ranges(result, path)
    return result


def _parse_plc_direct_controls(
    value: Any,
    *,
    profile: str,
    read_head: str,
    speed_feedback_offsets: Dict[str, int],
    status_device: str,
    status_bits: Dict[str, int],
    path: str,
) -> PlcDirectControls:
    """Parse and cross-check the photographed No.9~17 control contract."""
    values = _strict_mapping(value, path)
    _require_exact_keys(
        values,
        _PLC_DIRECT_CONTROL_KEYS,
        _PLC_DIRECT_CONTROL_KEYS,
        path,
    )

    raw_speed = _strict_mapping(values["speed"], f"{path}.speed")
    _require_exact_keys(
        raw_speed,
        _PLC_DIRECT_SPEED_KEYS,
        _PLC_DIRECT_SPEED_KEYS,
        f"{path}.speed",
    )
    speed: Dict[str, PlcWordControl] = {}
    for name in sorted(_PLC_DIRECT_SPEED_KEYS):
        entry_path = f"{path}.speed.{name}"
        entry = _strict_mapping(raw_speed[name], entry_path)
        _require_exact_keys(
            entry,
            _PLC_DIRECT_SPEED_ENTRY_KEYS,
            _PLC_DIRECT_SPEED_ENTRY_KEYS,
            entry_path,
        )
        device = _validate_word_device(entry["device"], f"{entry_path}.device")
        active_value = _strict_int(
            entry["active_value"], f"{entry_path}.active_value"
        )
        speed[name] = PlcWordControl(
            device=device,
            active_value=active_value,
        )
    speed_devices = [item.device for item in speed.values()]
    if len(set(speed_devices)) != len(speed_devices):
        raise ValueError(f"{path}.speed device는 서로 달라야 합니다")
    for name, control in speed.items():
        entry_path = f"{path}.speed.{name}"
        expected_device, expected_value, feedback_name = (
            _PLC_EXPECTED_SPEED_CONTROLS[name]
        )
        if (
            control.device != expected_device
            or control.active_value != expected_value
        ):
            raise ValueError(
                f"{entry_path}는 사진 No.9~11 계약 "
                f"{{device: {expected_device}, active_value: {expected_value}}}"
                " 이어야 합니다"
            )

        read_code, read_address = parse_device(read_head)
        control_code, control_address = parse_device(control.device)
        mapped_address = read_address + speed_feedback_offsets[feedback_name]
        if (control_code, control_address) != (read_code, mapped_address):
            raise ValueError(
                f"{entry_path}.device {control.device}가 read.feedback."
                f"{feedback_name} 주소와 다릅니다"
            )

    raw_control_word = _strict_mapping(
        values["control_word"], f"{path}.control_word"
    )
    _require_exact_keys(
        raw_control_word,
        _PLC_DIRECT_CONTROL_WORD_KEYS,
        _PLC_DIRECT_CONTROL_WORD_KEYS,
        f"{path}.control_word",
    )
    control_word_device = _validate_word_device(
        raw_control_word["device"], f"{path}.control_word.device"
    )
    if control_word_device != _PLC_EXPECTED_CONTROL_WORD:
        raise ValueError(
            f"{path}.control_word.device는 사진 No.12~17 계약 "
            f"{_PLC_EXPECTED_CONTROL_WORD} 이어야 합니다"
        )
    if control_word_device != status_device:
        raise ValueError(
            f"{path}.control_word.device가 status_word.device와 다릅니다"
        )

    raw_control_bits = _strict_mapping(
        raw_control_word["bits"], f"{path}.control_word.bits"
    )
    _require_exact_keys(
        raw_control_bits,
        _PLC_STATUS_BITS,
        _PLC_STATUS_BITS,
        f"{path}.control_word.bits",
    )
    control_bits = {
        name: _strict_int(value, f"{path}.control_word.bits.{name}")
        for name, value in raw_control_bits.items()
    }
    if control_bits != _PLC_EXPECTED_CONTROL_BITS:
        raise ValueError(
            f"{path}.control_word.bits는 사진 No.12~17 계약 "
            f"{_PLC_EXPECTED_CONTROL_BITS} 이어야 합니다"
        )
    if control_bits != status_bits:
        raise ValueError(
            f"{path}.control_word.bits가 status_word.bits와 다릅니다"
        )

    raw_writable = _strict_mapping(values["writable"], f"{path}.writable")
    _require_exact_keys(
        raw_writable,
        _PLC_WRITABLE_PROFILES,
        _PLC_WRITABLE_PROFILES,
        f"{path}.writable",
    )
    parsed_writable: Dict[str, FrozenSet[str]] = {}
    for writable_profile in sorted(_PLC_WRITABLE_PROFILES):
        writable_path = f"{path}.writable.{writable_profile}"
        raw_names = _strict_sequence(
            raw_writable[writable_profile], writable_path
        )
        names = [str(item).strip() for item in raw_names]
        if any(not name for name in names):
            raise ValueError(f"{writable_path}에 빈 제어 이름이 있습니다")
        if len(names) != len(set(names)):
            raise ValueError(f"{writable_path} 제어 이름은 중복될 수 없습니다")
        unknown = set(names) - _PLC_WRITABLE_CONTROLS
        if unknown:
            raise ValueError(
                f"{writable_path} 알 수 없거나 쓰기 금지된 제어: "
                f"{', '.join(sorted(unknown))}"
            )
        # emergency_stop is intentionally not in _PLC_WRITABLE_CONTROLS.  Keep
        # this explicit check so a future allowlist expansion cannot silently
        # expose D1100.1 as a network write.
        if "emergency_stop" in names:
            raise ValueError(f"{writable_path}: emergency_stop은 쓰기 금지입니다")
        parsed_writable[writable_profile] = frozenset(names)
    if profile not in parsed_writable:
        raise ValueError(f"{path}.writable에 profile '{profile}'이 없습니다")

    action_pulse_seconds = _positive_float(
        values["action_pulse_seconds"], f"{path}.action_pulse_seconds"
    )
    if not (
        _PLC_ACTION_PULSE_MIN_SECONDS
        <= action_pulse_seconds
        <= _PLC_ACTION_PULSE_MAX_SECONDS
    ):
        raise ValueError(
            f"{path}.action_pulse_seconds는 "
            f"{_PLC_ACTION_PULSE_MIN_SECONDS}~"
            f"{_PLC_ACTION_PULSE_MAX_SECONDS}초여야 합니다"
        )

    return PlcDirectControls(
        speed=speed,
        control_word_device=control_word_device,
        control_bits=control_bits,
        writable_controls=parsed_writable[profile],
        action_pulse_seconds=action_pulse_seconds,
    )


def _parse_plc_command_block(
    value: Any,
    required_fields: set[str],
    path: str,
) -> PlcCommandBlock:
    values = _strict_mapping(value, path)
    _require_exact_keys(values, {"device", "words", "fields"}, {"device", "words", "fields"}, path)
    device = _validate_word_device(values["device"], f"{path}.device")
    words = _word_count(values["words"], f"{path}.words")
    _validate_device_span(device, words, path)
    raw_fields = _strict_mapping(values["fields"], f"{path}.fields")
    _require_exact_keys(raw_fields, required_fields, required_fields, f"{path}.fields")
    fields = {
        name: _strict_int(value, f"{path}.fields.{name}")
        for name, value in raw_fields.items()
    }
    if any(offset < 0 or offset >= words for offset in fields.values()):
        raise ValueError(f"{path}.fields offset은 0 이상 words 미만이어야 합니다")
    if len(set(fields.values())) != len(fields):
        raise ValueError(f"{path}.fields offset은 서로 달라야 합니다")
    return PlcCommandBlock(device=device, words=words, fields=fields)


def _parse_offset_value(
    value: Any,
    path: str,
    *,
    require_name: bool = False,
    name: str = "",
) -> AxisMap:
    values = _strict_mapping(value, path)
    required = {"offset", "type", "name"} if require_name else {"offset", "type"}
    _require_exact_keys(values, required, required, path)
    value_type = str(values["type"]).strip().lower()
    if value_type not in ("word", "dword"):
        raise ValueError(f"{path}.type은 word 또는 dword여야 합니다")
    return AxisMap(
        name=str(values["name"]).strip() if require_name else name,
        offset=_strict_int(values["offset"], f"{path}.offset"),
        type=value_type,
    )


def _validate_offset_bounds(value: AxisMap, words: int, path: str) -> None:
    last = value.offset + (1 if value.type == "dword" else 0)
    if value.offset < 0 or last >= words:
        raise ValueError(
            f"{path}.offset {value.offset} ({value.type})가 "
            f"read words {words} 범위를 벗어납니다"
        )


def _offset_span(value: AxisMap) -> range:
    return range(value.offset, value.offset + (2 if value.type == "dword" else 1))


def _validate_one_plc_map_ranges(registers: PlcRegisterMap, path: str) -> None:
    read_ranges = [
        _device_range(registers.read_head, registers.read_words, f"{path}.read"),
        _device_range(registers.status_word, 1, f"{path}.status_word"),
    ]
    write_ranges = [
        _device_range(
            registers.motion_command.device,
            registers.motion_command.words,
            f"{path}.commands.motion",
        ),
        _device_range(
            registers.speed_command.device,
            registers.speed_command.words,
            f"{path}.commands.speed",
        ),
    ]
    for write in write_ranges:
        for other in read_ranges + [item for item in write_ranges if item is not write]:
            if _ranges_overlap(write, other):
                raise ValueError(
                    f"{path}: PLC read/write 범위가 겹칩니다: "
                    f"{write[3]} / {other[3]}"
                )


def _validate_enabled_plc_ranges(
    instances: Dict[str, PlcRobotInstance],
) -> None:
    occupied: List[Tuple[int, int, int, str, str]] = []
    for instance in instances.values():
        if not instance.enabled:
            continue
        registers = instance._require_registers()
        ranges = [
            _device_range(registers.read_head, registers.read_words, "read"),
            _device_range(registers.status_word, 1, "status_word"),
            _device_range(
                registers.motion_command.device,
                registers.motion_command.words,
                "motion_command",
            ),
            _device_range(
                registers.speed_command.device,
                registers.speed_command.words,
                "speed_command",
            ),
        ]
        for current in ranges:
            for previous in occupied:
                if _ranges_overlap(current, previous):
                    raise ValueError(
                        "enabled robot PLC 범위가 겹칩니다: "
                        f"{instance.robot_id}.{current[3]} / "
                        f"{previous[4]}.{previous[3]}"
                    )
            occupied.append((*current, instance.robot_id))


def _device_range(device: str, words: int, label: str) -> Tuple[int, int, int, str]:
    code, address = parse_device(device)
    return code, address, address + words - 1, label


def _ranges_overlap(
    left: Sequence[Any],
    right: Sequence[Any],
) -> bool:
    return (
        int(left[0]) == int(right[0])
        and int(left[1]) <= int(right[2])
        and int(right[1]) <= int(left[2])
    )


def _validate_word_device(value: Any, path: str) -> str:
    device = str(value).strip().upper()
    try:
        code, _ = parse_device(device)
    except ValueError as exc:
        raise ValueError(f"{path}: {exc}") from exc
    if code not in _WORD_DEVICE_CODES:
        raise ValueError(f"{path}는 D/W/R/ZR word device여야 합니다")
    return device


def _validate_device_span(device: str, words: int, path: str) -> None:
    _, address = parse_device(device)
    if address + words - 1 > 0xFFFFFF:
        raise ValueError(f"{path} device 범위가 MC 24-bit 주소를 넘습니다")


def _word_count(value: Any, path: str) -> int:
    count = _positive_int(value, path)
    if count > 960:
        raise ValueError(f"{path}는 MC batch word 한도 960 이하여야 합니다")
    return count


def _validate_plc_endpoint(connection: McConfig, profile: str) -> None:
    try:
        host = ipaddress.ip_address(connection.host)
    except ValueError as exc:
        raise ValueError(f"profile '{profile}' PLC host는 IPv4 주소여야 합니다") from exc
    if not isinstance(host, ipaddress.IPv4Address):
        raise ValueError(f"profile '{profile}' PLC host는 IPv4 주소여야 합니다")
    if profile == "sim":
        if not host.is_loopback:
            raise ValueError("sim PLC host는 loopback IPv4 주소여야 합니다")
    elif host.is_unspecified or host.is_loopback or host.is_multicast:
        raise ValueError("field PLC host는 유효한 현장 IPv4 주소여야 합니다")
    if connection.source_address:
        try:
            source = ipaddress.ip_address(connection.source_address)
        except ValueError as exc:
            raise ValueError("connection.source_address는 IPv4 주소여야 합니다") from exc
        if not isinstance(source, ipaddress.IPv4Address) or source.is_multicast:
            raise ValueError("connection.source_address는 IPv4 주소여야 합니다")


def _strict_mapping(value: Any, path: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{path}는 mapping이어야 합니다")
    return dict(value)


def _strict_sequence(value: Any, path: str) -> list:
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{path}는 list여야 합니다")
    return list(value)


def _require_exact_keys(
    values: Any,
    allowed: set[str],
    required: set[str],
    path: str,
) -> None:
    mapping = _strict_mapping(values, path)
    missing = required - set(mapping)
    unknown = set(mapping) - allowed
    if missing:
        raise ValueError(f"{path} 필수 항목 누락: {', '.join(sorted(missing))}")
    if unknown:
        raise ValueError(f"{path} 알 수 없는 항목: {', '.join(sorted(unknown))}")


def _strict_int(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{path}는 정수여야 합니다")
    return value


def _positive_int(value: Any, path: str) -> int:
    result = _strict_int(value, path)
    if result <= 0:
        raise ValueError(f"{path}는 0보다 커야 합니다")
    return result


def _positive_float(value: Any, path: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{path}는 유한한 양수여야 합니다")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path}는 유한한 양수여야 합니다") from exc
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{path}는 유한한 양수여야 합니다")
    return result


def _six_finite_numbers(value: Any, path: str) -> List[float]:
    values = _strict_sequence(value, path)
    if len(values) != 6:
        raise ValueError(f"{path}는 유한한 숫자 6개여야 합니다")
    try:
        result = [float(item) for item in values]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path}는 유한한 숫자 6개여야 합니다") from exc
    if any(not math.isfinite(item) for item in result):
        raise ValueError(f"{path}는 유한한 숫자 6개여야 합니다")
    return result


# ---------------------------------------------------------------- 전체 설정
@dataclass
class BridgeConfig:
    config_dir: Path
    profile: str
    connection: dict
    read_head: str
    read_words: int
    poll_hz: float
    status_offset: int
    axes: List[AxisMap]
    status_bit_word: str
    status_bits: Dict[str, int]
    write_blocks: dict
    rewrite_hz: float
    safety: dict
    robots: List[RobotDef]
    workers: dict
    unity: dict
    network: dict = field(default_factory=dict)

    @classmethod
    def load(cls, config_dir: Optional[str] = None, profile: str = "sim") -> "BridgeConfig":
        d = find_config_dir(config_dir)
        plc = _load(d / "plc.yaml")
        rob = _load(d / "robots.yaml")
        net = _load(d / "network.yaml") if (d / "network.yaml").is_file() else {}

        profiles = plc.get("profiles", {})
        if profile not in profiles:
            raise KeyError(f"plc.yaml 에 profile '{profile}' 이 없습니다. 사용 가능 : {list(profiles)}")

        rb = plc.get("read_block", {})
        sb = plc.get("status_bits", {})
        wb = plc.get("write_block", {})

        return cls(
            config_dir=d,
            profile=profile,
            connection=profiles[profile]["connection"],
            read_head=rb.get("head", "D1000"),
            read_words=int(rb.get("words", 14)),
            poll_hz=float(rb.get("poll_hz", 20)),
            status_offset=int((plc.get("status", {}).get("operation_state", {})).get("offset", 0)),
            axes=[AxisMap(a["name"], int(a["offset"]), a.get("type", "dword"))
                  for a in plc.get("axes", [])],
            status_bit_word=sb.get("word", "D1100"),
            status_bits=dict(sb.get("bits", {})),
            write_blocks={k: v for k, v in wb.items() if isinstance(v, dict)},
            rewrite_hz=float(wb.get("rewrite_hz", 19)),
            safety=dict(plc.get("safety", {})),
            robots=[
                RobotDef.from_dict(robot, profile=profile)
                for robot in rob.get("robots", [])
            ],
            workers=dict(rob.get("workers", {})),
            unity=dict(rob.get("unity", {})),
            network=net,
        )

    def robot(self, robot_id: str) -> RobotDef:
        for r in self.robots:
            if r.id == robot_id:
                return r
        raise KeyError(f"robots.yaml 에 robot id '{robot_id}' 가 없습니다.")

    def enabled_robots(self) -> List[RobotDef]:
        return [r for r in self.robots if r.enabled]
