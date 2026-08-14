"""config/*.yaml 을 읽어 노드들이 공유하는 설정 객체로 만든다.

설정 파일 위치는 다음 순서로 찾는다.
  1) ROS 파라미터 `config_dir`
  2) 환경변수 `ROBOT_DT_CONFIG`
  3) 패키지 share/config
  4) 저장소 루트의 config/   (소스 트리에서 바로 실행할 때)
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


def find_config_dir(explicit: Optional[str] = None) -> Path:
    candidates: List[Path] = []
    if explicit:
        candidates.append(Path(explicit))
    if os.environ.get("ROBOT_DT_CONFIG"):
        candidates.append(Path(os.environ["ROBOT_DT_CONFIG"]))
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory("robot_bridge")) / "config")
    except Exception:                      # ament 미설치 환경(단독 테스트)
        pass
    here = Path(__file__).resolve()
    for up in here.parents:
        candidates.append(up / "config")
    for c in candidates:
        if (c / "plc.yaml").is_file():
            return c
    raise FileNotFoundError(
        "plc.yaml 을 찾을 수 없습니다. ROBOT_DT_CONFIG 환경변수로 config 디렉터리를 지정하세요."
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
    limits_min: List[float]
    limits_max: List[float]
    topics: Dict[str, str]

    @classmethod
    def from_dict(cls, d: dict) -> "RobotDef":
        lim = d.get("limits_deg") or {}
        n = len(d.get("axis_names", [])) or 6
        return cls(
            id=d["id"],
            label=d.get("label", d["id"]),
            enabled=bool(d.get("enabled", True)),
            axis_names=list(d.get("axis_names", [f"J{i+1}" for i in range(n)])),
            calibrated=bool(d.get("calibrated", False)),
            scale=[float(x) for x in d.get("scale", [0.001] * n)],
            dir=[int(x) for x in d.get("dir", [1] * n)],
            offset=[float(x) for x in d.get("offset", [0.0] * n)],
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
            robots=[RobotDef.from_dict(r) for r in rob.get("robots", [])],
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
