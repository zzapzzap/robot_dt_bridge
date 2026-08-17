"""여러 발행자(XDI · XAG · Unity · 운전원)의 명령을 하나로 중재한다.

입력 경로가 두 가지다.

  · **토픽** `RobotCommand` — 주기 발행이 전제. `command_timeout_ms`(기본 300 ms)
    안에 갱신되지 않으면 소멸한다. XDI · XAG 처럼 계속 판단해서 쏘는 쪽이 쓴다.
  · **서비스** `SetSafetyMode` — 한 번 호출로 **고정(latch)** 된다. 운전원이나
    상위 시스템이 "지금부터 REDUCED_SPEED_50" 하고 걸어 두는 용도.
    `hold_seconds` 를 주면 그 시간 뒤 자동 해제된다.

규칙
  1. 우선순위가 높은 쪽이 이긴다 (plc.yaml safety.priority).
  2. 같은 우선순위면 나중에 온 것이 이긴다. 동순위면 고정 > 토픽.
  3. 토픽 지령은 timeout 으로 소멸, 서비스 지령(latch)은 명시적 해제 전까지 유지.
  4. 정지 계열과 속도제한 계열은 각각 배타적이며 서로 독립이다.
     단 정지 중에는 속도제한 지령을 겹쳐 쓰지 않는다.
  5. PLC 링크가 끊기면 fail_safe 를 강제한다 (모든 지령보다 우선).

CDR 후속조치 AI-102 「XDI ↔ XAG 중재 규칙」의 구현부다.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# ---------------------------------------------------------------- 모드 정의
#
# 이름에 **결과 속도**를 그대로 박아 넣는다.
#     REDUCED_SPEED_50  =  전속의 50 % 로 달린다.   해석의 여지가 없다.
#
# 「감속 2 (50 %)」 같은 표기는 50 % 를 줄인다는 뜻인지 50 % 로 달린다는 뜻인지
# 갈려서, 실제로 이 저장소 안에서 정반대 값이 돌아다녔다. 그래서 폐기했다.
# PLC 필드명(speed_down_N)은 에이시스 사양서 계약이므로 그대로 두고 매핑만 한다.
#
# 정지 용어는 ISO 10218 / ISO-TS 15066 을 따른다.
#     PROTECTIVE_STOP   전원을 유지한 채 멈춤 (복귀 가능)   ← PLC hold
#     EMERGENCY_STOP    비상정지                          ← PLC stop
MODE_UNKNOWN = 0
MODE_NORMAL = 1
MODE_REDUCED_75 = 2
MODE_REDUCED_50 = 3
MODE_REDUCED_25 = 4
MODE_PROTECTIVE_STOP = 5
MODE_EMERGENCY_STOP = 6

# 정규 식별자 — 로그 · 비교 · 외부 연동용
MODE_NAMES: Dict[int, str] = {
    MODE_UNKNOWN: "UNKNOWN",
    MODE_NORMAL: "NORMAL",
    MODE_REDUCED_75: "REDUCED_SPEED_75",
    MODE_REDUCED_50: "REDUCED_SPEED_50",
    MODE_REDUCED_25: "REDUCED_SPEED_25",
    MODE_PROTECTIVE_STOP: "PROTECTIVE_STOP",
    MODE_EMERGENCY_STOP: "EMERGENCY_STOP",
}

# 사람이 읽는 라벨 — 화면 · 회의 자료용
MODE_LABELS: Dict[int, str] = {
    MODE_UNKNOWN: "알 수 없음",
    MODE_NORMAL: "정상 운전 · 전속",
    MODE_REDUCED_75: "속도제한 75 %",
    MODE_REDUCED_50: "속도제한 50 %",
    MODE_REDUCED_25: "속도제한 25 %",
    MODE_PROTECTIVE_STOP: "보호정지 (전원 유지)",
    MODE_EMERGENCY_STOP: "비상정지",
}

# 이름 안의 숫자와 값이 항상 같다 — 어긋날 수 없는 구조
MODE_SPEED: Dict[int, float] = {
    MODE_UNKNOWN: 0.0,
    MODE_NORMAL: 1.00,
    MODE_REDUCED_75: 0.75,
    MODE_REDUCED_50: 0.50,
    MODE_REDUCED_25: 0.25,
    MODE_PROTECTIVE_STOP: 0.0,
    MODE_EMERGENCY_STOP: 0.0,
}

# 모드 ↔ RobotCommand / PLC 필드
#   PLC 의 speed_down_N 은 "N단 감속"(감속률) 표기라 숫자 방향이 반대다.
#   뒤집는 지점은 이 표 하나뿐이다.
#       speed_down_1 (25 % 감속) → REDUCED_SPEED_75
#       speed_down_3 (75 % 감속) → REDUCED_SPEED_25
MODE_FIELD: Dict[int, str] = {
    MODE_NORMAL: "run",
    MODE_REDUCED_75: "speed_down_1",
    MODE_REDUCED_50: "speed_down_2",
    MODE_REDUCED_25: "speed_down_3",
    MODE_PROTECTIVE_STOP: "hold",
    MODE_EMERGENCY_STOP: "stop",
}
FIELD_MODE: Dict[str, int] = {v: k for k, v in MODE_FIELD.items()}


def mode_name(mode: int) -> str:
    """정규 식별자 (NORMAL · REDUCED_SPEED_50 · EMERGENCY_STOP …)."""
    return MODE_NAMES.get(mode, MODE_NAMES[MODE_UNKNOWN])


def mode_label(mode: int) -> str:
    """사람이 읽는 한글 라벨."""
    return MODE_LABELS.get(mode, MODE_LABELS[MODE_UNKNOWN])


def speed_ratio(mode: int) -> float:
    """전속 대비 속도 배율 (1.0 / 0.75 / 0.5 / 0.25 / 0.0)."""
    return MODE_SPEED.get(mode, 0.0)


def parse_mode(text: str) -> Optional[int]:
    """CLI 입력을 모드 값으로 해석. 영문 · 한글 · 구 표기 모두 받는다."""
    t = str(text).strip().lower()
    if t.isdigit() and int(t) in MODE_NAMES:
        return int(t)                       # 모드 번호 직접 지정
    key = t.replace(" ", "").replace("_", "").replace("-", "").replace("%", "")
    table = {
        # 정상
        "normal": MODE_NORMAL, "full": MODE_NORMAL, "fullspeed": MODE_NORMAL,
        "run": MODE_NORMAL, "정상": MODE_NORMAL, "전속": MODE_NORMAL,
        # 속도제한 — 이름의 숫자가 곧 결과 속도
        "reducedspeed75": MODE_REDUCED_75, "reduced75": MODE_REDUCED_75,
        "rs75": MODE_REDUCED_75, "속도75": MODE_REDUCED_75,
        "reducedspeed50": MODE_REDUCED_50, "reduced50": MODE_REDUCED_50,
        "rs50": MODE_REDUCED_50, "속도50": MODE_REDUCED_50,
        "reducedspeed25": MODE_REDUCED_25, "reduced25": MODE_REDUCED_25,
        "rs25": MODE_REDUCED_25, "속도25": MODE_REDUCED_25,
        # 정지
        "protectivestop": MODE_PROTECTIVE_STOP, "protective": MODE_PROTECTIVE_STOP,
        "pstop": MODE_PROTECTIVE_STOP, "hold": MODE_PROTECTIVE_STOP,
        "보호정지": MODE_PROTECTIVE_STOP, "일시정지": MODE_PROTECTIVE_STOP,
        "emergencystop": MODE_EMERGENCY_STOP, "emergency": MODE_EMERGENCY_STOP,
        "estop": MODE_EMERGENCY_STOP, "stop": MODE_EMERGENCY_STOP,
        "비상정지": MODE_EMERGENCY_STOP, "정지": MODE_EMERGENCY_STOP,
        # 구 표기(감속률 기준) — 숫자가 뒤집히므로 호환용으로만 남긴다
        "감속1": MODE_REDUCED_75, "slow1": MODE_REDUCED_75,
        "감속2": MODE_REDUCED_50, "slow2": MODE_REDUCED_50,
        "감속3": MODE_REDUCED_25, "slow3": MODE_REDUCED_25,
    }
    return table.get(key)


@dataclass
class Applied:
    run: bool = True
    hold: bool = False
    stop: bool = False
    speed_down_1: bool = False
    speed_down_2: bool = False
    speed_down_3: bool = False
    source: str = "default"
    reason: str = ""
    priority: int = 0
    latched: bool = False
    latch_remaining_s: float = 0.0

    def key(self) -> tuple:
        return (self.run, self.hold, self.stop,
                self.speed_down_1, self.speed_down_2, self.speed_down_3)

    @property
    def mode(self) -> int:
        """적용 결과를 하나의 모드 값으로 환산 (엄격한 쪽 우선)."""
        if self.stop:
            return MODE_EMERGENCY_STOP
        if self.hold:
            return MODE_PROTECTIVE_STOP
        if self.speed_down_3:
            return MODE_REDUCED_25
        if self.speed_down_2:
            return MODE_REDUCED_50
        if self.speed_down_1:
            return MODE_REDUCED_75
        if self.run:
            return MODE_NORMAL
        return MODE_UNKNOWN

    @property
    def mode_name(self) -> str:
        return mode_name(self.mode)

    @property
    def mode_label(self) -> str:
        return mode_label(self.mode)

    @property
    def speed_ratio(self) -> float:
        return speed_ratio(self.mode)


@dataclass
class _Entry:
    priority: int
    stamp: float
    source: str
    reason: str
    field: str                     # "stop" | "hold" | "run" | "speed_down_N"
    ttl: Optional[float] = None    # None = 토픽(공통 timeout), 값 = latch 유지시간
    latched: bool = False

    def remaining(self, now: float, topic_timeout: float) -> float:
        if self.latched:
            if self.ttl is None or self.ttl <= 0.0:
                return float("inf")
            return max(0.0, self.ttl - (now - self.stamp))
        return max(0.0, topic_timeout - (now - self.stamp))


class SafetyGate:
    URGENT = ("stop", "hold", "run")
    SLOW = ("speed_down_3", "speed_down_2", "speed_down_1")

    def __init__(self, safety_cfg: dict, logger=None):
        self.prio: Dict[str, int] = dict(safety_cfg.get("priority", {}))
        self.cmd_timeout = float(safety_cfg.get("command_timeout_ms", 300)) / 1000.0
        self.fail_safe = str(safety_cfg.get("fail_safe", "hold"))
        self._log = logger
        self._urgent: Optional[_Entry] = None
        self._slow: Optional[_Entry] = None
        self._applied = Applied()
        self._mode_since = time.time()

    # ------------------------------------------------------------ 토픽 입력
    def submit(self, msg) -> None:
        """RobotCommand 토픽 수신 — timeout 으로 소멸하는 지령."""
        now = time.time()
        src = msg.source or "unknown"
        base = int(msg.priority) if msg.priority else 0

        urgent = self._pick(msg, self.URGENT)
        if urgent:
            e = _Entry(max(base, self.prio.get(urgent, 0)), now, src, msg.reason, urgent)
            if self._beats(e, self._urgent):
                self._urgent = e

        slow = self._pick(msg, self.SLOW)
        if slow:
            e = _Entry(max(base, self.prio.get(slow, 0)), now, src, msg.reason, slow)
            if self._beats(e, self._slow):
                self._slow = e

    # ---------------------------------------------------------- 서비스 입력
    def set_mode(self, mode: int, source: str = "service", reason: str = "",
                 hold_seconds: float = 0.0,
                 clear_latched: bool = False) -> Tuple[bool, str]:
        """SetSafetyMode 서비스 — 고정(latch) 지령. (수락여부, 메시지) 반환."""
        if mode not in MODE_FIELD:
            return False, f"알 수 없는 모드 값 {mode}"

        if clear_latched:
            self.clear_latch()

        field = MODE_FIELD[mode]
        now = time.time()
        e = _Entry(self.prio.get(field, 0), now, source or "service", reason,
                   field, ttl=float(hold_seconds), latched=True)

        slot = "_urgent" if field in self.URGENT else "_slow"
        cur: Optional[_Entry] = getattr(self, slot)
        if not self._beats(e, cur):
            keep = mode_name(FIELD_MODE.get(cur.field, MODE_UNKNOWN)) if cur else "?"
            return False, (f"우선순위가 낮아 적용되지 않았습니다 "
                           f"(현재 {keep} · {cur.source if cur else '-'} 우선)")
        setattr(self, slot, e)

        # 정지를 걸면 남아 있던 속도제한 고정은 의미가 없으므로 정리한다.
        # 반대로 NORMAL 을 명시적으로 요청하면 속도제한 고정까지 함께 푼다
        # (단 정지 고정은 clear_latched 로만 해제된다 — 안전상 의도).
        if field in ("stop", "hold", "run") and self._slow is not None and self._slow.latched:
            self._slow = None
        return True, "적용됨"

    def clear_latch(self) -> None:
        """서비스로 걸어 둔 고정 지령을 모두 해제한다."""
        if self._urgent is not None and self._urgent.latched:
            self._urgent = None
        if self._slow is not None and self._slow.latched:
            self._slow = None

    # ------------------------------------------------------------ 내부 판정
    @staticmethod
    def _pick(msg, fields) -> Optional[str]:
        for f in fields:                      # 엄격한 쪽부터 검사
            if getattr(msg, f, False):
                return f
        return None

    def _beats(self, new: _Entry, cur: Optional[_Entry]) -> bool:
        if cur is None or self._expired(cur):
            return True
        if new.priority > cur.priority:
            return True
        if new.priority < cur.priority:
            return False
        # 동순위 — 고정 지령이 토픽 지령보다 우선, 그 외에는 최신이 이긴다
        if new.latched and not cur.latched:
            return True
        if cur.latched and not new.latched:
            return False
        return True

    def _expired(self, e: _Entry) -> bool:
        return e.remaining(time.time(), self.cmd_timeout) <= 0.0

    # ------------------------------------------------------------------ 출력
    def resolve(self, link_stale: bool = False) -> Applied:
        now = time.time()
        if self._urgent and self._expired(self._urgent):
            self._urgent = None
        if self._slow and self._expired(self._slow):
            self._slow = None

        a = Applied()
        if link_stale and self.fail_safe in ("hold", "stop"):
            setattr(a, self.fail_safe, True)
            a.run = False
            a.source, a.reason = "watchdog", "PLC 링크 두절 — fail-safe 적용"
            a.priority = 255
            self._finish(a, now)
            return a

        if self._urgent:
            a.run = a.hold = a.stop = False
            setattr(a, self._urgent.field, True)
        if not (a.stop or a.hold) and self._slow:
            setattr(a, self._slow.field, True)

        # 귀속은 '실제로 적용된 모드' 를 만든 쪽이 가져간다.
        # (NORMAL 지령 + 속도제한 고정이 겹치면 의미 있는 제약은 속도제한 쪽이다)
        owner = None
        field = MODE_FIELD.get(a.mode)
        for e in (self._urgent, self._slow):
            if e is not None and e.field == field:
                owner = e
                break
        if owner is not None:
            a.source, a.reason = owner.source, owner.reason
            a.priority = owner.priority
            a.latched = owner.latched
            a.latch_remaining_s = _finite(owner.remaining(now, self.cmd_timeout))

        self._finish(a, now)
        return a

    def _finish(self, a: Applied, now: float) -> None:
        if a.key() != self._applied.key():
            self._mode_since = now
            if self._log is not None:
                self._log.info(
                    f"모드 변경 → {a.mode_name} ({a.mode_label} · "
                    f"속도 {a.speed_ratio * 100:.0f} %) "
                    f"[{a.source}]{' 고정' if a.latched else ''} {a.reason}"
                )
        self._applied = a

    def last_applied(self) -> Applied:
        return self._applied

    def mode(self) -> int:
        return self._applied.mode

    def mode_since(self) -> float:
        return self._mode_since


def _finite(v: float) -> float:
    return 0.0 if v == float("inf") else float(v)
