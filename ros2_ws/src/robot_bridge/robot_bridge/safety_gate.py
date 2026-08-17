"""여러 발행자(XDI · XAG · Unity · 운전원)의 명령을 하나로 중재한다.

입력 경로가 두 가지다.

  · **토픽** `RobotCommand` — 주기 발행이 전제. `command_timeout_ms`(기본 300 ms)
    안에 갱신되지 않으면 소멸한다. XDI · XAG 처럼 계속 판단해서 쏘는 쪽이 쓴다.
  · **서비스** `SetSafetyMode` — 한 번 호출로 **고정(latch)** 된다. 운전원이나
    상위 시스템이 "지금부터 감속 2" 하고 걸어 두는 용도. `hold_seconds` 를 주면
    그 시간 뒤 자동 해제된다.

규칙
  1. 우선순위가 높은 쪽이 이긴다 (plc.yaml safety.priority).
  2. 같은 우선순위면 나중에 온 것이 이긴다.
  3. 토픽 지령은 timeout 으로 소멸, 서비스 지령(latch)은 명시적 해제 전까지 유지.
  4. 긴급(run/hold/stop)과 감속(1/2/3)은 각각 배타적이며 서로 독립이다.
     단 정지·일시정지 중에는 감속 지령을 겹쳐 쓰지 않는다.
  5. PLC 링크가 끊기면 fail_safe 를 강제한다 (모든 지령보다 우선).

CDR 후속조치 AI-102 「XDI ↔ XAG 중재 규칙」의 구현부다.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# ---------------------------------------------------------------- 모드 정의
MODE_UNKNOWN = 0
MODE_RUN = 1
MODE_SLOW_1 = 2
MODE_SLOW_2 = 3
MODE_SLOW_3 = 4
MODE_HOLD = 5
MODE_STOP = 6

MODE_NAMES: Dict[int, str] = {
    MODE_UNKNOWN: "알 수 없음",
    MODE_RUN: "정상 운영",
    MODE_SLOW_1: "감속 1 (25 % 감속)",
    MODE_SLOW_2: "감속 2 (50 % 감속)",
    MODE_SLOW_3: "감속 3 (75 % 감속)",
    MODE_HOLD: "일시정지",
    MODE_STOP: "비상정지",
}

# 주의 — "감속 N (X %)" 의 X 는 **감속률**이다 (에이시스 사양서 표기).
#        실제 속도 배율은 1 - X 이며 아래 MODE_SPEED 가 그 값이다.
#          감속 1 = 25 % 감속 → 속도 0.75
#          감속 3 = 75 % 감속 → 속도 0.25
MODE_SPEED: Dict[int, float] = {
    MODE_UNKNOWN: 0.0,
    MODE_RUN: 1.00,
    MODE_SLOW_1: 0.75,
    MODE_SLOW_2: 0.50,
    MODE_SLOW_3: 0.25,
    MODE_HOLD: 0.0,
    MODE_STOP: 0.0,
}

# 모드 ↔ RobotCommand 필드
MODE_FIELD: Dict[int, str] = {
    MODE_RUN: "run",
    MODE_SLOW_1: "speed_down_1",
    MODE_SLOW_2: "speed_down_2",
    MODE_SLOW_3: "speed_down_3",
    MODE_HOLD: "hold",
    MODE_STOP: "stop",
}
FIELD_MODE: Dict[str, int] = {v: k for k, v in MODE_FIELD.items()}


def mode_name(mode: int) -> str:
    return MODE_NAMES.get(mode, MODE_NAMES[MODE_UNKNOWN])


def speed_ratio(mode: int) -> float:
    return MODE_SPEED.get(mode, 0.0)


def parse_mode(text: str) -> Optional[int]:
    """'stop' · '정지' · '2' 같은 표기를 모드 값으로 바꾼다 (CLI 편의)."""
    t = str(text).strip().lower()
    if t.isdigit():
        v = int(t)
        return v if v in MODE_NAMES else None
    table = {
        "run": MODE_RUN, "정상": MODE_RUN, "정상운영": MODE_RUN, "normal": MODE_RUN,
        "slow1": MODE_SLOW_1, "감속1": MODE_SLOW_1,
        "slow2": MODE_SLOW_2, "감속2": MODE_SLOW_2,
        "slow3": MODE_SLOW_3, "감속3": MODE_SLOW_3,
        "hold": MODE_HOLD, "일시정지": MODE_HOLD, "pause": MODE_HOLD,
        "stop": MODE_STOP, "정지": MODE_STOP, "멈춤": MODE_STOP, "estop": MODE_STOP,
    }
    return table.get(t.replace(" ", "").replace("_", ""))


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
            return MODE_STOP
        if self.hold:
            return MODE_HOLD
        if self.speed_down_3:
            return MODE_SLOW_3
        if self.speed_down_2:
            return MODE_SLOW_2
        if self.speed_down_1:
            return MODE_SLOW_1
        if self.run:
            return MODE_RUN
        return MODE_UNKNOWN

    @property
    def mode_name(self) -> str:
        return mode_name(self.mode)

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
                           f"(현재 '{keep}' · {cur.source if cur else '-'} 우선)")
        setattr(self, slot, e)

        # 정지 · 일시정지를 걸면 남아 있던 감속 고정은 의미가 없으므로 정리한다.
        # 반대로 '정상 운영' 을 명시적으로 요청하면 감속 고정까지 함께 푼다
        # (단 정지 · 일시정지 고정은 clear_latched 로만 해제된다 — 안전상 의도).
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
        # (정상 운영 지령 + 감속 고정이 겹치면 의미 있는 제약은 감속 쪽이다)
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
                    f"모드 변경 → {a.mode_name} (속도 {a.speed_ratio * 100:.0f} %) "
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
