"""여러 발행자(XDI · XAG · Unity · 운전원)의 명령을 하나로 중재한다.

규칙
  1. 우선순위가 높은 명령이 이긴다 (plc.yaml safety.priority).
  2. 같은 우선순위면 나중에 온 것이 이긴다.
  3. command_timeout_ms 안에 갱신되지 않은 명령은 소멸한다.
  4. 긴급(run/hold/stop)과 감속(1/2/3)은 각각 배타적이며 서로 독립이다.
  5. PLC 링크가 끊기면 fail_safe 상태를 강제한다.

이 규칙은 CDR 후속조치 AI-102 「XDI ↔ XAG 중재 규칙」의 구현부다.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Optional


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

    def key(self) -> tuple:
        return (self.run, self.hold, self.stop,
                self.speed_down_1, self.speed_down_2, self.speed_down_3)


@dataclass
class _Entry:
    priority: int
    stamp: float
    source: str
    reason: str
    field: str          # "stop" | "hold" | "run" | "speed_down_N"


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

    # ------------------------------------------------------------------ 입력
    def submit(self, msg) -> None:
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

    @staticmethod
    def _pick(msg, fields) -> Optional[str]:
        for f in fields:                      # 우선순위 높은 필드부터 검사
            if getattr(msg, f, False):
                return f
        return None

    def _beats(self, new: _Entry, cur: Optional[_Entry]) -> bool:
        if cur is None or self._expired(cur):
            return True
        if new.priority > cur.priority:
            return True
        return new.priority == cur.priority   # 동순위는 최신이 이김

    def _expired(self, e: _Entry) -> bool:
        return (time.time() - e.stamp) > self.cmd_timeout

    # ------------------------------------------------------------------ 출력
    def resolve(self, link_stale: bool = False) -> Applied:
        if self._urgent and self._expired(self._urgent):
            self._urgent = None
        if self._slow and self._expired(self._slow):
            self._slow = None

        a = Applied()
        if link_stale and self.fail_safe in ("hold", "stop"):
            setattr(a, self.fail_safe, True)
            a.run = False
            a.source, a.reason = "watchdog", "PLC 링크 두절 — fail-safe 적용"
        elif self._urgent:
            a.run = a.hold = a.stop = False
            setattr(a, self._urgent.field, True)
            a.source, a.reason = self._urgent.source, self._urgent.reason
        # 정지 · 일시정지 중에는 감속 지령을 겹쳐 쓰지 않는다
        if not (a.stop or a.hold) and self._slow:
            setattr(a, self._slow.field, True)
            if a.source == "default":
                a.source, a.reason = self._slow.source, self._slow.reason

        if a.key() != self._applied.key() and self._log is not None:
            self._log.info(
                f"지령 변경 → run={a.run} hold={a.hold} stop={a.stop} "
                f"sd={int(a.speed_down_1)}{int(a.speed_down_2)}{int(a.speed_down_3)} "
                f"[{a.source}] {a.reason}"
            )
        self._applied = a
        return a

    def last_applied(self) -> Applied:
        return self._applied
