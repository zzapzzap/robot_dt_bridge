"""안전 모드 CLI — 서비스 호출을 짧게 쓰기 위한 도구.

    ros2 run robot_bridge mode_cli 감속2 --reason "작업자 접근"
    ros2 run robot_bridge mode_cli stop
    ros2 run robot_bridge mode_cli 정상 --clear        # 정지 고정까지 해제
    ros2 run robot_bridge mode_cli 감속3 --hold 10     # 10초만 유지
    ros2 run robot_bridge mode_cli --get               # 현재 모드 조회
    ros2 run robot_bridge mode_cli --watch             # 모드 변화 실시간 감시

원래 형태는 이렇다 (CLI 없이 쓸 때).

    ros2 service call /robot/loading/set_mode robot_bridge_msgs/srv/SetSafetyMode \
      "{mode: 3, source: 'operator', reason: '작업자 접근', hold_seconds: 0.0}"
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node

from robot_bridge_msgs.msg import SafetyMode
from robot_bridge_msgs.srv import GetSafetyMode, SetSafetyMode

from .safety_gate import MODE_NAMES, mode_name, parse_mode


def _fmt(state: SafetyMode) -> str:
    latch = ("무기한" if state.latched and state.latch_remaining_s <= 0
             else f"{state.latch_remaining_s:.1f}s" if state.latched else "—")
    link = "정상" if state.link_ok else "두절"
    return (f"{state.mode_name:14s} 속도 {state.speed_ratio * 100:3.0f} %  "
            f"[{state.source}] 고정 {latch}  링크 {link}"
            + (f"  · {state.reason}" if state.reason else ""))


class ModeCli(Node):
    def __init__(self, ns: str) -> None:
        super().__init__("mode_cli")
        self.ns = ns.rstrip("/")

    def call_set(self, mode: int, source: str, reason: str,
                 hold: float, clear: bool) -> int:
        cli = self.create_client(SetSafetyMode, self.ns + "/set_mode")
        if not cli.wait_for_service(timeout_sec=3.0):
            print(f"서비스를 찾을 수 없습니다 : {self.ns}/set_mode")
            print("브리지가 떠 있는지 확인하세요 — ros2 launch robot_bridge bringup.launch.py")
            return 2
        req = SetSafetyMode.Request()
        req.mode = mode
        req.source = source
        req.reason = reason
        req.hold_seconds = float(hold)
        req.clear_latched = bool(clear)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            print("응답 없음 (타임아웃)")
            return 2
        r = fut.result()
        mark = "수락" if r.accepted else "거절"
        print(f"[{mark}] 요청 {mode_name(mode)} → 현재 {r.applied_mode_name}")
        if r.message and r.message != "적용됨":
            print(f"       {r.message}")
        return 0 if r.accepted else 1

    def call_get(self) -> int:
        cli = self.create_client(GetSafetyMode, self.ns + "/get_mode")
        if not cli.wait_for_service(timeout_sec=3.0):
            print(f"서비스를 찾을 수 없습니다 : {self.ns}/get_mode")
            return 2
        fut = cli.call_async(GetSafetyMode.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            print("응답 없음 (타임아웃)")
            return 2
        print(_fmt(fut.result().state))
        return 0

    def watch(self) -> int:
        print(f"{self.ns}/mode 감시 — Ctrl+C 로 종료\n")
        self._last = None

        def cb(msg: SafetyMode) -> None:
            key = (msg.mode, msg.source, msg.latched)
            if key != self._last:
                stamp = f"{msg.header.stamp.sec % 100000}.{msg.header.stamp.nanosec // 10**8}"
                print(f"  [{stamp:>8s}] {_fmt(msg)}")
                self._last = key

        self.create_subscription(SafetyMode, self.ns + "/mode", cb, 10)
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            print()
        return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="안전 모드 설정 · 조회",
        epilog="모드 : " + " · ".join(f"{k}={v}" for k, v in MODE_NAMES.items() if k),
    )
    ap.add_argument("mode", nargs="?",
                    help="정상 | 감속1 | 감속2 | 감속3 | 일시정지 | 정지 (숫자·영문도 가능)")
    ap.add_argument("--robot", default="loading", help="로봇 id (기본 loading)")
    ap.add_argument("--ns", default=None, help="네임스페이스 직접 지정 (예 /robot/loading)")
    ap.add_argument("--source", default="operator")
    ap.add_argument("--reason", default="")
    ap.add_argument("--hold", type=float, default=0.0, help="유지 시간(초). 0=무기한")
    ap.add_argument("--clear", action="store_true", help="기존 고정 지령을 먼저 해제")
    ap.add_argument("--get", action="store_true", help="현재 모드 조회")
    ap.add_argument("--watch", action="store_true", help="모드 변화 감시")
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])

    ns = a.ns or f"/robot/{a.robot}"
    rclpy.init()
    node = ModeCli(ns)
    try:
        if a.watch:
            return node.watch()
        if a.get or not a.mode:
            return node.call_get()
        m = parse_mode(a.mode)
        if m is None:
            print(f"알 수 없는 모드 : {a.mode!r}")
            print("사용 가능 : 정상 · 감속1 · 감속2 · 감속3 · 일시정지 · 정지")
            return 2
        return node.call_set(m, a.source, a.reason, a.hold, a.clear)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
