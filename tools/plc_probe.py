#!/usr/bin/env python3
"""PLC 접속 진단 — ROS 2 없이 단독 실행 가능.

현장에서 제일 먼저 돌려 보는 도구. 연결 → 응답코드 → 덤프 → 각도환산 순으로
어디서 막히는지 바로 보여 준다.

    python3 tools/plc_probe.py --host 192.168.10.30 --port 9000
    python3 tools/plc_probe.py --host 127.0.0.1 --port 5010 --watch
    python3 tools/plc_probe.py --dump D1000 14
    # --write는 승인된 FAT에서 실제 PLC-local 주소가 확정된 뒤에만 사용한다.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:                                   # 윈도우 cp949 콘솔에서도 한글·기호가 깨지지 않게
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "robot_bridge"))

from robot_bridge.mc_client import (  # noqa: E402
    McClient, McConfig, McError, words_to_dword,
)
from robot_bridge.config_loader import PlcBridgeConfig  # noqa: E402


def load_defaults(profile: str):
    try:
        config = PlcBridgeConfig.load(ROOT / "config", profile=profile)
        instance = list(config.enabled_instances())[0]
        connection = {
            name: getattr(config.connection, name)
            for name in McConfig.__dataclass_fields__
        }
        contract = {
            "read_block": {
                "head": instance.read_head,
                "words": instance.read_words,
            },
            "axes": [
                {"name": axis.name, "offset": axis.offset}
                for axis in instance.axes
            ],
            "scale": list(instance.scale),
            "dir": list(instance.dir),
            "offset": list(instance.offset),
        }
        return connection, contract
    except Exception:
        return {}, {}


def fmt(v: int) -> str:
    return f"{v:5d} (0x{v:04X})"


def main() -> int:
    ap = argparse.ArgumentParser(description="MELSEC MC 프로토콜 진단")
    ap.add_argument("--profile", default="field", help="plc.yaml 프로파일")
    ap.add_argument("--host")
    ap.add_argument("--port", type=int)
    ap.add_argument("--frame", choices=["3E", "4E"])
    ap.add_argument("--dump", nargs=2, metavar=("DEVICE", "COUNT"),
                    help="예: --dump D1000 14")
    ap.add_argument("--write", nargs="+", metavar="DEVICE/VALUE",
                    help="예: --write D2000 0 1 0")
    ap.add_argument("--watch", action="store_true", help="6축 각도를 계속 표시")
    ap.add_argument("--hz", type=float, default=5.0)
    a = ap.parse_args()

    conn, full = load_defaults(a.profile)
    if a.host:
        conn["host"] = a.host
    if a.port:
        conn["port"] = a.port
    if a.frame:
        conn["frame"] = a.frame
    if not conn.get("host"):
        print("host 를 지정하세요 (--host 또는 config/plc.yaml)")
        return 2

    cfg = McConfig.from_dict(conn)
    print(f"■ 접속 시도  {cfg.host}:{cfg.port}  frame={cfg.frame} {cfg.protocol}")
    client = McClient(cfg)
    try:
        client.connect()
    except OSError as e:
        print(f"  ✗ TCP 연결 실패 : {e}")
        print("    → 케이블 · IP · 게이트웨이 · PLC 내장 Ethernet 개방설정 확인")
        print("    → docs/02_plc_setup.md 3장 참조")
        return 1
    print("  ✓ TCP 연결 성공")

    head = (full.get("read_block", {}) or {}).get("head", "D1000")
    words_n = int((full.get("read_block", {}) or {}).get("words", 14))

    try:
        if a.write:
            dev, vals = a.write[0], [int(x) for x in a.write[1:]]
            client.write_words(dev, vals)
            print(f"  ✓ write {dev} ← {vals}")
            return 0

        if a.dump:
            head, words_n = a.dump[0], int(a.dump[1])

        words = client.read_words(head, words_n)
        print(f"  ✓ MC 응답 정상 (end code 0x0000) — {head} × {words_n}")
        print()
        base = int("".join(ch for ch in head if ch.isdigit()))
        for i, w in enumerate(words):
            print(f"    D{base + i:<6d} {fmt(w)}")
        print()

        if not a.dump:
            axes = full.get("axes", [])
            names = [x["name"] for x in axes] or [f"J{i+1}" for i in range(6)]
            offs = [int(x["offset"]) for x in axes] or [2, 4, 6, 8, 10, 12]
            scales = list(full.get("scale", [0.01] * 6))
            directions = list(full.get("dir", [1] * 6))
            zero_offsets = list(full.get("offset", [0.0] * 6))
            raw = [words_to_dword(words[o], words[o + 1]) for o in offs]
            print("  6축 원시값 → 각도 (마지막 두 자리=소수부)")
            for n, r, scale, direction, zero in zip(
                names, raw, scales, directions, zero_offsets
            ):
                degrees = r * scale * direction + zero
                print(f"    {n:<4s} raw {r:>10d}   →  {degrees:9.2f}°")

        if a.watch:
            print("\n  --watch : Ctrl+C 로 종료\n")
            offs = [int(x["offset"]) for x in full.get("axes", [])] or [2, 4, 6, 8, 10, 12]
            scales = list(full.get("scale", [0.01] * 6))
            directions = list(full.get("dir", [1] * 6))
            zero_offsets = list(full.get("offset", [0.0] * 6))
            while True:
                w = client.read_words(head, words_n)
                raw = [words_to_dword(w[o], w[o + 1]) for o in offs]
                line = "  ".join(
                    f"{r * scale * direction + zero:10.2f}"
                    for r, scale, direction, zero in zip(
                        raw, scales, directions, zero_offsets
                    )
                )
                print(f"\r  {line}   state={w[0]}", end="", flush=True)
                time.sleep(1.0 / max(a.hz, 0.5))
    except McError as e:
        print(f"  ✗ {e}")
        print("    → 디바이스 주소 범위 · 프레임(3E/4E) · 바이너리/ASCII 설정 확인")
        return 1
    except KeyboardInterrupt:
        print()
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
