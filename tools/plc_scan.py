#!/usr/bin/env python3
"""PLC 자동 탐색 — IP · 포트 · 프레임 · 코드를 모르는 상태에서 찾아낸다.

랜선만 꽂고 이걸 돌리면 된다. ROS 2 없이 표준 라이브러리만 쓴다.

    python3 tools/plc_scan.py                       # 전자동
    python3 tools/plc_scan.py --subnet 192.168.0.0/24
    python3 tools/plc_scan.py --host 192.168.0.10   # IP 는 아는데 포트를 모를 때
    python3 tools/plc_scan.py --passive 20          # 수동 청취만 (아무것도 안 보냄)

탐색 원리
  1. 인터페이스 · 링크 · 현재 IP 확인
  2. 수동 청취 — 상대가 먼저 떠드는 걸 듣는다 (ARP · MELSOFT 브로드캐스트)
  3. ARP 테이블 — 이미 통신한 흔적이 있으면 거기 다 있다
  4. TCP 포트 훑기 — 후보 대역 × MELSEC 상용 포트
  5. **MC 프로토콜 확인** — 열린 포트마다 3E/4E × 바이너리/ASCII 4조합으로
     실제 요청을 보내 본다. 정상응답이든 프로토콜 오류응답이든 **응답이 오면
     그건 MC 포트다.** (0xC056 같은 오류도 신원 확인으로는 성공이다)

찾으면 config/plc.yaml 에 그대로 붙일 수 있는 블록을 출력한다.
"""

from __future__ import annotations

import argparse
import ipaddress
import re
import socket
import struct
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "robot_bridge"))

from robot_bridge.mc_client import (  # noqa: E402
    CMD_BATCH_READ, SUB_WORD, END_CODES, McClient, McConfig, McError, parse_device,
)

# MELSEC 계열에서 실제로 자주 열려 있는 포트들
CANDIDATE_PORTS = [
    5000, 5001, 5002, 5003, 5004, 5005, 5006, 5007, 5010, 5011, 5012,
    1281,          # MELSOFT / GX 계열
    2000, 2001,
    4999, 5008, 5009,
    45237, 45238,  # 일부 iQ-R 구성
    502,           # Modbus/TCP 로 뚫려 있는 경우 (MC 아님 — 참고용)
]

# 자국 IP 를 못 받았을 때 찍어볼 대역 (미쓰비시 기본값 포함)
FALLBACK_SUBNETS = [
    "192.168.0.0/24",
    "192.168.1.0/24",
    "192.168.3.0/24",     # 미쓰비시 공장 출하 기본값이 흔히 192.168.3.39
    "192.168.10.0/24",
    "10.0.0.0/24",
]

FRAMES = ["3E", "4E"]
CODECS = ["binary", "ascii"]

C_OK, C_WARN, C_DIM, C_END = "\033[92m", "\033[93m", "\033[90m", "\033[0m"
if sys.platform == "win32":
    try:
        import ctypes
        ctypes.windll.kernel32.SetConsoleMode(
            ctypes.windll.kernel32.GetStdHandle(-11), 7)
    except Exception:
        C_OK = C_WARN = C_DIM = C_END = ""


def hr(title: str = "") -> None:
    print(f"\n{'─' * 70}")
    if title:
        print(f" {title}")
        print("─" * 70)


# ═══════════════════════════════════════════════════ 1. 인터페이스
def local_addresses() -> List[Tuple[str, str]]:
    """(IP, 설명) 목록. 루프백 제외."""
    out: List[Tuple[str, str]] = []
    try:
        host = socket.gethostname()
        for info in socket.getaddrinfo(host, None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127.") and (ip, "") not in out:
                out.append((ip, ""))
    except OSError:
        pass
    # UDP 트릭 — 기본 경로로 나가는 인터페이스의 IP
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        if ip not in [a for a, _ in out]:
            out.append((ip, "기본 경로"))
    except OSError:
        pass
    return out


def show_interfaces() -> List[str]:
    hr("1. 인터페이스 · 링크")
    addrs = local_addresses()
    if not addrs:
        print(f"  {C_WARN}IPv4 주소가 하나도 없습니다.{C_END}")
        print("  → 랜선 연결 확인, 또는 NIC 에 고정 IP 를 먼저 주십시오 (아래 참조)")
    for ip, note in addrs:
        print(f"  {C_OK}·{C_END} {ip:<16s} {note}")

    # 링크 상태 (있으면)
    try:
        if sys.platform == "win32":
            r = subprocess.run(["netsh", "interface", "show", "interface"],
                               capture_output=True, text=True, timeout=5,
                               encoding="utf-8", errors="replace")
            for ln in (r.stdout or "").splitlines():
                if "연결" in ln or "Connected" in ln or "Disconnected" in ln:
                    print(f"  {C_DIM}{ln.strip()}{C_END}")
        else:
            r = subprocess.run(["ip", "-br", "link"], capture_output=True,
                               text=True, timeout=5)
            for ln in (r.stdout or "").splitlines():
                print(f"  {C_DIM}{ln.strip()}{C_END}")
    except Exception:
        pass
    return [a for a, _ in addrs]


# ═══════════════════════════════════════════════════ 2. 수동 청취
def passive_listen(seconds: int) -> List[str]:
    """아무것도 보내지 않고 브로드캐스트만 듣는다. 상대가 떠들면 IP 가 드러난다."""
    hr(f"2. 수동 청취 ({seconds}초) — 아무것도 보내지 않습니다")
    found: Dict[str, str] = {}
    stop = threading.Event()
    ports = [5562, 5563, 5000, 5001, 1281, 67, 68, 137, 138, 5353, 1900]

    def listen(port: int) -> None:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            except OSError:
                pass
            s.settimeout(0.5)
            s.bind(("", port))
        except OSError:
            return
        while not stop.is_set():
            try:
                data, addr = s.recvfrom(2048)
            except (socket.timeout, OSError):
                continue
            ip = addr[0]
            if ip not in found:
                found[ip] = f"UDP :{port} ({len(data)} B)"
                print(f"  {C_OK}▸{C_END} {ip:<16s} {found[ip]}")
        s.close()

    threads = [threading.Thread(target=listen, args=(p,), daemon=True) for p in ports]
    for t in threads:
        t.start()
    for i in range(seconds):
        print(f"\r  듣는 중… {seconds - i}s   ", end="", flush=True)
        time.sleep(1)
    stop.set()
    print("\r" + " " * 30 + "\r", end="")
    if not found:
        print(f"  {C_DIM}브로드캐스트 없음 (조용한 장비이면 정상){C_END}")
    return list(found)


# ═══════════════════════════════════════════════════ 3. ARP 테이블
def arp_neighbors() -> List[str]:
    hr("3. ARP 테이블 — 이미 통신한 이웃")
    ips: List[str] = []
    try:
        cmd = ["arp", "-a"] if sys.platform == "win32" else ["ip", "neigh"]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=8,
                           encoding="utf-8", errors="replace")
        for ln in (r.stdout or "").splitlines():
            m = re.search(r"(\d+\.\d+\.\d+\.\d+)", ln)
            if not m:
                continue
            ip = m.group(1)
            if ip.startswith(("127.", "224.", "239.")) or ip.endswith(".255"):
                continue
            mac = re.search(r"([0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2}", ln)
            if ip not in ips:
                ips.append(ip)
                vendor = mac_vendor(mac.group(0)) if mac else ""
                print(f"  · {ip:<16s} {mac.group(0) if mac else '':<20s} {vendor}")
    except Exception as e:
        print(f"  {C_DIM}조회 실패 : {e}{C_END}")
    if not ips:
        print(f"  {C_DIM}비어 있음{C_END}")
    return ips


MITSUBISHI_OUI = {"00:00:70", "00:1a:e3", "08:00:70", "00:80:8f", "e4:e6:6e"}


def mac_vendor(mac: str) -> str:
    oui = mac.lower().replace("-", ":")[:8]
    if oui in MITSUBISHI_OUI:
        return f"{C_OK}← 미쓰비시 계열일 가능성{C_END}"
    return ""


# ═══════════════════════════════════════════════════ 4. 포트 훑기
def tcp_open(ip: str, port: int, timeout: float = 0.35) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def sweep(subnets: List[str], ports: List[int], workers: int = 256
          ) -> List[Tuple[str, int]]:
    hr(f"4. TCP 포트 훑기 — {len(subnets)}개 대역 × {len(ports)}개 포트")
    targets: List[Tuple[str, int]] = []
    for sn in subnets:
        try:
            net = ipaddress.ip_network(sn, strict=False)
        except ValueError:
            continue
        hosts = list(net.hosts())
        if len(hosts) > 254:
            print(f"  {C_WARN}{sn} 은 너무 넓어 앞 254개만 봅니다{C_END}")
            hosts = hosts[:254]
        for h in hosts:
            for p in ports:
                targets.append((str(h), p))

    print(f"  대상 {len(targets):,}개 … ", end="", flush=True)
    open_ports: List[Tuple[str, int]] = []
    t0 = time.time()
    with ThreadPoolExecutor(max_workers=workers) as ex:
        futs = {ex.submit(tcp_open, ip, p): (ip, p) for ip, p in targets}
        for f in as_completed(futs):
            if f.result():
                ip, p = futs[f]
                open_ports.append((ip, p))
                print(f"\n  {C_OK}▸ 열림{C_END} {ip}:{p}", end="", flush=True)
    print(f"\n  ({time.time() - t0:.1f}초 소요, 열린 포트 {len(open_ports)}개)")
    return sorted(open_ports)


# ═══════════════════════════════════════════════════ 5. MC 신원 확인
def identify(ip: str, port: int) -> Optional[dict]:
    """4조합을 시도해 MC 프로토콜인지 확정한다.

    정상응답은 물론, **프로토콜 오류응답도 신원 확인 성공**이다.
    (요청 형식을 알아듣고 오류코드를 돌려줬다는 뜻이므로)
    """
    for frame in FRAMES:
        for codec in CODECS:
            cfg = McConfig(host=ip, port=port, frame=frame, protocol=codec,
                           connect_timeout_s=1.5, read_timeout_s=1.5)
            cli = McClient(cfg)
            try:
                cli.connect()
            except OSError:
                return None
            try:
                words = cli.read_words("D0", 1)
                cli.close()
                return {"ip": ip, "port": port, "frame": frame, "codec": codec,
                        "verdict": "정상응답", "sample": words}
            except McError as e:
                cli.close()
                return {"ip": ip, "port": port, "frame": frame, "codec": codec,
                        "verdict": f"오류응답 0x{e.end_code:04X} "
                                   f"({END_CODES.get(e.end_code, '?')})",
                        "sample": None}
            except (OSError, ValueError, struct.error, UnicodeDecodeError):
                cli.close()
                continue
    return None


def probe_all(open_ports: List[Tuple[str, int]]) -> List[dict]:
    hr("5. MC 프로토콜 신원 확인 — 3E/4E × 바이너리/ASCII")
    hits: List[dict] = []
    for ip, port in open_ports:
        print(f"  {ip}:{port} … ", end="", flush=True)
        r = identify(ip, port)
        if r:
            hits.append(r)
            print(f"{C_OK}MC 확인{C_END}  {r['frame']} {r['codec']}  · {r['verdict']}")
        else:
            print(f"{C_DIM}MC 아님{C_END}")
    return hits


# ═══════════════════════════════════════════════════ 결과 출력
def report(hits: List[dict]) -> int:
    hr("결과")
    if not hits:
        print(f"  {C_WARN}MC 프로토콜 응답을 찾지 못했습니다.{C_END}\n")
        print("  탐색이 못 찾는 경우는 셋 중 하나입니다.\n")
        print("   ① 물리 연결 문제        → 링크 LED, 케이블, 스위치 포트")
        print("   ② 같은 대역이 아님       → PLC IP 를 모르면 아래 수동 설정 참고")
        print("   ③ MC 프로토콜 미개방     → 이건 스캔으로 못 뚫습니다.")
        print(f"      {C_WARN}GX Works 에서 오픈 설정을 해줘야 합니다 (에이시스 협조 필요){C_END}")
        print("      docs/02_plc_setup.md 2.2 참조\n")
        print("  대역을 모를 때 직접 찍어보는 법 :")
        print("    # 후보 대역마다 우리 쪽 IP 를 바꿔 달고 다시 스캔")
        print("    sudo ip addr add 192.168.3.100/24 dev eth0   # 미쓰비시 기본 대역")
        print("    python3 tools/plc_scan.py --subnet 192.168.3.0/24")
        return 1

    best = hits[0]
    print(f"  {C_OK}찾았습니다 — {len(hits)}곳{C_END}\n")
    for h in hits:
        print(f"   · {h['ip']}:{h['port']}  frame={h['frame']}  code={h['codec']}"
              f"   {h['verdict']}")
        if h["sample"] is not None:
            print(f"     D0 = {h['sample']}")

    print(f"\n  config/plc.yaml 의 field 프로파일에 그대로 넣으십시오 :\n")
    print(f"    field:")
    print(f"      connection:")
    print(f"        host: {best['ip']}")
    print(f"        port: {best['port']}")
    print(f'        frame: "{best["frame"]}"')
    print(f'        protocol: "{best["codec"]}"')
    print(f"\n  이어서 :")
    print(f"    python3 tools/plc_probe.py --host {best['ip']} --port {best['port']} "
          f"--frame {best['frame']}")
    print(f"    → D1000~D1013 이 실제로 읽히는지 확인 (여기서 축좌표가 보여야 합니다)")
    return 0


# ═══════════════════════════════════════════════════ main
def main() -> int:
    ap = argparse.ArgumentParser(description="MELSEC PLC 자동 탐색")
    ap.add_argument("--host", help="IP 는 아는데 포트를 모를 때")
    ap.add_argument("--subnet", action="append", help="훑을 대역 (여러 번 지정 가능)")
    ap.add_argument("--ports", help="포트 목록 (쉼표 구분). 기본은 MELSEC 상용 포트")
    ap.add_argument("--passive", type=int, default=8, help="수동 청취 시간(초). 0=생략")
    ap.add_argument("--no-sweep", action="store_true", help="포트 훑기 생략")
    a = ap.parse_args()

    print("=" * 70)
    print(" MELSEC PLC 자동 탐색")
    print("=" * 70)

    ports = ([int(x) for x in a.ports.split(",")] if a.ports else CANDIDATE_PORTS)

    # 특정 호스트만
    if a.host:
        hr(f"지정 호스트 {a.host} — 포트 {len(ports)}개 확인")
        opens = []
        with ThreadPoolExecutor(max_workers=64) as ex:
            futs = {ex.submit(tcp_open, a.host, p, 1.0): p for p in ports}
            for f in as_completed(futs):
                if f.result():
                    p = futs[f]
                    opens.append((a.host, p))
                    print(f"  {C_OK}▸ 열림{C_END} {a.host}:{p}")
        if not opens:
            print(f"  {C_WARN}열린 포트 없음{C_END} — 개방 설정 또는 방화벽 확인")
        return report(probe_all(sorted(opens)))

    local = show_interfaces()
    candidates: List[str] = []
    if a.passive > 0:
        candidates += passive_listen(a.passive)
    candidates += arp_neighbors()

    # 후보 IP 가 이미 있으면 그것부터
    hits: List[dict] = []
    if candidates:
        hr(f"후보 {len(candidates)}곳 우선 확인")
        opens = []
        with ThreadPoolExecutor(max_workers=128) as ex:
            futs = {ex.submit(tcp_open, ip, p): (ip, p)
                    for ip in candidates for p in ports}
            for f in as_completed(futs):
                if f.result():
                    opens.append(futs[f])
                    print(f"  {C_OK}▸ 열림{C_END} {futs[f][0]}:{futs[f][1]}")
        if opens:
            hits = probe_all(sorted(opens))

    if not hits and not a.no_sweep:
        subnets = a.subnet or []
        if not subnets:
            for ip in local:
                subnets.append(str(ipaddress.ip_network(ip + "/24", strict=False)))
            if not subnets:
                subnets = FALLBACK_SUBNETS
                print(f"\n  {C_WARN}자국 IP 가 없어 기본 대역을 찍어 봅니다{C_END}")
        hits = probe_all(sweep(list(dict.fromkeys(subnets)), ports))

    return report(hits)


if __name__ == "__main__":
    raise SystemExit(main())
