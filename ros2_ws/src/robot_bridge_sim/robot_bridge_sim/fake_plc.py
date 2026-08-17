"""PLC 없이 브리지를 검증하기 위한 가상 MELSEC 서버.

MC 3E 바이너리 프레임의 일괄읽기(0x0401) · 일괄쓰기(0x1401) 만 처리한다.
D 레지스터를 딕셔너리로 흉내 내고, 6축을 사인파로 움직여 준다.

    python3 -m robot_bridge_sim.fake_plc --port 5010
"""

from __future__ import annotations

import argparse
import math
import socket
import socketserver
import struct
import threading
import time
from typing import Dict

DEVICE_D = 0xA8
DEVICE_CODES_ASCII = {"D": 0xA8, "W": 0xB4, "R": 0xAF, "ZR": 0xB0,
                      "M": 0x90, "X": 0x9C, "Y": 0x9D, "B": 0xA0, "L": 0x92}


class Memory:
    """워드 단위 가상 메모리. 키는 (디바이스코드, 주소)."""

    def __init__(self) -> None:
        self._w: Dict[tuple, int] = {}
        self._lock = threading.Lock()
        self.t0 = time.time()

    def read(self, code: int, addr: int, count: int):
        with self._lock:
            return [self._w.get((code, addr + i), 0) for i in range(count)]

    def write(self, code: int, addr: int, values) -> None:
        with self._lock:
            for i, v in enumerate(values):
                self._w[(code, addr + i)] = v & 0xFFFF

    def set_dword(self, addr: int, value: int) -> None:
        lo, hi = struct.unpack("<HH", struct.pack("<i", value))
        self.write(DEVICE_D, addr, [lo, hi])

    def animate(self) -> None:
        """6축을 서로 다른 주기로 흔들어 준다 (밀리도 단위 = scale 0.001)."""
        t = time.time() - self.t0
        amps = [90, 45, 60, 120, 60, 180]        # degree
        freqs = [0.10, 0.13, 0.17, 0.07, 0.23, 0.05]
        for i in range(6):
            deg = amps[i] * math.sin(2 * math.pi * freqs[i] * t)
            self.set_dword(1002 + i * 2, int(round(deg * 1000)))   # 0.001 deg/LSB
        self.write(DEVICE_D, 1000, [1])          # 운전상태 = 1(운전중)
        # D1100 상태 비트 : 평소 0, 20초마다 2초간 hold 를 켜 본다
        hold = 1 if (int(t) % 20) < 2 else 0
        self.write(DEVICE_D, 1100, [hold & 0x01])


MEM = Memory()


class Handler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        sock: socket.socket = self.request
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"[fake-plc] 접속 {self.client_address}")
        try:
            while True:
                first = self._recv(2)
                if first is None:
                    break
                if first == b"50":               # ASCII 프레임 ("5000…")
                    head = self._recv(2 + 10 + 4)
                    if head is None:
                        break
                    length = int(head[-4:].decode("ascii"), 16)
                    body = self._recv(length)
                    if body is None:
                        break
                    sock.sendall(self._process_ascii(body.decode("ascii")))
                    continue
                # 바이너리 프레임 : 50 00 | NET PC IO(2) STN | LEN(2)
                head = self._recv(7)
                if head is None:
                    break
                (length,) = struct.unpack("<H", head[5:7])
                body = self._recv(length)
                if body is None:
                    break
                sock.sendall(self._process(body))
        except (ConnectionError, OSError):
            pass
        finally:
            print(f"[fake-plc] 종료 {self.client_address}")

    def _recv(self, n: int):
        buf = b""
        while len(buf) < n:
            c = self.request.recv(n - len(buf))
            if not c:
                return None
            buf += c
        return buf

    @staticmethod
    def _reply(end_code: int, data: bytes = b"") -> bytes:
        payload = struct.pack("<H", end_code) + data
        return (struct.pack("<H", 0x00D0)
                + struct.pack("<BBHB", 0x00, 0xFF, 0x03FF, 0x00)
                + struct.pack("<H", len(payload)) + payload)

    def _process(self, body: bytes) -> bytes:
        # body = TIMER(2) CMD(2) SUB(2) | 요청데이터
        if len(body) < 6:
            return self._reply(0xC061)
        _timer, cmd, sub = struct.unpack("<HHH", body[:6])
        req = body[6:]

        if cmd == 0x0401 and sub == 0x0000:              # 일괄 읽기 (워드)
            if len(req) < 6:
                return self._reply(0xC061)
            addr = int.from_bytes(req[0:3], "little")
            code = req[3]
            (count,) = struct.unpack("<H", req[4:6])
            if count > 960:
                return self._reply(0xC051)
            words = MEM.read(code, addr, count)
            return self._reply(0x0000, struct.pack(f"<{count}H", *words))

        if cmd == 0x1401 and sub == 0x0000:              # 일괄 쓰기 (워드)
            if len(req) < 6:
                return self._reply(0xC061)
            addr = int.from_bytes(req[0:3], "little")
            code = req[3]
            (count,) = struct.unpack("<H", req[4:6])
            need = 6 + count * 2
            if len(req) < need:
                return self._reply(0xC058)
            vals = struct.unpack(f"<{count}H", req[6:need])
            MEM.write(code, addr, vals)
            tag = {2000: "정지버퍼", 3000: "속도제한버퍼"}.get(addr, f"D{addr}")
            print(f"[fake-plc] write {tag} ← {list(vals)}")
            return self._reply(0x0000)

        return self._reply(0xC059)                       # 미지원 명령

    # ------------------------------------------------------------ ASCII 처리
    @staticmethod
    def _reply_ascii(end_code: int, data: str = "") -> bytes:
        payload = f"{end_code:04X}" + data
        return (f"D000" "00FF03FF00" f"{len(payload):04X}" + payload).encode("ascii")

    def _process_ascii(self, body: str) -> bytes:
        # body = TTTT CCCC SSSS | 요청데이터
        if len(body) < 12:
            return self._reply_ascii(0xC061)
        cmd = int(body[4:8], 16)
        sub = int(body[8:12], 16)
        req = body[12:]
        if len(req) < 12:
            return self._reply_ascii(0xC061)

        name = req[0:2].replace("*", "").strip()
        code = DEVICE_CODES_ASCII.get(name)
        if code is None:
            return self._reply_ascii(0xC05B)
        base = 16 if name in ("W", "X", "Y", "B") else 10
        addr = int(req[2:8], base)
        count = int(req[8:12], 16)

        if cmd == 0x0401 and sub == 0x0000:
            if count > 960:
                return self._reply_ascii(0xC051)
            words = MEM.read(code, addr, count)
            return self._reply_ascii(0x0000, "".join(f"{w:04X}" for w in words))

        if cmd == 0x1401 and sub == 0x0000:
            need = 12 + count * 4
            if len(req) < need:
                return self._reply_ascii(0xC058)
            vals = [int(req[12 + i * 4:16 + i * 4], 16) for i in range(count)]
            MEM.write(code, addr, vals)
            tag = {2000: "정지버퍼", 3000: "속도제한버퍼"}.get(addr, f"D{addr}")
            print(f"[fake-plc] write(ascii) {tag} ← {vals}")
            return self._reply_ascii(0x0000)

        return self._reply_ascii(0xC059)


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def animate_forever(period: float = 0.02) -> None:
    while True:
        MEM.animate()
        time.sleep(period)


def serve(host: str = "127.0.0.1", port: int = 5010) -> None:
    threading.Thread(target=animate_forever, daemon=True).start()
    with Server((host, port), Handler) as srv:
        print(f"[fake-plc] MC 3E 서버 대기 : {host}:{port}")
        srv.serve_forever()


def main() -> None:
    ap = argparse.ArgumentParser(description="가상 MELSEC PLC (MC 3E)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5010)
    a = ap.parse_args()
    serve(a.host, a.port)


if __name__ == "__main__":
    main()
