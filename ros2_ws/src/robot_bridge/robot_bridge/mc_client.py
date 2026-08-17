"""MELSEC 통신 프로토콜(MC Protocol) 클라이언트.

QnUCPU 내장 Ethernet 포트의 3E 프레임(바이너리)을 기본으로 하고, iQ-R 계열용
4E 프레임도 지원한다. 외부 패키지 의존 없이 표준 라이브러리만 사용한다.

참고 문서 (`[에이시스] 로봇통신/` 폴더)
  - Q_Corresponding_MELSEC_Communication_Protocol_Reference.pdf
  - QnUCPU_사용자_매뉴얼_내장_Ethernet_포토_통신편_17.09.pdf

3E 요청 프레임 (바이너리)
    50 00 | NET | PC | IO(2) | STN | LEN(2) | TIMER(2) | CMD(2) | SUB(2) | 요청데이터
    ^부헤더                            ^TIMER 부터의 바이트 수

3E 응답 프레임
    D0 00 | NET | PC | IO(2) | STN | LEN(2) | END(2) | 응답데이터
"""

from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

# ---------------------------------------------------------------- 디바이스 코드
DEVICE_CODES = {
    "D": 0xA8,   # 데이터 레지스터
    "W": 0xB4,   # 링크 레지스터
    "R": 0xAF,   # 파일 레지스터
    "ZR": 0xB0,
    "M": 0x90,   # 내부 릴레이
    "X": 0x9C,
    "Y": 0x9D,
    "B": 0xA0,
    "L": 0x92,
}

CMD_BATCH_READ = 0x0401
CMD_BATCH_WRITE = 0x1401
SUB_WORD = 0x0000
SUB_BIT = 0x0001

# 자주 만나는 종료 코드 → 사람이 읽을 수 있는 설명
END_CODES = {
    0x0000: "정상",
    0x0055: "온라인 변경 불가",
    0xC050: "ASCII/바이너리 설정 불일치",
    0xC051: "읽기/쓰기 점수 초과",
    0xC056: "요청 디바이스 범위 초과",
    0xC058: "요청 데이터 길이 불일치",
    0xC059: "명령·서브명령 지정 오류",
    0xC05B: "지정 디바이스에 접근 불가",
    0xC05C: "요청 내용 오류",
    0xC060: "비트 디바이스 지정 오류",
    0xC061: "요청 데이터 길이 오류",
    0xC200: "원격 패스워드 오류",
    0xC201: "포트가 잠김(원격 패스워드)",
}


class McError(RuntimeError):
    """MC 프로토콜 계층에서 발생한 오류."""

    def __init__(self, end_code: int, context: str = ""):
        self.end_code = end_code
        desc = END_CODES.get(end_code, "알 수 없는 종료 코드")
        super().__init__(f"MC 오류 0x{end_code:04X} ({desc}){' — ' + context if context else ''}")


def parse_device(text: str) -> tuple[int, int]:
    """'D1000' → (0xA8, 1000). 진수는 10진(D/W 는 16진 표기도 허용)."""
    text = text.strip().upper()
    for code_len in (2, 1):                       # ZR 처럼 2글자 코드 우선
        head, tail = text[:code_len], text[code_len:]
        if head in DEVICE_CODES and tail:
            base = 16 if head in ("W", "X", "Y", "B") else 10
            try:
                return DEVICE_CODES[head], int(tail, base)
            except ValueError:
                continue
    raise ValueError(f"디바이스 표기를 해석할 수 없음: {text!r}")


@dataclass
class McConfig:
    host: str = "192.168.0.10"
    port: int = 5000
    frame: str = "3E"                 # "3E" | "4E"
    protocol: str = "binary"          # 현재 binary 만 지원
    network_no: int = 0x00
    pc_no: int = 0xFF
    io_no: int = 0x03FF
    station_no: int = 0x00
    monitor_timer_250ms: int = 16
    connect_timeout_s: float = 3.0
    read_timeout_s: float = 1.0
    reconnect_backoff_s: Sequence[float] = field(default_factory=lambda: [0.5, 1.0, 2.0, 5.0])

    @classmethod
    def from_dict(cls, d: dict) -> "McConfig":
        known = {f for f in cls.__dataclass_fields__}          # noqa: SLF001
        return cls(**{k: v for k, v in d.items() if k in known})


class McClient:
    """단일 PLC 세션. 스레드 안전하지 않으므로 노드당 1개씩 사용한다."""

    def __init__(self, cfg: McConfig, logger=None):
        self.cfg = cfg
        self._sock: Optional[socket.socket] = None
        self._serial = 0                     # 4E 프레임 시리얼 번호
        self._fail_count = 0
        self._log = logger

    def _info(self, msg: str) -> None:
        if self._log is not None:
            self._log.info(msg)

    # ------------------------------------------------------------ 연결 관리
    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> None:
        self.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.cfg.connect_timeout_s)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.connect((self.cfg.host, self.cfg.port))
        s.settimeout(self.cfg.read_timeout_s)
        self._sock = s
        self._fail_count = 0
        self._info(f"PLC 연결 : {self.cfg.host}:{self.cfg.port} ({self.cfg.frame} {self.cfg.protocol})")

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def backoff_delay(self) -> float:
        b = list(self.cfg.reconnect_backoff_s) or [1.0]
        return b[min(self._fail_count, len(b) - 1)]

    def note_failure(self) -> None:
        self._fail_count += 1
        self.close()

    @property
    def is_ascii(self) -> bool:
        return self.cfg.protocol.lower().startswith("a")

    # ------------------------------------------------------------ 프레임 조립
    def _build(self, command: int, subcommand: int, payload: bytes) -> bytes:
        if self.is_ascii:
            return self._build_ascii(command, subcommand, payload)
        body = struct.pack("<HHH", self.cfg.monitor_timer_250ms, command, subcommand) + payload
        # LEN 은 TIMER 부터 끝까지의 바이트 수
        length = len(body)
        head = struct.pack(
            "<BBHB",
            self.cfg.network_no,
            self.cfg.pc_no,
            self.cfg.io_no,
            self.cfg.station_no,
        )
        if self.cfg.frame.upper() == "4E":
            self._serial = (self._serial + 1) & 0xFFFF
            sub = struct.pack("<HHH", 0x0054, self._serial, 0x0000)
        else:
            sub = struct.pack("<H", 0x0050)
        return sub + head + struct.pack("<H", length) + body

    def _build_ascii(self, command: int, subcommand: int, payload: bytes) -> bytes:
        """ASCII 프레임 — 모든 필드를 대문자 16진 문자로 보낸다.

        5000 | 00 | FF | 03FF | 00 | LLLL | TTTT | CCCC | SSSS | 요청데이터
        ^부헤더                          ^TIMER 부터의 '문자 수'
        payload 는 바이너리 형식으로 받아 여기서 ASCII 로 변환한다.
        """
        req = _payload_to_ascii(command, payload)
        body = (f"{self.cfg.monitor_timer_250ms:04X}"
                f"{command:04X}{subcommand:04X}") + req
        head = (f"{self.cfg.network_no:02X}{self.cfg.pc_no:02X}"
                f"{self.cfg.io_no:04X}{self.cfg.station_no:02X}")
        if self.cfg.frame.upper() == "4E":
            self._serial = (self._serial + 1) & 0xFFFF
            sub = f"5400{self._serial:04X}0000"
        else:
            sub = "5000"
        return (sub + head + f"{len(body):04X}" + body).encode("ascii")

    def _recv_exact(self, n: int) -> bytes:
        assert self._sock is not None
        buf = b""
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("PLC 가 연결을 닫음")
            buf += chunk
        return buf

    def _transact(self, command: int, subcommand: int, payload: bytes,
                  context: str = "") -> bytes:
        if self._sock is None:
            raise ConnectionError("PLC 에 연결되어 있지 않음")
        self._sock.sendall(self._build(command, subcommand, payload))

        if self.is_ascii:
            # D000 | 00FF03FF00 | LLLL | EEEE | 응답데이터   (전부 문자)
            head_len = 12 if self.cfg.frame.upper() == "4E" else 4
            header = self._recv_exact(head_len + 10 + 4).decode("ascii")
            resp_len = int(header[-4:], 16)
            body = self._recv_exact(resp_len).decode("ascii")
            end_code = int(body[:4], 16)
            if end_code != 0:
                raise McError(end_code, context)
            return body[4:].encode("ascii")       # 문자 그대로 돌려준다

        # 응답 헤더 : 부헤더(2 또는 6) + NET/PC/IO/STN(5) + LEN(2)
        head_len = 6 if self.cfg.frame.upper() == "4E" else 2
        header = self._recv_exact(head_len + 5 + 2)
        (resp_len,) = struct.unpack("<H", header[-2:])
        body = self._recv_exact(resp_len)

        (end_code,) = struct.unpack("<H", body[:2])
        if end_code != 0:
            raise McError(end_code, context)
        return body[2:]

    # ------------------------------------------------------------ 읽기 / 쓰기
    def read_words(self, device: str, count: int) -> List[int]:
        """워드 단위 일괄 읽기. 반환값은 0~65535 부호 없는 정수 리스트."""
        code, addr = parse_device(device)
        payload = struct.pack("<I", addr)[:3] + bytes([code]) + struct.pack("<H", count)
        data = self._transact(CMD_BATCH_READ, SUB_WORD, payload,
                              context=f"read {device} × {count}")
        if self.is_ascii:
            text = data.decode("ascii")
            if len(text) < count * 4:
                raise McError(0xC058, f"응답 길이 부족 ({len(text)} < {count * 4})")
            return [int(text[i * 4:(i + 1) * 4], 16) for i in range(count)]
        if len(data) < count * 2:
            raise McError(0xC058, f"응답 길이 부족 ({len(data)} < {count * 2})")
        return list(struct.unpack(f"<{count}H", data[: count * 2]))

    def write_words(self, device: str, values: Sequence[int]) -> None:
        """워드 단위 일괄 쓰기."""
        code, addr = parse_device(device)
        n = len(values)
        payload = (struct.pack("<I", addr)[:3] + bytes([code])
                   + struct.pack("<H", n)
                   + struct.pack(f"<{n}H", *[v & 0xFFFF for v in values]))
        self._transact(CMD_BATCH_WRITE, SUB_WORD, payload,
                       context=f"write {device} × {n}")

    def read_bits(self, device: str, count: int) -> List[int]:
        """비트 단위 일괄 읽기 (1비트 = 니블 1개로 반환됨)."""
        code, addr = parse_device(device)
        payload = struct.pack("<I", addr)[:3] + bytes([code]) + struct.pack("<H", count)
        data = self._transact(CMD_BATCH_READ, SUB_BIT, payload,
                              context=f"read bit {device} × {count}")
        out: List[int] = []
        for byte in data:
            out.append((byte >> 4) & 0x0F)
            out.append(byte & 0x0F)
        return out[:count]


# ------------------------------------------------------- ASCII 페이로드 변환
#   ASCII 프레임의 디바이스 지정은 바이너리와 형식이 다르다.
#     디바이스 코드  2문자  ("D*" 처럼 부족하면 '*' 로 채움)
#     선두 디바이스  6문자  (10진 디바이스는 10진, 16진 디바이스는 16진)
#     점수          4문자  (16진)
_ASCII_DEV = {v: k for k, v in DEVICE_CODES.items()}
_HEX_DEVICES = {"W", "X", "Y", "B"}


def _payload_to_ascii(command: int, payload: bytes) -> str:
    code = payload[3]
    addr = int.from_bytes(payload[0:3], "little")
    (count,) = struct.unpack("<H", payload[4:6])
    name = _ASCII_DEV.get(code, "D")
    head = f"{addr:06X}" if name in _HEX_DEVICES else f"{addr:06d}"
    out = f"{name:*<2}{head}{count:04X}"
    if command == CMD_BATCH_WRITE:                 # 쓰기는 데이터가 뒤에 붙는다
        words = struct.unpack(f"<{count}H", payload[6:6 + count * 2])
        out += "".join(f"{w:04X}" for w in words)
    return out


# ---------------------------------------------------------------- 값 변환 유틸
def words_to_dword(lo: int, hi: int) -> int:
    """하위워드 + 상위워드 → signed 32bit (리틀엔디언)."""
    return struct.unpack("<i", struct.pack("<HH", lo & 0xFFFF, hi & 0xFFFF))[0]


def word_to_int16(w: int) -> int:
    """WORD → signed 16bit."""
    return struct.unpack("<h", struct.pack("<H", w & 0xFFFF))[0]


def bit_of(word: int, position: int) -> bool:
    return bool((word >> position) & 1)
