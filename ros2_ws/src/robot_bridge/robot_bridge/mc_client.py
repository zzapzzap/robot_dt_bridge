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

import ipaddress
import math
import socket
import struct
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any, List, Optional, Sequence

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
CMD_RANDOM_WRITE = 0x1402
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
    # When the Jetson has more than one active NIC, bind the MC session to the
    # PLC-facing address instead of relying on the default route.  ``None``
    # keeps the operating-system routing choice used by older configurations.
    source_address: Optional[str] = None
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
    def from_dict(
        cls,
        values: Mapping[str, Any],
        *,
        strict: bool = False,
    ) -> "McConfig":
        """Build a connection config from YAML-compatible values.

        The historical API ignored unrelated keys, so that remains the
        default for the small standalone tools.  The production config loader
        opts into ``strict=True`` and fails closed on a misspelled field.
        Validation is kept explicit because :class:`PlcBridgeConfig` adds
        profile-specific endpoint rules after constructing this object.
        """
        if not isinstance(values, Mapping):
            raise ValueError("MC connection config must be a mapping")
        known = set(cls.__dataclass_fields__)                  # noqa: SLF001
        unknown = set(values) - known
        if strict and unknown:
            names = ", ".join(sorted(str(name) for name in unknown))
            raise ValueError(f"unknown MC connection config field(s): {names}")
        return cls(**{key: value for key, value in values.items() if key in known})

    def validate(self) -> None:
        """Reject values that cannot form a bounded IPv4 MC session."""
        if not isinstance(self.host, str) or not self.host.strip():
            raise ValueError("host must be a non-empty hostname or IPv4 address")
        self.host = self.host.strip()
        if "://" in self.host:
            raise ValueError("host must not include an URL scheme")

        if isinstance(self.port, bool) or not isinstance(self.port, int):
            raise ValueError("port must be an integer")
        if not 1 <= self.port <= 65535:
            raise ValueError("port must be in the range 1..65535")

        source = self.source_address
        if source is not None and not isinstance(source, str):
            raise ValueError(
                "source_address must be a numeric IPv4 address or empty"
            )
        source_text = (source or "").strip()
        if source_text:
            try:
                self.source_address = str(ipaddress.IPv4Address(source_text))
            except ipaddress.AddressValueError as exc:
                raise ValueError(
                    "source_address must be a numeric IPv4 address or empty"
                ) from exc
        else:
            self.source_address = None

        if not isinstance(self.frame, str):
            raise ValueError("frame must be '3E' or '4E'")
        self.frame = self.frame.strip().upper()
        if self.frame not in {"3E", "4E"}:
            raise ValueError("frame must be '3E' or '4E'")

        if not isinstance(self.protocol, str):
            raise ValueError("protocol must be 'binary' or 'ascii'")
        self.protocol = self.protocol.strip().lower()
        if self.protocol not in {"binary", "ascii"}:
            raise ValueError("protocol must be 'binary' or 'ascii'")

        integer_limits = {
            "network_no": 0xFF,
            "pc_no": 0xFF,
            "io_no": 0xFFFF,
            "station_no": 0xFF,
            "monitor_timer_250ms": 0xFFFF,
        }
        for name, maximum in integer_limits.items():
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or not 0 <= value <= maximum
            ):
                raise ValueError(f"{name} must be an integer in 0..{maximum}")

        for name in ("connect_timeout_s", "read_timeout_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a positive finite number")
            normalized = float(value)
            if not math.isfinite(normalized) or normalized <= 0.0:
                raise ValueError(f"{name} must be a positive finite number")
            setattr(self, name, normalized)

        backoff = self.reconnect_backoff_s
        if isinstance(backoff, (str, bytes)) or not isinstance(backoff, Sequence):
            raise ValueError("reconnect_backoff_s must be a sequence")
        normalized_backoff = []
        for value in backoff:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(
                    "reconnect_backoff_s values must be positive finite numbers"
                )
            delay = float(value)
            if not math.isfinite(delay) or delay <= 0.0:
                raise ValueError(
                    "reconnect_backoff_s values must be positive finite numbers"
                )
            normalized_backoff.append(delay)
        # Preserve the old empty-list fallback while preventing a zero-delay
        # reconnect loop.
        self.reconnect_backoff_s = normalized_backoff or [1.0]


class McClient:
    """단일 PLC 세션. 스레드 안전하지 않으므로 노드당 1개씩 사용한다."""

    def __init__(self, cfg: McConfig, logger=None):
        cfg.validate()
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
        try:
            s.settimeout(self.cfg.connect_timeout_s)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            if self.cfg.source_address is not None:
                s.bind((self.cfg.source_address, 0))
            s.connect((self.cfg.host, self.cfg.port))
            s.settimeout(self.cfg.read_timeout_s)
        except BaseException:
            # A failed bind/connect must not leave an untracked descriptor or
            # make ``connected`` report a half-open session.
            s.close()
            raise
        self._sock = s
        self._fail_count = 0
        self._info(
            f"PLC 연결 : {self.cfg.host}:{self.cfg.port} "
            f"({self.cfg.frame} {self.cfg.protocol})"
        )

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def backoff_delay(self) -> float:
        b = list(self.cfg.reconnect_backoff_s) or [1.0]
        # note_failure() increments before callers ask for the delay.  Convert
        # that 1-based failure count to a 0-based schedule index so the first
        # failure uses the configured first cooldown rather than skipping it.
        index = max(0, self._fail_count - 1)
        return b[min(index, len(b) - 1)]

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

    def write_random_words(
        self,
        writes: Mapping[str, int] | Sequence[tuple[str, int]],
    ) -> None:
        """Write non-contiguous word devices in one MC request.

        MC command ``0x1402/0x0000`` encodes one byte each for the word and
        double-word point counts, followed by ``device + WORD`` records.  This
        API deliberately exposes only the word half of that command because
        the robot-control contract consists entirely of individual D words.

        The order supplied by the caller is retained.  Duplicate physical
        devices are rejected even when their spellings differ (for example,
        ``d1018`` and ``D1018``), avoiding an ambiguous last-write-wins command.
        """
        if self.is_ascii:
            raise ValueError(
                "random word write (0x1402) is supported only for MC binary protocol"
            )

        if isinstance(writes, Mapping):
            requested = list(writes.items())
        elif isinstance(writes, Sequence) and not isinstance(
            writes,
            (str, bytes, bytearray),
        ):
            requested = list(writes)
        else:
            raise TypeError("writes must be a mapping or a sequence of (device, value) pairs")

        count = len(requested)
        if not 1 <= count <= 0xFF:
            raise ValueError("random word write requires 1..255 device/value pairs")

        normalized: list[tuple[int, int, int]] = []
        seen: set[tuple[int, int]] = set()
        for index, item in enumerate(requested):
            if not isinstance(item, (tuple, list)) or len(item) != 2:
                raise ValueError(
                    f"random word write item {index} must be a (device, value) pair"
                )
            device, value = item
            if not isinstance(device, str):
                raise TypeError(f"random word write device {index} must be a string")
            code, addr = parse_device(device)
            if not 0 <= addr <= 0xFFFFFF:
                raise ValueError(f"device address is outside the MC 24-bit range: {device!r}")
            key = (code, addr)
            if key in seen:
                raise ValueError(f"duplicate random word write device: {device!r}")
            seen.add(key)
            try:
                word = int(value) & 0xFFFF
            except (TypeError, ValueError) as exc:
                raise TypeError(
                    f"random word write value {index} must be an integer"
                ) from exc
            normalized.append((code, addr, word))

        # Binary random-write payload:
        #   word points (U8), dword points (U8=0), then N records of
        #   address (U24 LE), device code (U8), value (U16 LE).
        payload = bytearray((count, 0))
        for code, addr, word in normalized:
            payload.extend(struct.pack("<I", addr)[:3])
            payload.append(code)
            payload.extend(struct.pack("<H", word))

        self._transact(
            CMD_RANDOM_WRITE,
            SUB_WORD,
            bytes(payload),
            context=f"random write × {count}",
        )

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
