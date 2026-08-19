"""Functional MELSEC PLC simulator for the Robot -> PLC -> Jetson path.

The server implements the MC protocol 3E batch word read/write commands in
binary and ASCII form, plus binary random word write (0x1402).  It exposes the
fixed sample register map supplied for the Hyundai robot integration::

    D1000       operation state
    D1002..1013 six signed 32-bit joint angles (0.01 degree/LSB)
    D1014       run feedback
    D1016       25 percent direct speed-control word
    D1018       50 percent direct speed-control word
    D1020       75 percent direct speed-control word
    D1100.0     hold feedback
    D1100.1     emergency-stop feedback (external state injection only)
    D2000..2002 start / hold / stop requests
    D3000..3002 75 / 50 / 25 percent speed requests

This is a protocol and visualisation simulator, not a safety controller.  In
particular D2002 is a normal stop request; it never asserts the emergency-stop
feedback bit.

Run it without ROS with::

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
from typing import Callable, Dict, Iterable, Optional, Sequence

DEVICE_D = 0xA8
DEVICE_CODES_ASCII = {
    "D": DEVICE_D,
    "W": 0xB4,
    "R": 0xAF,
    "ZR": 0xB0,
    "M": 0x90,
    "X": 0x9C,
    "Y": 0x9D,
    "B": 0xA0,
    "L": 0x92,
}

OPERATION_STATE_D = 1000
AXIS_FIRST_D = 1002
RUN_FEEDBACK_D = 1014
SPEED_FEEDBACK_D = {25: 1016, 50: 1018, 75: 1020}
STATUS_D = 1100
COMMAND_FIRST_D = 2000
SPEED_COMMAND_FIRST_D = 3000
DIRECT_CONTROL_D = frozenset((*SPEED_FEEDBACK_D.values(), STATUS_D))

_DEFAULT_PLC_AXIS_DEG = (38.56, 136.25, -49.48, 0.17, -86.85, -50.68)
_DEGREES_PER_RAW = 0.01
_AXIS_AMPLITUDES_DEG = (60.0, 15.0, 30.0, 90.0, 35.0, 120.0)
_AXIS_FREQUENCIES_HZ = (0.10, 0.13, 0.17, 0.07, 0.23, 0.05)
_SPEED_COMMANDS = ((75, 3000), (50, 3001), (25, 3002))


class FakePlcState:
    """Thread-safe memory and robot process model owned by one server.

    Internal pose/status refreshes do not count as client writes.  ``write_count``
    therefore starts at zero and counts complete MC batch-write requests only.
    A monotonic clock can be injected to make motion tests deterministic.
    """

    def __init__(
        self,
        *,
        monotonic_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        self.lock = threading.RLock()
        self._words: Dict[tuple[int, int], int] = {}
        # D1016/D1018/D1020 and D1100 historically doubled as simulated
        # feedback.  Once an MC client writes one of these newly commissioned
        # direct-control words, preserve its exact raw value on later reads
        # instead of letting the animation refresh overwrite it.
        self._client_owned_d_words: set[int] = set()
        self._clock = monotonic_fn
        self._last_clock = float(self._clock())
        self._phase_seconds = 0.0

        # The simulator starts at the reviewed PLC axis sample with Hold
        # asserted.  The gateway applies scale/direction only when publishing
        # the visual ROS joint values; raw register signs are never pre-flipped.
        # No MC write is generated at startup.  SetHold(false) releases the
        # animation through the same direct-control service used by the PLC
        # gateway; remote Start deliberately remains subject to its own gate.
        self.running = True
        self.hold = True
        self.stopped = False
        self.emergency_stop = False
        self.speed_percent = 100

        self.write_count = 0
        self.write_log: list[dict] = []
        with self.lock:
            self._refresh_locked()

    @property
    def phase_seconds(self) -> float:
        """Current trajectory phase, after accounting for speed and pauses."""

        with self.lock:
            self._refresh_locked()
            return self._phase_seconds

    def _write_raw_locked(
        self,
        code: int,
        addr: int,
        values: Iterable[int],
    ) -> None:
        for offset, value in enumerate(values):
            self._words[(int(code), int(addr) + offset)] = int(value) & 0xFFFF

    def _read_raw_locked(self, code: int, addr: int, count: int) -> list[int]:
        return [
            self._words.get((int(code), int(addr) + offset), 0)
            for offset in range(int(count))
        ]

    def _set_dword_locked(self, addr: int, value: int) -> None:
        low, high = struct.unpack("<HH", struct.pack("<i", int(value)))
        self._write_raw_locked(DEVICE_D, addr, (low, high))

    def _motion_enabled_locked(self) -> bool:
        return (
            self.running
            and not self.hold
            and not self.stopped
            and not self.emergency_stop
        )

    def _refresh_locked(self) -> None:
        now = float(self._clock())
        elapsed = max(0.0, now - self._last_clock)
        self._last_clock = now
        if self._motion_enabled_locked():
            self._phase_seconds += elapsed * self.speed_percent / 100.0

        for axis, (center, amplitude, frequency) in enumerate(
            zip(
                _DEFAULT_PLC_AXIS_DEG,
                _AXIS_AMPLITUDES_DEG,
                _AXIS_FREQUENCIES_HZ,
            )
        ):
            degrees = center + amplitude * math.sin(
                2.0 * math.pi * frequency * self._phase_seconds
            )
            self._set_dword_locked(
                AXIS_FIRST_D + axis * 2,
                int(round(degrees / _DEGREES_PER_RAW)),
            )

        actual_run = self._motion_enabled_locked()
        self._write_raw_locked(
            DEVICE_D,
            OPERATION_STATE_D,
            (1 if actual_run else 0,),
        )
        self._write_raw_locked(
            DEVICE_D,
            RUN_FEEDBACK_D,
            (1 if actual_run else 0,),
        )
        for percent, feedback_addr in SPEED_FEEDBACK_D.items():
            if feedback_addr not in self._client_owned_d_words:
                self._write_raw_locked(
                    DEVICE_D,
                    feedback_addr,
                    (percent if self.speed_percent == percent else 0,),
                )
        current_status = self._words.get((DEVICE_D, STATUS_D), 0)
        status = (current_status & ~0x0003) | (1 if self.hold else 0) | (
            (1 if self.emergency_stop else 0) << 1
        )
        self._write_raw_locked(DEVICE_D, STATUS_D, (status,))

    def read(self, code: int, addr: int, count: int) -> list[int]:
        """Read words after refreshing simulated actual feedback."""

        with self.lock:
            self._refresh_locked()
            return self._read_raw_locked(code, addr, count)

    def write(self, code: int, addr: int, values: Sequence[int]) -> None:
        """Set memory without recording an MC client write.

        This method is retained for test/fault injection.  Network handlers use
        :meth:`apply_client_write`, which also applies command semantics.
        """

        with self.lock:
            self._refresh_locked()
            self._write_raw_locked(code, addr, values)

    def set_dword(self, addr: int, value: int) -> None:
        with self.lock:
            self._refresh_locked()
            self._set_dword_locked(addr, value)

    def animate(self) -> None:
        """Refresh pose/status once (compatibility with the old simulator)."""

        with self.lock:
            self._refresh_locked()

    def _value_from_request_locked(
        self,
        first_addr: int,
        values: Sequence[int],
        target_addr: int,
    ) -> Optional[int]:
        offset = target_addr - first_addr
        if 0 <= offset < len(values):
            return int(values[offset]) & 0xFFFF
        return None

    def _apply_motion_command_locked(
        self,
        first_addr: int,
        values: Sequence[int],
    ) -> None:
        asserted: list[int] = []
        for command_addr in range(COMMAND_FIRST_D, COMMAND_FIRST_D + 3):
            value = self._value_from_request_locked(
                first_addr,
                values,
                command_addr,
            )
            if value:
                asserted.append(command_addr)
        if not asserted:
            return

        # Restrictive requests take priority if a malformed request asserts
        # more than one command in the same batch.
        selected = max(asserted)
        self._write_raw_locked(
            DEVICE_D,
            COMMAND_FIRST_D,
            tuple(1 if addr == selected else 0 for addr in range(2000, 2003)),
        )
        if selected == 2002:  # normal stop request, never emergency stop
            self.running = False
            self.hold = False
            self.stopped = True
        elif selected == 2001:
            self.running = False
            self.hold = True
            self.stopped = False
        elif not self.emergency_stop:
            self.running = True
            self.hold = False
            self.stopped = False

    def _apply_speed_command_locked(self) -> None:
        active = [
            percent
            for percent, address in _SPEED_COMMANDS
            if self._words.get((DEVICE_D, address), 0)
        ]
        # Multiple asserted bits are invalid in the real contract.  The mock
        # fails safely by selecting the most restrictive requested speed.
        self.speed_percent = min(active) if active else 100

    def _apply_direct_controls_locked(self, addresses: Iterable[int]) -> None:
        """Apply the commissioned D1016/18/20 and D1100 mock semantics.

        These registers are command/readback memory rather than independent
        robot feedback in the field contract.  The functional simulator still
        needs them to affect its local animation so the ROS services can be
        checked visually.  D1100.1 remains external/read-only: a network write
        can never assert the simulated emergency-stop state.
        """

        changed = set(int(address) for address in addresses)
        if changed.intersection(SPEED_FEEDBACK_D.values()):
            active = [
                percent
                for percent, address in SPEED_FEEDBACK_D.items()
                if self._words.get((DEVICE_D, address), 0) == percent
            ]
            # A malformed multi-selection is resolved to the most restrictive
            # value.  All-zero words mean normal (100 percent) animation speed.
            self.speed_percent = min(active) if active else 100

        if STATUS_D in changed:
            requested = self._words.get((DEVICE_D, STATUS_D), 0)
            self.hold = bool(requested & 0x0001)

    def apply_client_write(
        self,
        code: int,
        addr: int,
        values: Sequence[int],
        *,
        protocol: str,
    ) -> None:
        """Apply one successful MC batch write and update actual feedback."""

        clean_values = tuple(int(value) & 0xFFFF for value in values)
        with self.lock:
            # Capture the exact freeze point before changing mode or speed.
            self._refresh_locked()
            self._write_raw_locked(code, addr, clean_values)
            if code == DEVICE_D:
                self._client_owned_d_words.update(
                    address
                    for address in range(addr, addr + len(clean_values))
                    if address in DIRECT_CONTROL_D
                )
            self.write_count += 1
            self.write_log.append(
                {
                    "protocol": str(protocol),
                    "device_code": int(code),
                    "address": int(addr),
                    "values": list(clean_values),
                }
            )

            if code == DEVICE_D:
                end_addr = addr + len(clean_values)
                self._apply_direct_controls_locked(range(addr, end_addr))
                if addr < 2003 and end_addr > 2000:
                    self._apply_motion_command_locked(addr, clean_values)
                if addr < 3003 and end_addr > 3000:
                    self._apply_speed_command_locked()
            self._refresh_locked()

    def apply_client_random_write(
        self,
        writes: Sequence[tuple[int, int, int]],
        *,
        protocol: str,
    ) -> None:
        """Atomically apply one MC random-word-write request.

        Each tuple is ``(device_code, address, WORD value)``.  Direct-control
        devices remain raw memory so a later read can verify the exact command
        and a D1100 read/modify/write cycle can retain unrelated status bits.
        Legacy D2000/D3000 simulator semantics remain available when those
        devices are included in a random request.
        """

        clean_writes = tuple(
            (int(code), int(addr), int(value) & 0xFFFF)
            for code, addr, value in writes
        )
        with self.lock:
            self._refresh_locked()
            for code, addr, value in clean_writes:
                self._write_raw_locked(code, addr, (value,))
                if code == DEVICE_D and addr in DIRECT_CONTROL_D:
                    self._client_owned_d_words.add(addr)

            self.write_count += 1
            self.write_log.append(
                {
                    "protocol": str(protocol),
                    "command": "random_word_write",
                    "writes": [
                        {
                            "device_code": code,
                            "address": addr,
                            "value": value,
                        }
                        for code, addr, value in clean_writes
                    ],
                }
            )

            d_addresses = {
                addr for code, addr, _value in clean_writes if code == DEVICE_D
            }
            self._apply_direct_controls_locked(d_addresses)
            if d_addresses.intersection(range(2000, 2003)):
                self._apply_motion_command_locked(
                    COMMAND_FIRST_D,
                    self._read_raw_locked(DEVICE_D, COMMAND_FIRST_D, 3),
                )
            if d_addresses.intersection(range(3000, 3003)):
                self._apply_speed_command_locked()
            self._refresh_locked()

    def set_emergency_stop(self, active: bool) -> None:
        """Inject external safety feedback; no D-register command asserts it."""

        with self.lock:
            self._refresh_locked()
            self.emergency_stop = bool(active)
            if self.emergency_stop:
                self.running = False
                self.hold = False
                self.stopped = True
            self._refresh_locked()


# Compatibility name used by older local scripts.  Unlike the old module-wide
# ``MEM`` singleton, each Memory/FakePlcState belongs to exactly one server.
Memory = FakePlcState


class FakePlcHandler(socketserver.BaseRequestHandler):
    """One MC 3E TCP connection."""

    @property
    def state(self) -> FakePlcState:
        return self.server.state  # type: ignore[attr-defined,no-any-return]

    def _log(self, message: str) -> None:
        server = self.server
        if not getattr(server, "quiet", False):
            print(message, flush=True)

    def handle(self) -> None:
        sock: socket.socket = self.request
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._log(f"[fake-plc] connected {self.client_address}")
        try:
            while True:
                first = self._recv_exact(2)
                if first is None:
                    break
                if first == b"50":
                    # Remaining 3E ASCII header: "00" + route(10) + len(4).
                    header = self._recv_exact(16)
                    if header is None or header[:2] != b"00":
                        break
                    length = int(header[-4:].decode("ascii"), 16)
                    body = self._recv_exact(length)
                    if body is None:
                        break
                    sock.sendall(self._process_ascii(body.decode("ascii")))
                    continue
                if first != b"\x50\x00":
                    break

                # Remaining 3E binary header: route(5) + length(2).
                header = self._recv_exact(7)
                if header is None:
                    break
                (length,) = struct.unpack("<H", header[5:7])
                body = self._recv_exact(length)
                if body is None:
                    break
                sock.sendall(self._process_binary(body))
        except (ConnectionError, OSError, UnicodeError, ValueError):
            pass
        finally:
            self._log(f"[fake-plc] disconnected {self.client_address}")

    def _recv_exact(self, size: int) -> Optional[bytes]:
        data = b""
        while len(data) < size:
            chunk = self.request.recv(size - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    @staticmethod
    def _reply_binary(end_code: int, data: bytes = b"") -> bytes:
        payload = struct.pack("<H", end_code) + data
        return (
            struct.pack("<H", 0x00D0)
            + struct.pack("<BBHB", 0x00, 0xFF, 0x03FF, 0x00)
            + struct.pack("<H", len(payload))
            + payload
        )

    def _process_binary(self, body: bytes) -> bytes:
        # body = timer(2), command(2), subcommand(2), request data
        if len(body) < 6:
            return self._reply_binary(0xC061)
        _timer, command, subcommand = struct.unpack("<HHH", body[:6])
        request = body[6:]

        if command == 0x1402 and subcommand == 0x0000:
            return self._process_binary_random_word_write(request)

        if len(request) < 6:
            return self._reply_binary(0xC061)

        addr = int.from_bytes(request[0:3], "little")
        code = request[3]
        (count,) = struct.unpack("<H", request[4:6])
        if count > 960:
            return self._reply_binary(0xC051)

        if command == 0x0401 and subcommand == 0x0000:
            words = self.state.read(code, addr, count)
            return self._reply_binary(
                0x0000,
                struct.pack(f"<{count}H", *words),
            )

        if command == 0x1401 and subcommand == 0x0000:
            required = 6 + count * 2
            if len(request) < required:
                return self._reply_binary(0xC058)
            values = struct.unpack(f"<{count}H", request[6:required])
            self.state.apply_client_write(
                code,
                addr,
                values,
                protocol="binary",
            )
            self._log_write(addr, values, "binary")
            return self._reply_binary(0x0000)

        return self._reply_binary(0xC059)

    def _process_binary_random_word_write(self, request: bytes) -> bytes:
        """Handle MC 1402/0000 word and double-word records.

        The bridge client currently emits word records only.  Accepting the
        protocol's double-word section as two little-endian words keeps the
        simulator framing-correct for independent protocol tests as well.
        """

        if len(request) < 2:
            return self._reply_binary(0xC061)
        word_count, dword_count = request[0], request[1]
        if word_count == 0 and dword_count == 0:
            return self._reply_binary(0xC051)
        required = 2 + word_count * 6 + dword_count * 8
        if len(request) != required:
            return self._reply_binary(0xC058)

        writes: list[tuple[int, int, int]] = []
        offset = 2
        for _ in range(word_count):
            addr = int.from_bytes(request[offset:offset + 3], "little")
            code = request[offset + 3]
            (value,) = struct.unpack("<H", request[offset + 4:offset + 6])
            writes.append((code, addr, value))
            offset += 6

        for _ in range(dword_count):
            addr = int.from_bytes(request[offset:offset + 3], "little")
            code = request[offset + 3]
            low, high = struct.unpack("<HH", request[offset + 4:offset + 8])
            writes.extend(((code, addr, low), (code, addr + 1, high)))
            offset += 8

        self.state.apply_client_random_write(writes, protocol="binary")
        self._log_random_write(writes)
        return self._reply_binary(0x0000)

    @staticmethod
    def _reply_ascii(end_code: int, data: str = "") -> bytes:
        payload = f"{end_code:04X}" + data
        return (
            "D000"
            "00FF03FF00"
            f"{len(payload):04X}"
            + payload
        ).encode("ascii")

    def _process_ascii(self, body: str) -> bytes:
        # body = timer(4), command(4), subcommand(4), request data
        if len(body) < 12:
            return self._reply_ascii(0xC061)
        command = int(body[4:8], 16)
        subcommand = int(body[8:12], 16)
        request = body[12:]

        # McClient intentionally exposes random word write only for binary.
        # Return a normal MC end code if another client tries ASCII 1402,
        # rather than mis-parsing its count fields as a batch device name.
        if command == 0x1402 and subcommand == 0x0000:
            return self._reply_ascii(0xC059)

        if len(request) < 12:
            return self._reply_ascii(0xC061)

        name = request[0:2].replace("*", "").strip()
        code = DEVICE_CODES_ASCII.get(name)
        if code is None:
            return self._reply_ascii(0xC05B)
        base = 16 if name in ("W", "X", "Y", "B") else 10
        addr = int(request[2:8], base)
        count = int(request[8:12], 16)
        if count > 960:
            return self._reply_ascii(0xC051)

        if command == 0x0401 and subcommand == 0x0000:
            words = self.state.read(code, addr, count)
            return self._reply_ascii(
                0x0000,
                "".join(f"{word:04X}" for word in words),
            )

        if command == 0x1401 and subcommand == 0x0000:
            required = 12 + count * 4
            if len(request) < required:
                return self._reply_ascii(0xC058)
            values = [
                int(request[12 + index * 4:16 + index * 4], 16)
                for index in range(count)
            ]
            self.state.apply_client_write(
                code,
                addr,
                values,
                protocol="ascii",
            )
            self._log_write(addr, values, "ascii")
            return self._reply_ascii(0x0000)

        return self._reply_ascii(0xC059)

    def _log_write(
        self,
        addr: int,
        values: Sequence[int],
        protocol: str,
    ) -> None:
        label = {
            COMMAND_FIRST_D: "motion-command",
            SPEED_COMMAND_FIRST_D: "speed-command",
        }.get(addr, f"D{addr}")
        self._log(
            f"[fake-plc] write({protocol}) {label} <- {list(values)}"
        )

    def _log_random_write(
        self,
        writes: Sequence[tuple[int, int, int]],
    ) -> None:
        rendered = ", ".join(
            f"0x{code:02X}:{addr}={value}"
            for code, addr, value in writes
        )
        self._log(f"[fake-plc] random-write(binary) {rendered}")


class FakePlcServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        state: FakePlcState,
        *,
        quiet: bool = False,
    ) -> None:
        self.state = state
        self.quiet = bool(quiet)
        super().__init__(server_address, FakePlcHandler)


# Older code imported these names; retain aliases without restoring global state.
Handler = FakePlcHandler
Server = FakePlcServer


def make_server(
    host: str = "127.0.0.1",
    port: int = 5010,
    *,
    state: Optional[FakePlcState] = None,
    quiet: bool = False,
) -> FakePlcServer:
    """Bind and return one independent fake PLC server.

    Binding happens synchronously, so an occupied/invalid address raises
    ``OSError`` in the caller instead of leaving a seemingly healthy process.
    """

    return FakePlcServer(
        (str(host), int(port)),
        state or FakePlcState(),
        quiet=quiet,
    )


def serve(
    host: str = "127.0.0.1",
    port: int = 5010,
    *,
    state: Optional[FakePlcState] = None,
    quiet: bool = False,
) -> None:
    """Run until interrupted.  Bind failures propagate to the process."""

    with make_server(host, port, state=state, quiet=quiet) as server:
        actual_host, actual_port = server.server_address[:2]
        if not quiet:
            print(
                f"[fake-plc] MC 3E binary/ascii listening on "
                f"{actual_host}:{actual_port}",
                flush=True,
            )
        server.serve_forever(poll_interval=0.1)


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Functional MELSEC MC 3E PLC simulator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5010)
    args = parser.parse_args(argv)
    try:
        serve(args.host, args.port)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
