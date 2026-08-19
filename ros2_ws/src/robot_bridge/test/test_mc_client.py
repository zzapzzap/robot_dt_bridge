"""Unit tests for the MC transport configuration and socket lifecycle."""

from __future__ import annotations

import struct

import pytest

from robot_bridge import mc_client
from robot_bridge.mc_client import McClient, McConfig


class _FakeSocket:
    def __init__(self, *, fail_connect: bool = False) -> None:
        self.fail_connect = fail_connect
        self.events = []
        self.closed = False

    def settimeout(self, value) -> None:
        self.events.append(("timeout", value))

    def setsockopt(self, level, option, value) -> None:
        self.events.append(("setsockopt", level, option, value))

    def bind(self, address) -> None:
        self.events.append(("bind", address))

    def connect(self, address) -> None:
        self.events.append(("connect", address))
        if self.fail_connect:
            raise OSError("test connect failure")

    def close(self) -> None:
        self.events.append(("close",))
        self.closed = True


def test_from_dict_keeps_legacy_filtering_and_supports_strict_mode() -> None:
    config = McConfig.from_dict({"host": " plc.local ", "unrelated": 1})
    config.validate()
    assert config.host == "plc.local"

    with pytest.raises(ValueError, match="unrelated"):
        McConfig.from_dict(
            {"host": "plc.local", "unrelated": 1}, strict=True
        )


@pytest.mark.parametrize("value", [None, "", "   "])
def test_empty_source_address_disables_explicit_binding(value) -> None:
    config = McConfig(source_address=value)
    config.validate()
    assert config.source_address is None


@pytest.mark.parametrize(
    "value",
    ["plc-nic.local", "2001:db8::1", "999.1.1.1", 1234],
)
def test_invalid_source_address_is_rejected(value) -> None:
    with pytest.raises(ValueError, match="source_address"):
        McConfig(source_address=value).validate()


def test_connect_binds_plc_facing_source_before_destination(monkeypatch) -> None:
    fake = _FakeSocket()
    monkeypatch.setattr(mc_client.socket, "socket", lambda *_: fake)
    client = McClient(
        McConfig(
            host="192.168.10.30",
            port=9000,
            source_address="192.168.10.61",
        )
    )

    client.connect()

    bind_index = fake.events.index(("bind", ("192.168.10.61", 0)))
    connect_index = fake.events.index(("connect", ("192.168.10.30", 9000)))
    assert bind_index < connect_index
    assert client.connected


def test_failed_connect_closes_untracked_socket(monkeypatch) -> None:
    fake = _FakeSocket(fail_connect=True)
    monkeypatch.setattr(mc_client.socket, "socket", lambda *_: fake)
    client = McClient(McConfig(host="192.168.10.30", port=9000))

    with pytest.raises(OSError, match="test connect failure"):
        client.connect()

    assert fake.closed
    assert not client.connected


def test_first_failure_uses_first_reconnect_cooldown() -> None:
    client = McClient(McConfig(reconnect_backoff_s=[0.5, 1.0, 2.0]))

    client.note_failure()
    assert client.backoff_delay() == pytest.approx(0.5)
    client.note_failure()
    assert client.backoff_delay() == pytest.approx(1.0)
    client.note_failure()
    client.note_failure()
    assert client.backoff_delay() == pytest.approx(2.0)


def test_random_word_write_builds_ordered_binary_1402_payload(monkeypatch) -> None:
    client = McClient(McConfig(protocol="binary"))
    captured = {}

    def capture(command, subcommand, payload, context=""):
        captured.update(
            command=command,
            subcommand=subcommand,
            payload=payload,
            context=context,
        )
        return b""

    monkeypatch.setattr(client, "_transact", capture)

    client.write_random_words(
        [
            ("D1016", 25),
            ("D1018", 50),
            ("D1020", 75),
            ("D1100", 0x0024),
        ]
    )

    expected = bytearray((4, 0))
    for address, value in (
        (1016, 25),
        (1018, 50),
        (1020, 75),
        (1100, 0x0024),
    ):
        expected.extend(struct.pack("<I", address)[:3])
        expected.append(mc_client.DEVICE_CODES["D"])
        expected.extend(struct.pack("<H", value))

    assert captured == {
        "command": mc_client.CMD_RANDOM_WRITE,
        "subcommand": mc_client.SUB_WORD,
        "payload": bytes(expected),
        "context": "random write × 4",
    }


def test_random_word_write_mapping_keeps_insertion_order(monkeypatch) -> None:
    client = McClient(McConfig(protocol="binary"))
    payloads = []
    monkeypatch.setattr(
        client,
        "_transact",
        lambda _command, _subcommand, payload, context="": payloads.append(payload),
    )

    client.write_random_words({"D1020": 3, "D1016": 1, "D1018": 2})

    addresses = [
        int.from_bytes(payloads[0][offset:offset + 3], "little")
        for offset in range(2, len(payloads[0]), 6)
    ]
    assert addresses == [1020, 1016, 1018]


def test_random_word_write_rejects_duplicate_normalized_devices() -> None:
    client = McClient(McConfig(protocol="binary"))

    with pytest.raises(ValueError, match="duplicate"):
        client.write_random_words([("d1018", 1), ("D1018", 0)])


def test_random_word_write_rejects_ascii_before_transport(monkeypatch) -> None:
    client = McClient(McConfig(protocol="ascii"))
    called = False

    def forbidden(*_args, **_kwargs):
        nonlocal called
        called = True

    monkeypatch.setattr(client, "_transact", forbidden)

    with pytest.raises(ValueError, match="binary"):
        client.write_random_words({"D1018": 1})
    assert called is False


@pytest.mark.parametrize("writes", [[], [("D0", 0)] * 256])
def test_random_word_write_enforces_binary_point_count(writes) -> None:
    client = McClient(McConfig(protocol="binary"))

    with pytest.raises(ValueError, match="1..255"):
        client.write_random_words(writes)
