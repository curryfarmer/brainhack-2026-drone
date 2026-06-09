"""finals.flight.discovery — the pure parser + the bounded scan, no real UDP.

discovery.py is stdlib-only (no pyhulax), so everything here runs on the bare
dev venv. _parse_packet is exercised against crafted 44-byte frames; the scan
runs against an injected fake socket (the `sock=` seam) so the found / missing
/ deadline logic is deterministic with no firewall, no port bind, no network.
"""
from __future__ import annotations

import socket
import time

import pytest

from finals.errors import PreflightError
from finals.flight.discovery import (DISCOVERY_PORT, _parse_packet,
                                     discover_required)


# ============================================================
# Packet crafting (the dola.py byte layout)
# ============================================================
def make_packet(plane_id: int, ip: str, *, stx: int = 0xFE, msg_id: int = 232,
                serial: bytes = b"\xab" * 16, wifi_mode: int = 1,
                bind_client: int = 1, wifi_power: int = 20) -> bytes:
    ip_field = ip.encode("ascii")[:16].ljust(16, b"\x00")
    serial = serial[:16].ljust(16, b"\x00")
    pkt = bytearray(44)
    pkt[0] = stx
    pkt[1:5] = b"\x00\x10\x01\x01"        # len/seq/sysid/compid (unread)
    pkt[5] = msg_id
    pkt[6:22] = serial
    pkt[22:38] = ip_field
    pkt[38] = plane_id
    pkt[39] = wifi_mode
    pkt[40] = bind_client
    pkt[41] = wifi_power
    pkt[42:44] = b"\x00\x00"              # checksum (unread)
    return bytes(pkt)


def test_parse_good_packet():
    info = _parse_packet(make_packet(7, "192.168.1.42"), "192.168.1.42")
    assert info is not None
    assert info["plane_id"] == 7
    assert info["ip"] == "192.168.1.42"
    assert info["sn"] == ("ab" * 16)
    assert info["wifi_mode"] == 1
    assert info["bind_client"] == 1
    assert info["wifi_power"] == 20
    assert info["sender_ip"] == "192.168.1.42"


def test_parse_rejects_wrong_stx():
    assert _parse_packet(make_packet(1, "10.0.0.1", stx=0x00), "10.0.0.1") is None


def test_parse_rejects_wrong_msg_id():
    assert _parse_packet(make_packet(1, "10.0.0.1", msg_id=99), "10.0.0.1") is None


@pytest.mark.parametrize("length", [0, 10, 43, 45, 88])
def test_parse_rejects_wrong_length(length):
    assert _parse_packet(b"\xfe" * length, "10.0.0.1") is None


def test_parse_ip_is_nul_trimmed_and_ascii_tolerant():
    # Non-ascii bytes in the IP field are dropped (errors="ignore"); NULs and
    # whitespace are stripped — never crashes the pure parser.
    pkt = bytearray(make_packet(3, "10.0.0.9"))
    pkt[22:38] = b"10.0.0.9\xff\xfe\x00\x00\x00\x00\x00\x00"
    info = _parse_packet(bytes(pkt), "10.0.0.9")
    assert info is not None
    assert info["ip"] == "10.0.0.9"


def test_port_constant_is_the_code_value_not_the_docstring():
    # dola.py's docstring says 8688; its CODE says 8668 — trust the code.
    assert DISCOVERY_PORT == 8668


# ============================================================
# The bounded scan via an injected fake socket
# ============================================================
class FakeSocket:
    """Yields scripted (packet, (ip, port)) tuples, then raises socket.timeout
    forever — the recv-deadline path with no real UDP."""

    def __init__(self, packets):
        self._queue = list(packets)
        self._timeout = None
        self.closed = False

    def settimeout(self, t):
        self._timeout = t

    def recvfrom(self, bufsize):
        if self._queue:
            return self._queue.pop(0)
        # Honor the deadline pacing (don't busy-spin the test).
        time.sleep(min(0.01, self._timeout or 0.01))
        raise socket.timeout()

    def close(self):
        self.closed = True


def test_discover_returns_when_all_found():
    sock = FakeSocket([
        (make_packet(1, "192.168.1.11"), ("192.168.1.11", 8668)),
        (make_packet(2, "192.168.1.12"), ("192.168.1.12", 8668)),
    ])
    got = discover_required([1, 2], timeout_s=2.0, sock=sock)
    assert got == {1: "192.168.1.11", 2: "192.168.1.12"}
    # Injected socket is NOT owned -> not closed by discover_required.
    assert sock.closed is False


def test_discover_skips_malformed_then_finds():
    sock = FakeSocket([
        (make_packet(1, "10.0.0.5", stx=0x00), ("10.0.0.5", 8668)),  # -> None
        (make_packet(1, "10.0.0.5"), ("10.0.0.5", 8668)),            # good
    ])
    assert discover_required([1], timeout_s=2.0, sock=sock) == {1: "10.0.0.5"}


def test_discover_missing_raises_preflight_naming_the_gap():
    sock = FakeSocket([(make_packet(1, "10.0.0.5"), ("10.0.0.5", 8668))])
    with pytest.raises(PreflightError) as ei:
        discover_required([1, 2, 3], timeout_s=0.2, sock=sock)
    msg = str(ei.value)
    assert "planes not found" in msg
    assert "[2, 3]" in msg          # exactly the missing ids
    assert "[1]" in msg             # and the ones it did hear


def test_discover_returns_only_requested_planes():
    sock = FakeSocket([
        (make_packet(1, "10.0.0.1"), ("10.0.0.1", 8668)),
        (make_packet(9, "10.0.0.9"), ("10.0.0.9", 8668)),   # unrequested
        (make_packet(2, "10.0.0.2"), ("10.0.0.2", 8668)),
    ])
    assert discover_required([1, 2], timeout_s=2.0, sock=sock) == {
        1: "10.0.0.1", 2: "10.0.0.2"}


@pytest.mark.parametrize("plane_ids, timeout_s, match", [
    ([], 1.0, "empty"),
    ([1, "2"], 1.0, "ints"),
    ([True, 2], 1.0, "ints"),       # bool is not a valid plane id
    ([1], 0.0, "timeout_s"),
    ([1], -1.0, "timeout_s"),
])
def test_discover_validates_inputs(plane_ids, timeout_s, match):
    with pytest.raises(ValueError, match=match):
        discover_required(plane_ids, timeout_s, sock=FakeSocket([]))
