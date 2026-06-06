"""Drone discovery — audited adaptation of the official dola.py listener.

Planned surface (S9):
- Same protocol as docs/finals/example_code/dola.py (verified): UDP port 8668
  (the docstring there says 8688 — the CODE says 8668, trust the code),
  44-byte MAVLink-framed broadcast (STX 0xFE, MSG_ID 232), payload carrying
  plane_id / ip / serial / wifi flags.
- _parse_packet(packet: bytes, sender_ip: str) -> dict | None stays a PURE
  function -> unit-tested with crafted 44-byte packets, no sockets.
- discover_required(plane_ids: list[int], timeout_s: float) -> dict[int, str]
  raises PreflightError("planes not found: [2, 3] — found [1]; check
  Wi-Fi/SSID/power, UDP 8668 firewall inbound") instead of returning Nones.
- Audit fixes over the example: parse errors logged with traceback + counter
  (not print-and-continue), stop() closes the socket BEFORE join and never
  swallows the close error silently, listener loop bound by a stop event.

Derives from: docs/finals/example_code/dola.py (near-copy, audited).

STUB — session S9.
"""
from __future__ import annotations

_STUB = "finals.flight.discovery: session S9 — see finals/docs/module_map.md"


def discover_required(*args, **kwargs):
    raise NotImplementedError(_STUB)
