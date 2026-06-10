"""Drone discovery — audited adaptation of the official dola.py listener.

Resolves the configured plane_ids to their (DHCP-assigned) Wi-Fi IPs by
listening for the HULA aircraft's MAVLink-framed UDP broadcast, so the
preflight gate (S10) can hand each PyhulaxAdapter a concrete IP to connect to.

Protocol (verified against docs/finals/example_code/dola.py — the CODE, not its
docstring): the aircraft broadcasts a 44-byte MAVLink v1 frame (STX 0xFE,
MSG_ID 232) on UDP port 8668 carrying plane_id / ip / serial / wifi flags. The
dola docstring says port 8688 — that is WRONG; `Dola.UDP_PORT = 8668` is the
real value, trust the code.

This is a near-copy, audited line-by-line (convention 7). dola.py's bugs, fixed
here:
- dola `_listener_loop` swallows EVERY parse exception with a bare
  `print("Parse error:", e)` and continues — a malformed-packet storm scrolls
  off and is uncountable. Here parse errors are caught TYPED, logged WITH a
  traceback, and counted (a running counter + an end-of-scan summary).
- dola `stop()` wraps `self.sock.close()` in a bare-except/pass (swallowed
  silently) and only THEN joins the thread. This design removes the
  background listener thread ENTIRELY — `discover_required` is a synchronous,
  wall-clock-bounded scan — so there is no stop()/join ordering bug to have; the
  socket is closed in a `finally` and a close error is logged, never swallowed
  bare.
- dola's `while self._running` listener loop has no deadline (it runs until an
  external `stop()`); here the recv loop is bound by a `time.monotonic()`
  deadline (convention 3) and returns the instant every requested plane is seen.
- dola's `get_ips_by_plane_ids` returns `{id: None}` for planes it never heard
  from — a silent "fly anyway" hole. Here missing planes RAISE PreflightError
  naming exactly which ids are missing and what to check.

PURE stdlib (socket / struct / threading-free): imports NO SDK, so this module
stays in the conventions scan (tests/test_conventions.py SDK_ALLOWED does NOT
list it) and never depends on pyhulax being installed.

Session: S9.
"""
from __future__ import annotations

import logging
import socket
import time
from typing import Dict, Iterable, List, Optional

from finals.errors import PreflightError

_LOG = logging.getLogger(__name__)

#: UDP port the aircraft broadcast lands on. dola.py's docstring says 8688; its
#: CODE (`Dola.UDP_PORT`) says 8668 — the code is authoritative.
DISCOVERY_PORT = 8668

_MAVLINK_STX = 0xFE
_MSG_ID = 232
_PACKET_LEN = 44


def _parse_packet(packet: bytes, sender_ip: str) -> Optional[dict]:
    """Decode one broadcast frame -> info dict, or None if it is not a valid
    HULA discovery packet. PURE: no sockets, no clock — unit-tested against
    crafted 44-byte byte strings.

    Field layout (dola.py `_parse_packet`, byte-for-byte):
        [0]      MAVLink STX (0xFE)
        [5]      message id (232)
        [6:22]   serial number (16 bytes, hex-encoded)
        [22:38]  IP address (16 bytes ASCII, NUL-padded)
        [38]     plane_id
        [39]     wifi_mode
        [40]     bind_client
        [41]     wifi_power
        [42:44]  checksum (unused here)

    Unlike dola, this carries NO `time.time()` last_seen stamp — a pure parser
    must not read the clock; freshness is the caller's concern.
    """
    if len(packet) != _PACKET_LEN:
        return None
    if packet[0] != _MAVLINK_STX:
        return None
    if packet[5] != _MSG_ID:
        return None

    serial_number = packet[6:22].hex()
    ip_address = (packet[22:38]
                  .decode("ascii", errors="ignore")
                  .rstrip("\x00")
                  .strip())
    return {
        "plane_id": packet[38],
        "ip": ip_address,
        "sn": serial_number,
        "wifi_mode": packet[39],
        "bind_client": packet[40],
        "wifi_power": packet[41],
        "sender_ip": sender_ip,
    }


def discover_required(plane_ids: Iterable[int], timeout_s: float, *,
                      port: int = DISCOVERY_PORT,
                      listen_ip: str = "0.0.0.0",
                      sock: Optional[socket.socket] = None,
                      min_count: Optional[int] = None) -> Dict[int, str]:
    """Listen up to timeout_s for the broadcast and return {plane_id: ip}.

    STRICT (min_count is None, the default): require EVERY requested plane —
    raise PreflightError naming the ones never heard. Returns as soon as all are
    seen (no need to burn the full timeout).

    DEGRADED (min_count set — the allow_partial_fleet path): require only
    `min_count` of the requested planes. Listens the FULL window to collect as
    many as answer, then returns WHATEVER subset was heard (>= min_count),
    raising only if fewer than min_count appeared. The caller (preflight P3)
    drops the un-found drones and flies the survivors.

    Bounded by a wall-clock deadline (convention 3). The recv socket is created
    here unless `sock` is injected (the test seam — a fake socket feeds crafted
    packets so the found/missing logic is exercised with NO real UDP, no
    firewall, no port binding).
    """
    wanted: List[int] = list(plane_ids)
    if not wanted:
        raise ValueError(
            "discover_required: plane_ids is empty — nothing to discover "
            "(check the config's per-drone plane_id values)")
    if not all(isinstance(p, int) and not isinstance(p, bool) for p in wanted):
        raise ValueError(
            f"discover_required: plane_ids must be ints, got {wanted!r}")
    if not (isinstance(timeout_s, (int, float)) and timeout_s > 0):
        raise ValueError(
            f"discover_required: timeout_s must be > 0, got {timeout_s!r}")
    if min_count is not None and not (
            isinstance(min_count, int) and not isinstance(min_count, bool)
            and 1 <= min_count <= len(wanted)):
        raise ValueError(
            f"discover_required: min_count must be an int in [1, {len(wanted)}] "
            f"or None (strict), got {min_count!r}")

    wanted_set = set(wanted)
    found: Dict[int, str] = {}
    parse_errors = 0

    owns_sock = sock is None
    if owns_sock:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((listen_ip, port))

    deadline = time.monotonic() + timeout_s
    try:
        # Bounded (convention 3): the deadline ends the loop; an all-found set
        # ends it early. Per-recv timeout = the remaining budget (capped at
        # 0.5 s so a fresh plane appearing is noticed promptly).
        while wanted_set - found.keys():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            sock.settimeout(min(0.5, remaining))
            try:
                packet, addr = sock.recvfrom(1024)
            except socket.timeout:
                # No packet this slice — normal; loop re-checks the deadline.
                continue
            except OSError as e:
                # A real socket error (closed underfoot, network down): stop
                # listening and let the missing-plane check below report it.
                _LOG.warning("discovery: recv failed on UDP %d (%s: %s) — "
                             "stopping scan", port, type(e).__name__, e)
                break

            try:
                info = _parse_packet(packet, addr[0])
            except (ValueError, IndexError, UnicodeError):
                # dola printed-and-continued; we log WITH a traceback + count.
                parse_errors += 1
                _LOG.warning("discovery: malformed packet #%d from %s "
                             "(ignored)", parse_errors, addr[0], exc_info=True)
                continue
            if info is None:
                continue
            found[info["plane_id"]] = info["ip"]
    finally:
        if owns_sock:
            try:
                sock.close()
            except OSError as e:
                # Never swallow the close bare (dola did): log it.
                _LOG.warning("discovery: socket close failed (%s: %s)",
                             type(e).__name__, e)

    if parse_errors:
        _LOG.warning("discovery: %d malformed packet(s) ignored during the "
                     "scan", parse_errors)

    missing = sorted(wanted_set - found.keys())
    if min_count is None:
        if missing:
            found_ids = sorted(found)
            raise PreflightError(
                f"planes not found: {missing} — found {found_ids}; check "
                f"Wi-Fi/SSID/power, that the aircraft are bound to this client, "
                f"and UDP {port} firewall inbound (laptop on the drone network?)")
        return {pid: found[pid] for pid in wanted}

    # DEGRADED: only min_count required. Return the found subset (in request
    # order) if it clears the floor; otherwise a hard abort — too few to fly.
    if len(found) < min_count:
        raise PreflightError(
            f"degraded-fleet discovery: only {len(found)} plane(s) found "
            f"{sorted(found)}, need >= {min_count} (missing {missing}); check "
            f"Wi-Fi/SSID/power, aircraft bound to this client, and UDP {port} "
            f"firewall inbound (laptop on the drone network?)")
    return {pid: found[pid] for pid in wanted if pid in found}
