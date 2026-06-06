"""PyhulaxAdapter — the real-drone backend (HULA over Wi-Fi) + FakeDroneAPI.

Planned surface (S9):
- Per-drone single-thread executor; EVERY blocking SDK call runs as
  await asyncio.wait_for(loop.run_in_executor(pool, fn), timeout_s) — the
  adapter is the single choke point where every command gets a hard deadline,
  because Wi-Fi-dropout-mid-blocking-call behavior is UNDOCUMENTED (pyhulax
  research open question). On TimeoutError the executor thread may still
  complete the move: the adapter marks itself degraded, raises FlightTimeout,
  and the agent safes the drone down.
- connect(ip): DroneAPI().connect under wait_for; one retry via
  robust_connect on DroneConnectionError; then enable_battery_failsafe()
  ALWAYS; then a 2 Hz telemetry poller thread (get_battery / get_altitude /
  get_orientation / get_state -> latest Telemetry; getters return immediately
  per docs).
- move/rotate/takeoff/land/hover: the documented blocking SDK calls, with
  pyhulax exceptions (CommandTimeout, CommandRejected, NotReady, LowBattery,
  DroneConnectionError) re-raised as typed finals.errors with drone_id +
  action in the message.
- move_to(x, y, z) is deliberately NOT exposed: it is not closed-loop
  (verified research) and would lie to mission logic.
- set_led(r, g, b): identity colour, used by preflight P9.
- FakeDroneAPI: module-level test double mirroring the pyhulax surface, with
  the constructor taking api_factory=DroneAPI injectable — this adapter's
  logic is unit-testable on machines WITHOUT pyhulax installed.

Units note: the contract says distance_cm (pyhulax docs), but
hula_connection.py:45 shows move(FORWARD, 0.5) — contradiction. The onsite
"unit hop" preflight gate settles it; a fix touches only this boundary.

Derives from: hula_connection.py:29-37 (connect/video sequence, audited) +
pyhulax reference docs https://pyhulax.xenops.ae/reference/pyhulax/.

STUB — session S9 (mock-first; hardware validation deferred to the onsite
window — the mapping_drone.py bug list is the audit bar).
"""
from __future__ import annotations

_STUB = "finals.flight.pyhulax_adapter: session S9 — see finals/docs/module_map.md"


class PyhulaxAdapter:  # implements finals.flight.adapter.FlightAdapter in S9
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)


class FakeDroneAPI:  # pyhulax-surface test double, S9
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)
