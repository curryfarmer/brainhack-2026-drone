"""Preflight: the ordered P0-P10 gate before any flight + bench scripts B1-B8.

Planned surface (S10):
- run_preflight(profile, agents, cfg) -> list[CheckResult]; live table printed,
  runs/<ts>/preflight.json persisted, FIRST CRITICAL FAILURE ABORTS (clean
  disconnect, exit 3). Runnable standalone via --preflight-only — this doubles
  as the primary bench tool.
- Checks, in order: P0 config sanity (incl. non-placeholder arena params) ->
  P1 log dir writable+fsync probe -> P2 detector load + smoke inference +
  class-map intersection -> P3 Dola discovery finds EXACTLY the expected
  plane_ids -> P4 per-drone connect -> P5 telemetry sane (battery >= floor,
  heartbeat < 2 s, altitude ~ 0) -> P6 video fresh (first frame < 10 s,
  fps >= floor, timestamps advancing) -> P7 detector on live frames +
  projected tick load (the laptop-overload gate, caught on the ground) ->
  P8 UWB serial (only if use_uwb) -> P9 safety systems (battery failsafe
  ack'd, identity LED set per drone) -> P10 operator types literal GO within
  60 s (default-deny — fixes mapping_drone.py:318-327 where invalid input
  fell through to arming and sys was never imported).
- Bench scripts B1-B8 (real drones, props off, zero flight risk): discovery,
  telemetry, 3-streams+detector load, real-frame capture, command-semantics
  probe (incl. the cm-vs-m "unit hop"), link-loss drill, get_position()
  characterization, abort-channel rehearsal. Each writes a JSON result file.

Derives from: mapping_drone.py confirm-prompt idea (audited),
docs/quali/deployment.md pre-run checklist, dola.py discovery.

STUB — session S10.
"""
from __future__ import annotations

_STUB = "finals.preflight: session S10 — see finals/docs/module_map.md"


class CheckResult:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)


async def run_preflight(*args, **kwargs):
    raise NotImplementedError(_STUB)
