"""MavsdkSitlAdapter — the qualifier PX4 SITL VM backend.

Implements the SAME relative-move semantics as pyhulax on top of MAVSDK NED
setpoints, so the VM exercises the exact mission logic the real drones run.

Planned surface (S6):
- connect(): wraps root drone_control.Drone.connect() (incl. its
  _kill_stale_servers cleanup), then starts the get_position_with_task.py
  SharedState/position_monitor_task pattern internally; asyncio.wait_for
  everywhere.
- takeoff(): set_takeoff_altitude(height_cm/100) -> arm -> takeoff -> POLL
  telemetry until -down_m >= 0.9*target (replaces drone_control.py's blind
  20 s sleep) -> 20x zero-velocity setpoints -> offboard.start()
  (mapping_drone.py:335-341 sequence, audited).
- move(): read (N, E, D, yaw) from SharedState; rotate the body offset into
  NED (_body_offset_to_ned — pure function, unit-tested without SITL); then
  the PROVEN _go_to_waypoint pattern from qualifier_run.py:268-331 (stream
  position setpoints @10 Hz, poll until within 0.15 m) WITH the hard deadline
  the example lacks -> FlightTimeout naming the waypoint.
- rotate(): drone_control.Drone.rotate_to_yaw (proven PID) + deadline.
- land(): offboard.stop -> action.land -> poll in_air == False
  (mapping_drone.py:357-373, audited) -> disarm; timeout raises.
- emergency_land(): offboard.stop / land / disarm each in its own
  try/except-log (best effort).

Derives from: drone_control.py (root, PROVEN — imported, never modified),
get_position_with_task.py (root, proven), qualifier_run.py:268-331 (proven),
mapping_drone.py takeoff/land sequence (audited — its four verified bugs are
fixed by construction here: the stale-state wait loop, the two missing awaits,
sys.exit without import, battery read-before-assign).

STUB — session S6. VM gate V1: --profile sitl --phases takeoff_demo flies
takeoff -> square -> land; killing PX4 mid-move raises FlightTimeout with an
actionable message instead of hanging.
"""
from __future__ import annotations

_STUB = "finals.flight.sitl_adapter: session S6 — see finals/docs/module_map.md"


class MavsdkSitlAdapter:  # implements finals.flight.adapter.FlightAdapter in S6
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)
