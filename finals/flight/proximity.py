"""ProximitySensor — the HULA 4-directional IR obstacle-avoidance seam
(SENSE-IR), plus a dependency-free SyntheticProximitySensor.

The CONFIRMED sensor on the HULA drone is a 4-directional IR obstacle-avoidance
transmitter (~30-50 cm range). It feeds finals.guards.ProximityGuard (the
advisory->LAND ladder) through the agent's `proximity_fn` injectable, exactly
the shape PerceptionLoop.last_frame_ts feeds the VideoWatchdog.

LIVE-WIRE STATUS — ONSITE GATE (binding): pyhulax exposes NO IR / proximity
getter today (finals/flight/pyhulax_adapter.py reads only battery / altitude /
orientation / state — there is no get_ir / get_proximity / get_obstacle). So
the LIVE reading cannot be wired now; the deliverable is the GUARD + a
SyntheticProximitySensor (an honest degrade-absent feed) + this documented
seam. At the onsite hardware window: confirm the pyhulax avoidance API,
implement PyhulaxProximitySensor.read() against it (the one method below), and
flip guards.proximity_enable. Until then SyntheticProximitySensor reports the
truth it can: "no obstacle reading available" -> the guard SKIPS (None), so a
mock/SITL run with proximity_enable on never false-trips.

PURE module (NOT in tests/test_conventions.py SDK_ALLOWED): no SDK at the top
level — a real PyhulaxProximitySensor reads through the SAME injected DroneAPI
the adapter/video source already share (no new import). Stdlib only.

Session: SENSE-IR.
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from typing import Callable, Optional

from finals.guards import ProximityReading


class ProximitySensor(ABC):
    """The 4-directional IR reading seam. One per drone; read() each agent
    tick. Returns the latest ProximityReading, or None when there is no usable
    reading (no IR feed wired / before the first sample) — the guard treats
    None as 'skip, no guess' (degrade-absent)."""

    @abstractmethod
    def read(self) -> Optional[ProximityReading]:
        """Latest 4-directional reading, or None when none is available."""


class SyntheticProximitySensor(ProximitySensor):
    """A dependency-free ProximitySensor for SITL / the live-IR-absent path.

    Two honest modes:
    - default (reading=None, the production default): returns None every read
      -> the ProximityGuard SKIPS. This is the truth on the swarm path until
      the pyhulax IR API is wired onsite: we have a guard, but no live sensor,
      so we must NOT fabricate a clear reading and pretend the lane is sensed.
    - scriptable (reading set, or a callable producing one): returns that
      reading (re-stamped `ts` with the injected clock each read) — drives
      SITL rehearsals / drills of the ladder without hardware. A callable form
      lets a rehearsal vary the range over time (an approach run).

    The clock is injectable so a re-stamped reading shares the agent's
    monotonic domain (its ts is then meaningful for any future staleness use).
    """

    def __init__(self, drone_id: str, *,
                 reading: "Optional[ProximityReading |"
                          " Callable[[], Optional[ProximityReading]]]" = None,
                 clock: Callable[[], float] = time.monotonic):
        if not isinstance(drone_id, str) or not drone_id:
            raise ValueError(
                f"SyntheticProximitySensor: drone_id must be a non-empty str, "
                f"got {drone_id!r} — check the wiring")
        if (reading is not None and not callable(reading)
                and not isinstance(reading, ProximityReading)):
            raise ValueError(
                f"SyntheticProximitySensor({drone_id!r}): reading must be None, "
                f"a ProximityReading, or a callable returning one — got "
                f"{reading!r}")
        self.drone_id = drone_id
        self._reading = reading
        self._clock = clock

    def read(self) -> Optional[ProximityReading]:
        src = self._reading
        if src is None:
            return None                       # no live IR -> guard SKIPS
        base = src() if callable(src) else src
        if base is None:
            return None
        # Re-stamp so ts tracks the agent's clock (the reading content is the
        # script; the freshness is now).
        return ProximityReading(
            ts=self._clock(),
            front_cm=base.front_cm, back_cm=base.back_cm,
            left_cm=base.left_cm, right_cm=base.right_cm)


class PyhulaxProximitySensor(ProximitySensor):
    """REFERENCE STUB — the LIVE 4-directional IR read (ONSITE GATE).

    Reads through the SAME pyhulax DroneAPI the adapter + video source share
    (the same-link invariant in main._build_agents). It is a STUB because
    pyhulax exposes no documented IR getter yet (see the module docstring):
    read() raises NotImplementedError pointing at the onsite step, so it can
    never be silently wired half-built. At the hardware window, replace the
    body with the real call (the likely shape, to confirm against the SDK):

        r = self._api.get_obstacle_distances()   # or get_ir() / get_proximity()
        return ProximityReading(
            ts=self._clock(),
            front_cm=r.front, back_cm=r.back, left_cm=r.left, right_cm=r.right)

    mapping each direction's cm reading (None where that direction reports no
    return). NO pyhulax import here — the api is injected, identical to the
    adapter's name-based seam.
    """

    def __init__(self, drone_id: str, api, *,
                 clock: Callable[[], float] = time.monotonic):
        if not isinstance(drone_id, str) or not drone_id:
            raise ValueError(
                f"PyhulaxProximitySensor: drone_id must be a non-empty str, "
                f"got {drone_id!r} — check the wiring")
        self.drone_id = drone_id
        self._api = api
        self._clock = clock

    def read(self) -> Optional[ProximityReading]:
        raise NotImplementedError(
            "finals.flight.proximity.PyhulaxProximitySensor.read — ONSITE GATE: "
            "pyhulax exposes no IR/proximity getter yet (see "
            "finals/docs/module_map.md / the module docstring). Confirm the "
            "avoidance API at the hardware window and implement read() against "
            "the shared DroneAPI, then flip guards.proximity_enable.")
