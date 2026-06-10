"""obstacle_map.py — the shared collective map of FIXED obstacles (extension).

The user's extension: "each drone builds a collective map of fixed obstacles and
shares it." Because all three drones' code runs in ONE process on the C2 laptop,
"shared" = ONE ObstacleMap instance every drone's navigate phase reads — exactly
like the in-process ConvoyRegistry ([[brainhack-project-state]]). A keep-out any
drone (or the operator pre-flight tap, finals.mission.planning.map_sensing lever
A) contributes is merged into EVERY drone's transit plan, so a drone routes
around a crate it never saw itself.

SINGLE PURPOSE: store observed keep-outs and merge them with the static arena.
It does NOT sense, plan, or localize — geometry lives in planning/, sensing in
map_sensing/vision, planning in visibility_graph. This module is a thread-safe
keyed store + a conservative merge, nothing more.

MERGE POLICY (deliberate, fail-safe): the hand-surveyed arena keep-outs are
AUTHORITATIVE. An observation may only ADD an obstacle the arena does not list;
it can NEVER override or delete a surveyed one. So a bad/duplicate observation of
a known crate is harmless (dropped at merge), while a genuinely new obstacle
still makes every drone detour. This is the honest behaviour for POSITION-BLIND
HULA: observations land in the drifting dead-reckon frame, so we trust them to
ADD caution, never to relax the surveyed map.

PURE: stdlib only (threading + planning.types). No cv2/numpy -> bare-venv green.
"""
from __future__ import annotations

import math
import threading
from typing import Dict, Iterable, Optional, Tuple

from finals.errors import FinalsError
from finals.mission.planning.types import KeepOut, Point


class MapError(FinalsError):
    """A bad contribution to the shared obstacle map (subsystem-local, the
    convoy_registry.RegistryError pattern). Names WHAT/WHICH/WHY/CHECK."""


def _validate_keep_out(keep_out: KeepOut, *, drone_id: str) -> None:
    """A contributed keep-out must be a real obstacle footprint: a KeepOut with a
    ring of >= 3 DISTINCT finite (north_m, east_m) vertices (the same area rule
    KeepOut.from_dict enforces for arena JSON). Fail LOUD — a degenerate polygon
    would silently vanish from the plan (point_in_polygon treats it as empty),
    re-admitting a collision the operator thought they had mapped."""
    if not isinstance(keep_out, KeepOut):
        raise MapError(
            f"ObstacleMap.add_keep_out: drone {drone_id!r} contributed a "
            f"{type(keep_out).__name__}, not a KeepOut — CHECK: build it via "
            f"map_sensing.keep_outs_from_overhead_corners or KeepOut.from_dict.")
    poly = keep_out.polygon_m
    if not isinstance(poly, (list, tuple)):
        raise MapError(
            f"ObstacleMap.add_keep_out: keep-out {keep_out.id!r} (drone "
            f"{drone_id!r}) polygon_m is {type(poly).__name__}, not a ring of "
            f"points — CHECK the contribution source.")
    for i, p in enumerate(poly):
        if (not isinstance(p, (list, tuple)) or len(p) != 2
                or any(not isinstance(c, (int, float)) or isinstance(c, bool)
                       or not math.isfinite(c) for c in p)):
            raise MapError(
                f"ObstacleMap.add_keep_out: keep-out {keep_out.id!r} (drone "
                f"{drone_id!r}) vertex [{i}] = {p!r} is not a finite "
                f"[north_m, east_m] — CHECK the rectification that produced it.")
    distinct = []
    for p in poly:
        pt = (float(p[0]), float(p[1]))
        if pt not in distinct:
            distinct.append(pt)
    if len(distinct) < 3:
        raise MapError(
            f"ObstacleMap.add_keep_out: keep-out {keep_out.id!r} (drone "
            f"{drone_id!r}) has only {len(distinct)} distinct vertex/vertices — "
            f"a keep-out must enclose AREA (>= 3) or it vanishes from the plan. "
            f"CHECK: did the operator tap a full footprint, not a point/edge?")


class ObstacleMap:
    """Shared, thread-safe store of observed keep-outs, keyed by id.

    One instance per mission (finals.main builds it and threads the SAME object
    into every drone's navigate, so the map is genuinely collective). Re-adding
    an id updates it (last writer wins) and records the contributing drone + ts
    as provenance. Like SightingBus/ConvoyRegistry, all state changes hold a
    threading.Lock so concurrent contributions resolve cleanly.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        # id -> (KeepOut, contributing drone_id, observed-at ts)
        self._by_id: Dict[str, Tuple[KeepOut, str, float]] = {}

    def add_keep_out(self, drone_id: str, keep_out: KeepOut,
                     now: float) -> bool:
        """Contribute one observed keep-out. Returns True if `keep_out.id` was
        NEW to the map, False if it updated an existing id (re-tap / a second
        drone mapping the same crate). Raises MapError (loud) on a degenerate
        polygon — a contribution is data, and bad data must never silently
        weaken the map."""
        if not isinstance(drone_id, str) or not drone_id:
            raise MapError(
                f"ObstacleMap.add_keep_out: drone_id must be a non-empty str, "
                f"got {drone_id!r}.")
        if not isinstance(now, (int, float)) or isinstance(now, bool) \
                or not math.isfinite(now):
            raise MapError(
                f"ObstacleMap.add_keep_out: now must be a finite timestamp, got "
                f"{now!r} (drone {drone_id!r}, keep-out "
                f"{getattr(keep_out, 'id', '?')!r}).")
        _validate_keep_out(keep_out, drone_id=drone_id)
        with self._lock:
            is_new = keep_out.id not in self._by_id
            self._by_id[keep_out.id] = (keep_out, drone_id, float(now))
            return is_new

    def keep_outs(self) -> Tuple[KeepOut, ...]:
        """Snapshot of the contributed keep-outs, sorted by id (deterministic)."""
        with self._lock:
            return tuple(ko for ko, _d, _t in
                         (self._by_id[k] for k in sorted(self._by_id)))

    def merge(self, static_keep_outs: Iterable[KeepOut]) -> Tuple[KeepOut, ...]:
        """The plan-time obstacle set: every static (surveyed) keep-out, plus
        each observed keep-out whose id the static set does NOT already define.
        Static is authoritative (an observation only ADDS, never overrides), so
        the result never drops a surveyed obstacle. Order: static first (input
        order), then the extra observed ids sorted — deterministic for tests."""
        static = list(static_keep_outs)
        static_ids = {ko.id for ko in static}
        with self._lock:
            extra = [self._by_id[k][0] for k in sorted(self._by_id)
                     if k not in static_ids]
        return tuple(static) + tuple(extra)

    def snapshot(self, now: Optional[float] = None) -> dict:
        """Provenance for the heartbeat: id -> {drone, observed_ts, n_vertices,
        age_s}. age_s is None unless `now` is given. Non-mutating."""
        with self._lock:
            out = {}
            for kid, (ko, drone, ts) in self._by_id.items():
                out[kid] = {
                    "drone": drone,
                    "observed_ts": ts,
                    "n_vertices": len(ko.polygon_m),
                    "age_s": (None if now is None else float(now) - ts),
                }
            return out

    def contributors(self) -> Dict[str, str]:
        """id -> the drone that last contributed it (provenance for logs)."""
        with self._lock:
            return {kid: drone for kid, (_ko, drone, _ts) in self._by_id.items()}

    def __len__(self) -> int:
        with self._lock:
            return len(self._by_id)
