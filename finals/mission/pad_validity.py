"""pad_validity.py — the C2 laptop's single authority on landing-pad validity.

THE pad-coordination object (Roboverse landing). On the day there are 5 landing
pads, each with an ArUco BEACON beside it; a pad is VALID iff its beacon id is in
the safe-codes the organizers hand over right before showcase
(zone["land_on_pad"].valid_marker_ids). Three drones must (a) NOT all re-read the
same INVALID (red) beacon — the first drone to read it BROADCASTS the verdict so
the others skip it — and (b) NOT two-at-once chase/claim the SAME valid pad.

Because ALL mission code runs in ONE python process on the C2 laptop (drones over
Wi-Fi via pyhulax; SITL/Mock stand in behind the same FlightAdapter), "shared" =
ONE PadValidityMap instance every drone's land_on_pad phase reads — method calls,
not network pings. That is the design with the fewest failure modes: no sockets,
no packet loss, no TTL races on the wire, no server-down-blinds-the-swarm. It is
exactly the in-process model of ObstacleMap (the collective keep-out store) and
ConvoyRegistry (the convoy claim/CAS authority) — this file is their pad-validity
sibling.

Two independent facts per beacon id, both under one lock:

  VALIDITY  record(beacon_id, valid, drone_id, ts): the cross-drone broadcast.
            A drone that reads a beacon writes whether its pad is valid.
            INVALID IS STICKY (safety-monotone): once any drone broadcasts a
            beacon invalid (False) it stays invalid for the whole mission — a
            later valid=True NEVER resurrects a red pad (validity is STATIC in
            reality, so a True-after-False is a disagreement and invalid wins).
            A True otherwise records normally; provenance follows the latest read.
            is_valid(beacon_id) reads it back: None until ANY drone has read it
            (unknown — fall through to the static set), then the recorded bool.
            An INVALID broadcast (False) makes every other drone's
            _valid_sightings drop that beacon so nobody wastes battery
            re-checking a red pad.

  CLAIM     claim(beacon_id, drone_id): race-free single-winner CAS under the
            lock — of two drones reaching for the same unclaimed pad in one
            orchestrator tick, exactly one wins. Idempotent for the current owner
            (re-claim by the same drone is True). False if another live drone
            already owns it. There is NO TTL/expire here (unlike ConvoyRegistry):
            landing is the TERMINAL act of the mission — a drone that claims a pad
            is committing to land on it and does not hand it back. (A future
            release()/expire() is a deliberate non-goal; see the module note.)

Clock: every `ts` is INJECTED (the same time.monotonic() the agents' ctx.now and
the orchestrator tick use); the map never reads the clock itself, so it is pure
and unit-testable with hand-fed timestamps — the ObstacleMap/ConvoyRegistry
discipline.

Thread-safety: one threading.Lock guards ALL state (the SightingBus / ObstacleMap
/ ConvoyRegistry discipline), so record/claim/is_valid/snapshot are safe from both
the asyncio loop and any detector thread. Every mutator is small and does ONE
thing; nothing here performs I/O or sleeps.

Fail-loud: malformed inputs (empty/None drone id, non-int beacon id, non-bool
valid, non-finite ts) raise PadValidityError naming WHAT/WHICH/WHY/CHECK. Expected
outcomes (a lost claim) are returned as a bool, NOT raised — callers branch.

Bounded + deterministic: state is a dict keyed by beacon id (the 5 fixed ids on
the day); snapshot() emits sorted, JSON-serializable views. No module globals
(convention 4).

PURE: stdlib only (threading + finals.errors). No cv2/numpy -> bare-venv green.

Session: PAD-VALID (Roboverse-landing).
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Dict, Optional

from finals.errors import FinalsError


class PadValidityError(FinalsError):
    """A PadValidityMap call was given malformed input (a programming/wiring
    error). Names WHAT/WHICH/WHY/CHECK. Subsystem-local, mirroring
    obstacle_map.MapError / convoy_registry.RegistryError. Expected race
    outcomes (a lost claim) are returned as booleans, NOT raised."""


@dataclass
class PadRecord:
    """One beacon id's coordination record. Mutable; ONLY the map mutates it,
    and ONLY under the lock. snapshot() reads copies of these fields, never
    hands the record out."""

    beacon_id: int
    #: None until ANY drone has read this beacon, then its broadcast validity.
    valid: Optional[bool] = None
    #: Provenance of the LAST validity write (who/when) — for the heartbeat log.
    read_by: Optional[str] = None
    read_ts: Optional[float] = None
    #: The drone that holds this pad (None = unclaimed). At most one ever.
    claimed_by: Optional[str] = None
    claimed_ts: Optional[float] = None


class PadValidityMap:
    """Shared, thread-safe pad-validity broadcast + single-owner claim store,
    keyed by beacon id.

    One instance per mission, owned by the orchestrator/main and passed by
    reference into every drone's land_on_pad phase. See the module docstring for
    the model. All state changes hold a threading.Lock, so concurrent
    contributions resolve cleanly (single-winner claims, sticky-invalid validity).
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._by_id: Dict[int, PadRecord] = {}

    # ---------------- input validation (fail loud) ----------------
    @staticmethod
    def _check_beacon(beacon_id: int) -> None:
        # bool is an int subclass — a True/False beacon id is always a wiring
        # bug (the marker_id source is an int ArUco id), so reject it.
        if not isinstance(beacon_id, int) or isinstance(beacon_id, bool):
            raise PadValidityError(
                f"PadValidityMap: beacon_id={beacon_id!r} invalid — must be an "
                f"int ArUco beacon id — check the sighting/marker source "
                f"(Sighting.marker_id)")

    @staticmethod
    def _check_drone(drone_id: str) -> None:
        if not isinstance(drone_id, str) or not drone_id:
            raise PadValidityError(
                f"PadValidityMap: drone_id={drone_id!r} invalid — must be a "
                f"non-empty str — check the caller (agent/phase) drone id")

    @staticmethod
    def _check_ts(ts: float) -> None:
        if (not isinstance(ts, (int, float)) or isinstance(ts, bool)
                or not math.isfinite(ts)):
            raise PadValidityError(
                f"PadValidityMap: ts={ts!r} invalid — must be a finite "
                f"monotonic timestamp (the agent's ctx.now / time.monotonic()); "
                f"the map never reads the clock itself")

    # ---------------- internal helper (caller holds the lock) ----------------
    def _record(self, beacon_id: int) -> PadRecord:
        """Get-or-create the record for a beacon id (lazily, on first
        read/claim). Caller MUST hold self._lock."""
        rec = self._by_id.get(beacon_id)
        if rec is None:
            rec = PadRecord(beacon_id=beacon_id)
            self._by_id[beacon_id] = rec
        return rec

    def _race_hook(self) -> None:
        """No-op extension seam called INSIDE the lock at each mutator's
        check->set boundary (the TOCTOU point). In production it does nothing;
        a concurrency test OVERRIDES it to force a deterministic thread switch
        there, so a 'dropped the lock' mutant fails the single-winner test every
        run (not flaky — the GIL otherwise hides the tiny critical section). It
        stays inside the lock, so overriding it never breaks mutual exclusion."""
        return None

    # ---------------- core API ----------------
    def record(self, beacon_id: int, valid: bool, drone_id: str,
               ts: float) -> None:
        """Broadcast one drone's validity reading of a beacon.

        INVALID IS STICKY (the safety-monotone rule): once ANY drone has
        broadcast a beacon invalid (False), a later valid=True NEVER resurrects
        it — a red pad stays red for the whole mission. This is what makes the
        cross-drone broadcast genuinely protective: one drone's INVALID read
        steers EVERY other drone off that pad even when a second drone's own
        (possibly mis-configured / differing) static set would have called it
        valid. Validity is STATIC in reality (a beacon's number does not change),
        so the only way True follows False is a disagreement, and the safe
        resolution is invalid-wins. A True broadcast otherwise records normally
        (first read, or re-confirming a still-valid pad). The provenance
        (read_by/read_ts) always records the LATEST write that was applied.

        Loud on malformed input (a bad read is data, and bad data must never
        silently corrupt the shared verdict)."""
        self._check_beacon(beacon_id)
        if not isinstance(valid, bool):
            raise PadValidityError(
                f"PadValidityMap.record: valid={valid!r} invalid (beacon "
                f"{beacon_id!r}, drone {drone_id!r}) — must be a bool (True = a "
                f"valid/green pad, False = an invalid/red pad to broadcast-skip) "
                f"— check the valid_marker_ids membership test that produced it")
        self._check_drone(drone_id)
        self._check_ts(ts)
        with self._lock:
            rec = self._record(beacon_id)
            self._race_hook()        # test seam: force an interleave here
            if rec.valid is False and valid is True:
                # Invalid is sticky — a red pad is never upgraded to green. Keep
                # the False verdict, but still record WHO/WHEN last looked (the
                # broadcast is live; provenance follows the latest read).
                rec.read_by = drone_id
                rec.read_ts = float(ts)
                return
            rec.valid = valid
            rec.read_by = drone_id
            rec.read_ts = float(ts)

    def is_valid(self, beacon_id: int) -> Optional[bool]:
        """The broadcast validity of a beacon: None if NO drone has read it yet
        (unknown — the caller falls through to the static valid_marker_ids set),
        else the recorded bool. A None here is NOT "invalid" — it is "not yet
        known", a distinction land_on_pad relies on (an unread valid beacon must
        still be landable via the static set)."""
        self._check_beacon(beacon_id)
        with self._lock:
            rec = self._by_id.get(beacon_id)
            return None if rec is None else rec.valid

    def claim(self, beacon_id: int, drone_id: str) -> bool:
        """Try to take exclusive ownership of a pad (by its beacon id). Returns
        True iff this drone now owns it. Race-free: under the lock, of two drones
        claiming the same UNCLAIMED pad only the first wins. Idempotent for the
        current owner (a re-claim by the same drone is True). False if another
        live drone already owns it. No TTL — landing is terminal, the owner does
        not hand the pad back (see the module note)."""
        self._check_beacon(beacon_id)
        self._check_drone(drone_id)
        with self._lock:
            rec = self._record(beacon_id)
            unclaimed = rec.claimed_by is None
            self._race_hook()        # test seam: force an interleave here
            if unclaimed:
                rec.claimed_by = drone_id
                return True
            return rec.claimed_by == drone_id

    def claim_with_ts(self, beacon_id: int, drone_id: str, ts: float) -> bool:
        """claim(), additionally stamping the claim time for the heartbeat
        provenance. Same single-winner contract as claim(). Kept separate so the
        hot-path claim() stays ts-free (a caller that does not care about
        provenance need not thread a clock)."""
        self._check_beacon(beacon_id)
        self._check_drone(drone_id)
        self._check_ts(ts)
        with self._lock:
            rec = self._record(beacon_id)
            unclaimed = rec.claimed_by is None
            self._race_hook()        # test seam: force an interleave here
            if unclaimed:
                rec.claimed_by = drone_id
                rec.claimed_ts = float(ts)
                return True
            return rec.claimed_by == drone_id

    # ---------------- queries (non-mutating) ----------------
    def claimed_by(self, beacon_id: int) -> Optional[str]:
        """The drone currently owning a pad (None if unclaimed/unknown).
        Non-mutating — does NOT create a record for an unseen beacon (so a
        read-only query never pollutes the map)."""
        self._check_beacon(beacon_id)
        with self._lock:
            rec = self._by_id.get(beacon_id)
            return None if rec is None else rec.claimed_by

    def claimed_by_other(self, beacon_id: int, drone_id: str) -> bool:
        """True iff a DIFFERENT drone holds this pad — the predicate
        land_on_pad's _valid_sightings uses to exclude a pad another drone is
        already committing to. False when unclaimed, unknown, or held by this
        same drone (so re-seeing your own pad never excludes it). Non-mutating."""
        self._check_beacon(beacon_id)
        self._check_drone(drone_id)
        with self._lock:
            rec = self._by_id.get(beacon_id)
            if rec is None or rec.claimed_by is None:
                return False
            return rec.claimed_by != drone_id

    # ---------------- snapshot (the swarm-wide 'pull') ----------------
    def snapshot(self, now: Optional[float] = None) -> dict:
        """JSON-serializable view for the heartbeat file (the swarm-wide pull):
        per-beacon {valid, read_by, read_ts, claimed_by, claimed_ts, age_s} plus
        flat invalid_ids / claimed_by maps for at-a-glance reading. Sorted by
        beacon id (deterministic). age_s is None unless `now` is given.
        Non-mutating."""
        if now is not None:
            self._check_ts(now)
        with self._lock:
            beacons: Dict[str, dict] = {}
            invalid_ids = []
            claimed = {}
            for bid in sorted(self._by_id):
                rec = self._by_id[bid]
                beacons[str(bid)] = {
                    "valid": rec.valid,
                    "read_by": rec.read_by,
                    "read_ts": rec.read_ts,
                    "claimed_by": rec.claimed_by,
                    "claimed_ts": rec.claimed_ts,
                    "age_s": (None if now is None or rec.read_ts is None
                              else float(now) - rec.read_ts),
                }
                if rec.valid is False:
                    invalid_ids.append(bid)
                if rec.claimed_by is not None:
                    claimed[str(bid)] = rec.claimed_by
            return {
                "beacons": beacons,
                "invalid_ids": invalid_ids,   # broadcast red pads (skip these)
                "claimed_by": claimed,        # beacon id (str) -> owning drone
            }

    def __len__(self) -> int:
        with self._lock:
            return len(self._by_id)


# ============================================================
# Manual smoke demo
# ============================================================
if __name__ == "__main__":
    import json

    pv = PadValidityMap()
    t = 100.0

    # A reads beacon 67 and finds it INVALID -> broadcast -> B/C skip it.
    pv.record(67, valid=False, drone_id="alpha", ts=t)
    assert pv.is_valid(67) is False, "the red broadcast must read back False"
    assert pv.is_valid(11) is None, "an unread beacon is UNKNOWN (None), not red"

    # Two drones reach for the same valid pad in one tick: exactly one wins.
    assert pv.claim(51, "alpha") is True
    assert pv.claim(51, "bravo") is False, "double-claim must be refused"
    assert pv.claim(51, "alpha") is True, "owner re-claim is idempotent"
    assert pv.claimed_by_other(51, "bravo") is True
    assert pv.claimed_by_other(51, "alpha") is False, "your own pad is not 'other'"

    # bravo takes a different valid pad.
    assert pv.claim_with_ts(101, "bravo", t + 1.0) is True

    snap = pv.snapshot(t + 2.0)
    assert snap["invalid_ids"] == [67]
    assert snap["claimed_by"] == {"51": "alpha", "101": "bravo"}
    json.dumps(snap)        # JSON-serializable
    print("snapshot:", json.dumps(snap, indent=2))
    print("pad_validity smoke OK")
