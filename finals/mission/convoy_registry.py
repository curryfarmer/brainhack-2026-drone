"""ConvoyRegistry — the C2 laptop's single authority on convoy ownership.

THE coordination object. On the day there are 3 drones and 5 ArUco convoys, and
two drones must never track the same convoy while the swarm tracks which of the 5
are done. Because ALL mission code runs in ONE python process on the C2 laptop
(drones over Wi-Fi via pyhulax; SITL stands in behind the same FlightAdapter),
this is a plain in-process shared object — method calls, not network pings. That
is the design with the fewest failure modes: no sockets, no packet loss, no TTL
races on the wire, no server-down-blinds-the-swarm. The user's "ping the ArUco id
to the laptop / other drones pull what's locked" maps exactly onto claim()/renew()
(the ping) and claimable_ids()/snapshot() (the pull).

Ownership lifecycle (ConvoyStatus):
    UNCLAIMED --claim--> CLAIMED --release(serviced=True)--> SERVICED   (done, final)
                            |  ^                                         (counts to 5-of-5)
                            |  +--claim (LOST or UNCLAIMED reclaim)
          release(serviced=False) / expire(stale)
                            v
                         UNCLAIMED / LOST  (back in the pool, re-claimable)

- claim()  = race-free single-winner CAS under one lock: of two drones reaching
  for the same UNCLAIMED id in one orchestrator tick, exactly one wins.
- renew()  = the heartbeat a tracking drone sends each track tick. Its ONE job is
  to keep the lock fresh so expire() can free a convoy whose drone dropped Wi-Fi
  (or wedged) WITHOUT releasing it. In a single synchronous process claims are
  already serialized on the loop, so the heartbeat is not needed for correctness
  of dedup — it is needed for liveness on the real radio link. Sized so a live
  drone (renew ~every track_dwell_s) never goes stale: lock_ttl_s >> renew cadence.
- expire() = orchestrator calls it each tick; CLAIMED entries with no heartbeat
  for lock_ttl_s flip to LOST (owner cleared) and become re-claimable.

Clock: every `now` MUST come from the SAME monotonic clock as the agents' ctx.now
and the orchestrator tick (time.monotonic() in this one process). The registry
never reads the clock itself — `now` is injected, exactly like MissionPhase.step,
so it is pure and unit-testable with hand-fed timestamps.

Thread-safety: one threading.Lock guards all state (same discipline as
SightingBus/SightingLog), so claim/renew/release/expire are safe from both the
asyncio loop and any detector thread. Every public mutator is small and does ONE
thing; nothing here performs I/O or sleeps.

Fail-loud: malformed inputs (empty drone id, non-int convoy id, non-finite now,
bad lock_ttl_s) raise RegistryError naming WHAT/WHICH/WHY. Expected race outcomes
(lost a contested claim, released an id ownership already moved off) are NOT
errors — they are the bool/return contract, so callers branch instead of crashing.

Session: WS-1 (convoy-coordination).
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, List, Optional, Set

from finals.errors import FinalsError


class RegistryError(FinalsError):
    """A ConvoyRegistry call was given malformed input (programming error).
    Carries the bad argument and what to check. Expected race outcomes (a lost
    claim, a stale release) are returned as booleans, NOT raised."""


class ConvoyStatus(str, Enum):
    """Ownership state of one convoy id. str-valued so snapshots serialize to
    plain JSON strings for the heartbeat file."""

    UNCLAIMED = "unclaimed"   # seen or seeded, nobody owns it — claimable
    CLAIMED = "claimed"       # a live drone owns and is heartbeating it
    SERVICED = "serviced"     # tracked to completion — final, never re-claimable
    LOST = "lost"             # owner stopped heartbeating (Wi-Fi drop) — claimable


@dataclass
class ConvoyEntry:
    """One convoy id's ownership record. Mutable; only the registry mutates it,
    only under the lock. snapshot() hands out copies, never these."""

    convoy_id: int
    status: ConvoyStatus = ConvoyStatus.UNCLAIMED
    owner_drone: Optional[str] = None
    first_seen_ts: Optional[float] = None
    claimed_ts: Optional[float] = None
    last_heartbeat_ts: Optional[float] = None
    serviced_ts: Optional[float] = None


class ConvoyRegistry:
    """Shared C2 authority: claim / renew / release / expire + a 5-of-5 tally.

    One instance per mission, owned by the orchestrator, passed by reference into
    every TrackConvoy phase. See the module docstring for the model.
    """

    def __init__(self, lock_ttl_s: float = 12.0,
                 known_ids: Optional[Iterable[int]] = None):
        if (not isinstance(lock_ttl_s, (int, float))
                or isinstance(lock_ttl_s, bool)
                or not math.isfinite(lock_ttl_s) or lock_ttl_s <= 0):
            raise RegistryError(
                f"ConvoyRegistry: lock_ttl_s={lock_ttl_s!r} invalid — must be a "
                f"finite number > 0 (seconds a CLAIMED lock survives without a "
                f"renew before expire() frees it) — check the config")
        self.lock_ttl_s = float(lock_ttl_s)
        self._lock = threading.Lock()
        self._entries: Dict[int, ConvoyEntry] = {}
        self._known: Set[int] = set()
        if known_ids is not None:
            self.seed(known_ids)

    # ---------------- input validation (fail loud) ----------------
    @staticmethod
    def _check_drone(drone_id: str) -> None:
        if not isinstance(drone_id, str) or not drone_id:
            raise RegistryError(
                f"ConvoyRegistry: drone_id={drone_id!r} invalid — must be a "
                f"non-empty str — check the caller (agent/phase) drone id")

    @staticmethod
    def _check_convoy(convoy_id: int) -> None:
        if not isinstance(convoy_id, int) or isinstance(convoy_id, bool):
            raise RegistryError(
                f"ConvoyRegistry: convoy_id={convoy_id!r} invalid — must be an "
                f"int ArUco marker id — check the sighting/marker source")

    @staticmethod
    def _check_now(now: float) -> None:
        if (not isinstance(now, (int, float)) or isinstance(now, bool)
                or not math.isfinite(now)):
            raise RegistryError(
                f"ConvoyRegistry: now={now!r} invalid — must be a finite "
                f"monotonic timestamp (time.monotonic()); the registry never "
                f"reads the clock itself")

    # ---------------- internal helpers (caller holds the lock) ----------------
    def _entry(self, convoy_id: int) -> ConvoyEntry:
        """Get-or-create the entry for an id (lazily, on first sighting/claim)."""
        entry = self._entries.get(convoy_id)
        if entry is None:
            entry = ConvoyEntry(convoy_id=convoy_id)
            self._entries[convoy_id] = entry
        return entry

    def _effective_status(self, entry: ConvoyEntry, now: float) -> ConvoyStatus:
        """Status with staleness folded in: a CLAIMED entry whose heartbeat has
        aged past lock_ttl_s reads as LOST even before expire() materializes it,
        so claim()/claimable_ids() are correct regardless of expire() timing."""
        if entry.status is ConvoyStatus.CLAIMED:
            hb = entry.last_heartbeat_ts
            if hb is None or (now - hb) > self.lock_ttl_s:
                return ConvoyStatus.LOST
        return entry.status

    # ---------------- core API ----------------
    def seed(self, convoy_ids: Iterable[int]) -> None:
        """Declare the full set of convoy ids expected this mission (e.g. the 5
        on the day) so remaining_ids()/all_serviced() have a denominator. Creates
        any unknown id as UNCLAIMED. Idempotent; never downgrades existing state."""
        ids = list(convoy_ids)
        for cid in ids:
            self._check_convoy(cid)
        with self._lock:
            for cid in ids:
                self._known.add(cid)
                if cid not in self._entries:
                    self._entries[cid] = ConvoyEntry(convoy_id=cid)

    def claim(self, drone_id: str, convoy_id: int, now: float) -> bool:
        """Try to take ownership of a convoy id. Returns True iff this drone now
        owns it. Race-free: under the lock, of two drones claiming the same
        UNCLAIMED/LOST id only the first wins. Idempotent for the current owner
        (re-claim refreshes the heartbeat). False if a live drone owns it or it
        is already SERVICED."""
        self._check_drone(drone_id)
        self._check_convoy(convoy_id)
        self._check_now(now)
        with self._lock:
            entry = self._entry(convoy_id)
            if entry.first_seen_ts is None:
                entry.first_seen_ts = now
            eff = self._effective_status(entry, now)
            if eff is ConvoyStatus.SERVICED:
                return False
            if eff is ConvoyStatus.CLAIMED:
                if entry.owner_drone == drone_id:
                    entry.last_heartbeat_ts = now      # idempotent refresh
                    return True
                return False                           # a live owner holds it
            # UNCLAIMED or (effectively) LOST -> grant.
            entry.status = ConvoyStatus.CLAIMED
            entry.owner_drone = drone_id
            entry.claimed_ts = now
            entry.last_heartbeat_ts = now
            return True

    def renew(self, drone_id: str, convoy_id: int, now: float) -> bool:
        """Heartbeat: refresh this drone's lock. Returns True if the lock is still
        ours and was refreshed; False if it expired, was stolen, was serviced, or
        we never owned it (caller should drop to re-acquire on False)."""
        self._check_drone(drone_id)
        self._check_convoy(convoy_id)
        self._check_now(now)
        with self._lock:
            entry = self._entries.get(convoy_id)
            if entry is None:
                return False
            if entry.owner_drone != drone_id:
                return False
            if self._effective_status(entry, now) is not ConvoyStatus.CLAIMED:
                return False                           # we went stale -> lost it
            entry.last_heartbeat_ts = now
            return True

    def release(self, drone_id: str, convoy_id: int, now: float, *,
                serviced: bool) -> bool:
        """Give up a lock. serviced=True -> SERVICED (done, counts to 5-of-5,
        never re-claimable); serviced=False -> UNCLAIMED (back in the pool: clean
        handover or a false lock). Returns True if this drone was the owner and
        the release applied; False if ownership had already moved off us (expired/
        stolen) — an expected race, not an error, so the caller can just log it."""
        self._check_drone(drone_id)
        self._check_convoy(convoy_id)
        self._check_now(now)
        if not isinstance(serviced, bool):
            raise RegistryError(
                f"ConvoyRegistry.release: serviced={serviced!r} invalid — must "
                f"be a bool (True=done/SERVICED, False=back to UNCLAIMED)")
        with self._lock:
            entry = self._entries.get(convoy_id)
            if entry is None or entry.owner_drone != drone_id:
                return False                           # not ours to release
            entry.owner_drone = None
            if serviced:
                entry.status = ConvoyStatus.SERVICED
                entry.serviced_ts = now
            else:
                entry.status = ConvoyStatus.UNCLAIMED
            return True

    def expire(self, now: float) -> List[int]:
        """Materialize staleness: flip every CLAIMED entry with no heartbeat for
        lock_ttl_s to LOST (clearing the owner) so it can be re-claimed. Returns
        the ids that flipped this call (for the orchestrator to log). Idempotent."""
        self._check_now(now)
        flipped: List[int] = []
        with self._lock:
            for entry in self._entries.values():
                if entry.status is ConvoyStatus.CLAIMED:
                    hb = entry.last_heartbeat_ts
                    if hb is None or (now - hb) > self.lock_ttl_s:
                        entry.status = ConvoyStatus.LOST
                        entry.owner_drone = None
                        flipped.append(entry.convoy_id)
        return sorted(flipped)

    # ---------------- queries (the "pull") ----------------
    def claimable_ids(self, drone_id: str, now: float,
                      candidates: Iterable[int]) -> List[int]:
        """Of `candidates` (e.g. the ids a drone is seeing this tick), those it
        could claim right now: UNCLAIMED, (effectively) LOST, or already its own.
        Excludes SERVICED and ids a live OTHER drone holds. Non-mutating."""
        self._check_drone(drone_id)
        self._check_now(now)
        cand = list(candidates)
        for cid in cand:
            self._check_convoy(cid)
        out: List[int] = []
        with self._lock:
            for cid in cand:
                entry = self._entries.get(cid)
                if entry is None:
                    out.append(cid)                    # never seen -> UNCLAIMED
                    continue
                eff = self._effective_status(entry, now)
                if eff in (ConvoyStatus.UNCLAIMED, ConvoyStatus.LOST):
                    out.append(cid)
                elif eff is ConvoyStatus.CLAIMED and entry.owner_drone == drone_id:
                    out.append(cid)
        return out

    def owner_of(self, convoy_id: int, now: float) -> Optional[str]:
        """The drone currently owning a convoy id (None if unowned/lost/serviced/
        unknown). Uses effective staleness so a dropped drone reads as no owner."""
        self._check_convoy(convoy_id)
        self._check_now(now)
        with self._lock:
            entry = self._entries.get(convoy_id)
            if entry is None:
                return None
            if self._effective_status(entry, now) is ConvoyStatus.CLAIMED:
                return entry.owner_drone
            return None

    def serviced_ids(self) -> List[int]:
        """Convoy ids tracked to completion (the 5-of-5 numerator)."""
        with self._lock:
            return sorted(cid for cid, e in self._entries.items()
                          if e.status is ConvoyStatus.SERVICED)

    def remaining_ids(self, known: Optional[Iterable[int]] = None) -> List[int]:
        """Known convoy ids not yet SERVICED. `known` overrides the seeded set;
        with neither seeded nor passed, remaining is empty (no denominator)."""
        with self._lock:
            universe = (set(known) if known is not None else set(self._known))
            done = {cid for cid, e in self._entries.items()
                    if e.status is ConvoyStatus.SERVICED}
            return sorted(universe - done)

    def all_serviced(self, known: Optional[Iterable[int]] = None) -> bool:
        """True iff the known set is non-empty and every id in it is SERVICED —
        i.e. the swarm is finished. False when nothing is known (no denominator)."""
        with self._lock:
            universe = (set(known) if known is not None else set(self._known))
            if not universe:
                return False
            done = {cid for cid, e in self._entries.items()
                    if e.status is ConvoyStatus.SERVICED}
            return universe <= done

    def snapshot(self, now: Optional[float] = None) -> dict:
        """JSON-serializable view for the heartbeat file (the swarm-wide 'pull').
        Pass `now` to fold in staleness (CLAIMED-but-stale reads as lost); the
        orchestrator calls expire(now) first, so raw status is already current."""
        if now is not None:
            self._check_now(now)
        with self._lock:
            serviced, in_flight, lost, unclaimed = [], {}, [], []
            for cid, e in sorted(self._entries.items()):
                eff = (self._effective_status(e, now)
                       if now is not None else e.status)
                if eff is ConvoyStatus.SERVICED:
                    serviced.append(cid)
                elif eff is ConvoyStatus.CLAIMED:
                    in_flight[str(cid)] = e.owner_drone
                elif eff is ConvoyStatus.LOST:
                    lost.append(cid)
                else:
                    unclaimed.append(cid)
            universe = set(self._known)
            remaining = sorted(universe - set(serviced)) if universe else []
            done = bool(universe) and universe <= set(serviced)
            return {
                "known": sorted(universe),
                "serviced": serviced,
                "in_flight": in_flight,
                "lost": lost,
                "unclaimed": unclaimed,
                "remaining": remaining,
                "done": done,
            }


# ============================================================
# Manual smoke demo
# ============================================================
if __name__ == "__main__":
    reg = ConvoyRegistry(lock_ttl_s=10.0, known_ids=[7, 11, 23, 42, 88])
    t = 100.0

    # Two drones reach for the same id in one tick: exactly one wins.
    assert reg.claim("alpha", 7, t) is True
    assert reg.claim("bravo", 7, t) is False, "double-claim must be refused"
    assert reg.claim("bravo", 23, t) is True
    print("claims:", reg.snapshot(t))

    # Heartbeats keep the locks alive; a third drone pulls what's free.
    t += 1.0
    assert reg.renew("alpha", 7, t) is True
    assert reg.claimable_ids("charlie", t, [7, 11, 23, 42]) == [11, 42], \
        "charlie should only see the unclaimed ids as claimable"

    # alpha finishes id 7 -> SERVICED, counts toward 5-of-5.
    t += 1.0
    assert reg.release("alpha", 7, t, serviced=True) is True
    assert reg.serviced_ids() == [7]
    assert reg.claim("charlie", 7, t) is False, "serviced is never re-claimable"

    # bravo drops Wi-Fi: no renew for > lock_ttl_s -> expire frees id 23.
    t += 11.0
    assert reg.expire(t) == [23], "stale lock must flip to LOST"
    assert reg.claim("charlie", 23, t) is True, "LOST id is re-claimable"
    print("after expiry+reclaim:", reg.snapshot(t))

    # Finish the rest -> done.
    for cid, drone in ((23, "charlie"), (11, "alpha"), (42, "bravo"), (88, "alpha")):
        reg.claim(drone, cid, t)
        reg.release(drone, cid, t, serviced=True)
    assert reg.all_serviced() is True, "all 5 serviced -> mission complete"
    print("final:", reg.snapshot(t))
    print("convoy_registry smoke OK")
