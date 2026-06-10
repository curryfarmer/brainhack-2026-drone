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
    # --- WS-7A soft-zone handover (only meaningful while CLAIMED) ---
    #: The current owner flagged that the convoy left its assigned sector. The
    #: owner KEEPS tracking it (soft zoning, never a hard cut) while the
    #: orchestrator looks for an idle neighbour to hand it to.
    exited_zone: bool = False
    #: Bearing-from-C2 (deg, CCW+, the in_sector/Sighting.bearing_deg
    #: convention) the owner last saw the convoy at when it flagged the exit, so
    #: the orchestrator can decide WHICH neighbour's sector it entered without
    #: re-reading sightings. None until flagged.
    exit_bearing_deg: Optional[float] = None
    #: A pending handover offer: the idle neighbour the orchestrator picked.
    #: Only that drone may claim this convoy while the offer stands; cleared on
    #: accept, on the owner releasing, or when the owner re-enters its sector.
    offered_to: Optional[str] = None
    offered_ts: Optional[float] = None


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

    @staticmethod
    def _check_bearing(bearing_deg: Optional[float]) -> None:
        if bearing_deg is None:
            return                                          # bearing is optional
        if (not isinstance(bearing_deg, (int, float))
                or isinstance(bearing_deg, bool)
                or not math.isfinite(bearing_deg)):
            raise RegistryError(
                f"ConvoyRegistry: exit_bearing_deg={bearing_deg!r} invalid — "
                f"must be None or a finite number (deg from C2, CCW+, the "
                f"Sighting.bearing_deg convention) — check the flagging phase")

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

    @staticmethod
    def _clear_handover(entry: ConvoyEntry) -> None:
        """Wipe all soft-zone handover state on an entry (caller holds the lock).
        Called whenever ownership turns over or resets — a fresh owner starts
        in-zone with no pending offer, so a stale flag/offer can never leak
        across a claim/release/expire boundary."""
        entry.exited_zone = False
        entry.exit_bearing_deg = None
        entry.offered_to = None
        entry.offered_ts = None

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
                # A live OTHER drone holds it. The ONE exception is a standing
                # soft-zone handover offer to THIS drone: accept it (atomic
                # transfer under the same lock — the offeree's normal acquire
                # loop drives the handover through claim, no separate call).
                if entry.offered_to == drone_id:
                    return self._take_offer(entry, drone_id, now)
                return False                           # a live owner holds it
            # UNCLAIMED or (effectively) LOST -> grant a fresh lock.
            self._clear_handover(entry)                # fresh owner starts clean
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
            self._clear_handover(entry)                # offer/flag die with owner
            if serviced:
                entry.status = ConvoyStatus.SERVICED
                entry.serviced_ts = now
            else:
                entry.status = ConvoyStatus.UNCLAIMED
            return True

    # ---------------- WS-7A soft-zone handover ----------------
    def flag_exited(self, drone_id: str, convoy_id: int, now: float, *,
                    exited: bool = True,
                    exit_bearing_deg: Optional[float] = None) -> None:
        """The CURRENT OWNER marks that its tracked convoy left (exited=True) or
        re-entered (exited=False) its assigned sector. Soft zoning: the owner
        KEEPS tracking — this only records the flag (+ the bearing-from-C2 the
        convoy was last seen at) so the orchestrator can look for an idle
        neighbour to hand it to. exited=False clears any standing offer too (the
        convoy came back; no handover needed).

        Fail loud: only the live owner may flag its own convoy. A non-owner
        (or stale owner) flagging is a programming error — RegistryError naming
        WHAT/WHICH/WHY/CHECK — NOT a silent no-op, because a wrong-drone flag
        would offer away a convoy nobody is actually tracking."""
        self._check_drone(drone_id)
        self._check_convoy(convoy_id)
        self._check_now(now)
        if not isinstance(exited, bool):
            raise RegistryError(
                f"ConvoyRegistry.flag_exited: exited={exited!r} invalid — must "
                f"be a bool (True=left my sector, False=re-entered) — check the "
                f"track_convoy soft-zone call")
        self._check_bearing(exit_bearing_deg)
        with self._lock:
            entry = self._entries.get(convoy_id)
            if (entry is None or entry.owner_drone != drone_id
                    or self._effective_status(entry, now)
                    is not ConvoyStatus.CLAIMED):
                owner = entry.owner_drone if entry is not None else None
                raise RegistryError(
                    f"ConvoyRegistry.flag_exited: drone {drone_id!r} cannot flag "
                    f"convoy {convoy_id} — it is owned by {owner!r}, not this "
                    f"drone (or its lock went stale) — only the live owner may "
                    f"flag a zone exit; CHECK the track_convoy/registry wiring "
                    f"(the phase must hold a fresh claim before flagging)")
            entry.exited_zone = exited
            if exited:
                entry.exit_bearing_deg = exit_bearing_deg
            else:
                # Came back in-zone: drop the flag, the recorded bearing, and any
                # pending offer (the handover is moot).
                entry.exit_bearing_deg = None
                entry.offered_to = None
                entry.offered_ts = None

    def offer_to(self, convoy_id: int, target_drone: str, now: float) -> None:
        """Mark an EXITED convoy as offered to an idle neighbour (the
        orchestrator matcher calls this). Records the offeree; the offeree's
        normal acquire loop (claimable_ids -> claim) then takes ownership via
        accept_offer. Re-offering refreshes the target/timestamp (the matcher
        runs every tick).

        Fail loud: the convoy must be CLAIMED (a live owner is still tracking it)
        and flagged exited_zone — offering a convoy nobody owns, or one still
        in-zone, is a matcher bug. target_drone must be a different drone than
        the owner (you cannot hand a convoy to itself)."""
        self._check_convoy(convoy_id)
        self._check_drone(target_drone)
        self._check_now(now)
        with self._lock:
            entry = self._entries.get(convoy_id)
            if (entry is None
                    or self._effective_status(entry, now)
                    is not ConvoyStatus.CLAIMED):
                raise RegistryError(
                    f"ConvoyRegistry.offer_to: convoy {convoy_id} is not CLAIMED "
                    f"(no live owner is tracking it) — cannot offer it to "
                    f"{target_drone!r}; CHECK the matcher (only flagged, owned "
                    f"convoys are offerable)")
            if not entry.exited_zone:
                raise RegistryError(
                    f"ConvoyRegistry.offer_to: convoy {convoy_id} is not flagged "
                    f"exited_zone — only a convoy its owner flagged as having "
                    f"left its sector may be offered to {target_drone!r}; CHECK "
                    f"the matcher (it must read flagged_exits first)")
            if entry.owner_drone == target_drone:
                raise RegistryError(
                    f"ConvoyRegistry.offer_to: convoy {convoy_id} cannot be "
                    f"offered to its own owner {target_drone!r} — the offeree "
                    f"must be a DIFFERENT (idle neighbour) drone; CHECK the "
                    f"matcher's neighbour selection")
            entry.offered_to = target_drone
            entry.offered_ts = now

    def accept_offer(self, drone_id: str, convoy_id: int, now: float) -> bool:
        """The OFFERED drone atomically takes ownership of a convoy handed to it.
        Transfers the lock to drone_id under the registry lock and clears the
        offer + exited flag. Returns True on a successful transfer; False — NOT
        an error — when the offer is stale or rescinded (a different drone is the
        offeree, the convoy is no longer CLAIMED, or no offer stands), an
        expected race the caller just retries/re-acquires past.

        track_convoy reaches this through claim() (claimable_ids surfaces the
        offer, claim() transfers it); accept_offer is the explicit entry point +
        the unit-testable transfer core."""
        self._check_drone(drone_id)
        self._check_convoy(convoy_id)
        self._check_now(now)
        with self._lock:
            entry = self._entries.get(convoy_id)
            if entry is None or entry.offered_to != drone_id:
                return False                           # not offered to us
            if self._effective_status(entry, now) is not ConvoyStatus.CLAIMED:
                return False                           # owner already gone/stale
            return self._take_offer(entry, drone_id, now)

    def _take_offer(self, entry: ConvoyEntry, drone_id: str,
                    now: float) -> bool:
        """Transfer a standing offer to drone_id (caller holds the lock and has
        already confirmed entry.offered_to == drone_id and the entry is live
        CLAIMED). The ONE place ownership moves on a handover, so the offeree
        guard lives here: an offeree mismatch is a hard assert, never a silent
        wrong-drone transfer."""
        assert entry.offered_to == drone_id, (
            f"_take_offer: convoy {entry.convoy_id} offered_to "
            f"{entry.offered_to!r} != claimer {drone_id!r} — transfer guard")
        entry.owner_drone = drone_id
        entry.claimed_ts = now
        entry.last_heartbeat_ts = now
        self._clear_handover(entry)                    # fresh owner starts clean
        return True

    def flagged_exits(self, now: float) -> List[tuple]:
        """The orchestrator matcher's pull: every CLAIMED convoy whose owner has
        flagged it exited its sector, as (convoy_id, owner_drone,
        exit_bearing_deg, offered_to). Non-mutating; sorted by convoy_id for a
        deterministic match order. Stale/lost owners are skipped (effective
        status folds in heartbeat staleness), so a dropped drone's flag never
        triggers a handover from a convoy nobody is tracking."""
        self._check_now(now)
        out: List[tuple] = []
        with self._lock:
            for cid, e in sorted(self._entries.items()):
                if (e.exited_zone
                        and self._effective_status(e, now)
                        is ConvoyStatus.CLAIMED):
                    out.append((cid, e.owner_drone, e.exit_bearing_deg,
                                e.offered_to))
        return out

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
                        self._clear_handover(entry)    # owner gone -> offer dead
                        flipped.append(entry.convoy_id)
        return sorted(flipped)

    # ---------------- queries (the "pull") ----------------
    def claimable_ids(self, drone_id: str, now: float,
                      candidates: Iterable[int]) -> List[int]:
        """Of `candidates` (e.g. the ids a drone is seeing this tick), those it
        could claim right now: UNCLAIMED, (effectively) LOST, already its own, OR
        a live OTHER drone's convoy that has been OFFERED to this drone (a
        soft-zone handover — claim() will transfer it). Excludes SERVICED and
        ids a live other drone holds with no offer to us. Non-mutating."""
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
                elif eff is ConvoyStatus.CLAIMED and (
                        entry.owner_drone == drone_id
                        or entry.offered_to == drone_id):
                    out.append(cid)                    # ours, or handed to us
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
            exited, offered = [], {}
            for cid, e in sorted(self._entries.items()):
                eff = (self._effective_status(e, now)
                       if now is not None else e.status)
                if eff is ConvoyStatus.SERVICED:
                    serviced.append(cid)
                elif eff is ConvoyStatus.CLAIMED:
                    in_flight[str(cid)] = e.owner_drone
                    # WS-7A: surface the soft-zone handover state so the
                    # heartbeat shows which convoys left their owner's sector and
                    # which are mid-handover to a neighbour.
                    if e.exited_zone:
                        exited.append(cid)
                    if e.offered_to is not None:
                        offered[str(cid)] = e.offered_to
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
                "exited_zone": exited,
                "offered": offered,
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
