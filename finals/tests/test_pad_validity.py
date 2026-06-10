"""finals.mission.pad_validity — PadValidityMap, the C2 pad-validity authority.

100% pure: no gz, no SDK, no cv2, no clock reads (every `ts` is hand-fed). Proves
the three properties the cross-drone landing coordination rests on — a single
winner per contested claim (both same-tick AND under real thread contention), the
INVALID broadcast that steers other drones off a red pad, and the deterministic
last-writer/no-lost-update under concurrent records — plus the JSON-serializable
snapshot shape and the fail-loud input validation.

Mirrors test_convoy_registry.py (the sibling claim-authority's test layout).
"""
from __future__ import annotations

import json
import threading

import pytest

from finals.errors import FinalsError
from finals.mission.pad_validity import (PadRecord, PadValidityError,
                                         PadValidityMap)

T0 = 1000.0   # an arbitrary monotonic base; the map never reads the clock


# ---------------- is_valid before / after record ----------------
def test_is_valid_unknown_before_any_record():
    """A beacon NO drone has read is UNKNOWN (None) — NOT invalid. This is the
    distinction land_on_pad relies on: an unread valid beacon must still be
    landable via the static set."""
    pv = PadValidityMap()
    assert pv.is_valid(11) is None


def test_record_valid_then_is_valid_true():
    pv = PadValidityMap()
    pv.record(11, valid=True, drone_id="alpha", ts=T0)
    assert pv.is_valid(11) is True


def test_record_invalid_then_is_valid_false():
    """The INVALID (red) broadcast: a drone reads a red beacon -> records False
    -> every other drone reads it back False and skips that pad."""
    pv = PadValidityMap()
    pv.record(67, valid=False, drone_id="alpha", ts=T0)
    assert pv.is_valid(67) is False


def test_record_true_then_false_applies_invalid():
    """A valid pad that is later read INVALID flips to invalid (a True->False
    downgrade applies — the safety-monotone direction) + updates provenance."""
    pv = PadValidityMap()
    pv.record(51, valid=True, drone_id="alpha", ts=T0)
    pv.record(51, valid=False, drone_id="bravo", ts=T0 + 1)
    assert pv.is_valid(51) is False
    snap = pv.snapshot()
    assert snap["beacons"]["51"]["read_by"] == "bravo"


def test_invalid_is_sticky_never_upgraded_to_valid():
    """The safety-monotone rule: once a beacon is broadcast INVALID, a later
    valid=True does NOT resurrect it (a red pad stays red). The provenance still
    follows the latest read, but the verdict is pinned False — this is what makes
    one drone's invalid broadcast protective even if another drone's static set
    disagrees."""
    pv = PadValidityMap()
    pv.record(51, valid=False, drone_id="alpha", ts=T0)
    pv.record(51, valid=True, drone_id="bravo", ts=T0 + 1)   # cannot upgrade
    assert pv.is_valid(51) is False, "invalid must be sticky"
    snap = pv.snapshot()
    assert snap["beacons"]["51"]["read_by"] == "bravo"        # latest read noted
    assert snap["invalid_ids"] == [51]


def test_record_does_not_imply_claim():
    """Recording a beacon's validity must NOT claim it — the two facts are
    independent (a drone can broadcast a pad valid without committing to land)."""
    pv = PadValidityMap()
    pv.record(11, valid=True, drone_id="alpha", ts=T0)
    assert pv.claimed_by(11) is None
    assert pv.claim(11, "bravo") is True       # bravo can still take it


# ---------------- claim exclusivity (single-winner CAS) ----------------
def test_uncontested_claim_grants():
    pv = PadValidityMap()
    assert pv.claim(51, "alpha") is True
    assert pv.claimed_by(51) == "alpha"


def test_same_tick_double_claim_one_winner():
    pv = PadValidityMap()
    assert pv.claim(51, "alpha") is True
    assert pv.claim(51, "bravo") is False      # another drone already owns it
    assert pv.claimed_by(51) == "alpha"


def test_owner_reclaim_is_idempotent():
    pv = PadValidityMap()
    assert pv.claim(51, "alpha") is True
    assert pv.claim(51, "alpha") is True       # re-claim by the owner is True
    assert pv.claimed_by(51) == "alpha"


def test_claim_is_terminal_no_release():
    """No TTL/expire: landing is the terminal act, so a claimed pad stays
    claimed for the whole mission — another drone NEVER gets it."""
    pv = PadValidityMap()
    pv.claim(51, "alpha")
    assert pv.claim(51, "bravo") is False
    assert pv.claim(51, "charlie") is False
    assert pv.claimed_by(51) == "alpha"


def test_claimed_by_other_semantics():
    pv = PadValidityMap()
    assert pv.claimed_by_other(51, "alpha") is False   # unclaimed
    pv.claim(51, "alpha")
    assert pv.claimed_by_other(51, "bravo") is True    # held by another
    assert pv.claimed_by_other(51, "alpha") is False   # your own pad isn't 'other'


def test_claimed_by_other_unknown_beacon_is_false_and_non_mutating():
    pv = PadValidityMap()
    assert pv.claimed_by_other(999, "alpha") is False
    assert len(pv) == 0, "a read-only query must not create a record"


def test_claimed_by_non_mutating_on_unknown():
    pv = PadValidityMap()
    assert pv.claimed_by(999) is None
    assert len(pv) == 0


def test_claim_with_ts_stamps_provenance():
    pv = PadValidityMap()
    assert pv.claim_with_ts(101, "bravo", T0 + 5) is True
    assert pv.claim_with_ts(101, "alpha", T0 + 6) is False   # bravo owns it
    snap = pv.snapshot()
    assert snap["beacons"]["101"]["claimed_by"] == "bravo"
    assert snap["beacons"]["101"]["claimed_ts"] == T0 + 5


def test_two_drones_claim_two_different_pads_both_win():
    pv = PadValidityMap()
    assert pv.claim(51, "alpha") is True
    assert pv.claim(101, "bravo") is True
    assert pv.claimed_by(51) == "alpha"
    assert pv.claimed_by(101) == "bravo"


# ---------------- concurrency: single winner under real threads ----------------
def test_concurrent_claim_exactly_one_winner_under_threads():
    """Hammer one UNCLAIMED pad from many real threads; exactly one must win and
    it must be the recorded owner. Kills a 'drop the lock' mutation."""
    pv = PadValidityMap()
    winners = []
    barrier = threading.Barrier(24)

    def race(name: str) -> None:
        barrier.wait()                         # maximize the collision window
        if pv.claim(51, name):
            winners.append(name)

    threads = [threading.Thread(target=race, args=(f"d{i}",)) for i in range(24)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert len(winners) == 1, f"expected exactly one winner, got {winners}"
    assert pv.claimed_by(51) == winners[0]


def test_concurrent_records_deterministic_last_writer_no_lost_update():
    """Two writer threads each record a DIFFERENT beacon many times (valid=True
    throughout, so the sticky-invalid rule never fires and the last write is the
    deterministic outcome). Under the lock every write lands (no lost update) and
    each beacon's final {valid, read_by, read_ts} triple is internally consistent
    — NOT a torn/half-applied record mixing a new field with an old one."""
    pv = PadValidityMap()
    n = 2000
    barrier = threading.Barrier(2)

    def writer(beacon: int, drone: str) -> None:
        barrier.wait()
        for i in range(n):
            # read_ts is monotone with i, so the LAST applied write is i = n-1.
            pv.record(beacon, valid=True, drone_id=drone, ts=T0 + i)

    a = threading.Thread(target=writer, args=(11, "alpha"))
    b = threading.Thread(target=writer, args=(51, "bravo"))
    a.start(); b.start()
    a.join(); b.join()

    snap = pv.snapshot()
    for beacon, drone in ((11, "alpha"), (51, "bravo")):
        rec = snap["beacons"][str(beacon)]
        # Deterministic last-writer: the triple is internally consistent (value,
        # writer AND ts all from the SAME final write — a torn write under no
        # lock could mix a new field with an old one).
        assert rec["valid"] is True
        assert rec["read_by"] == drone
        assert rec["read_ts"] == T0 + (n - 1)


def test_lock_makes_claim_single_winner_deterministically():
    """DETERMINISTIC 'drop the lock' kill (not the flaky GIL-race above): two
    threads claim the SAME beacon; an injected _race_hook forces BOTH past the
    check->set boundary at once IF AND ONLY IF they can be inside the critical
    section together — which the real lock FORBIDS. So with the lock exactly one
    wins (the second thread is blocked OUT of the critical section, the barrier
    times out harmlessly); drop the lock and BOTH win -> this test FAILS every
    run. Pins mutual exclusion structurally, no probabilistic interleave."""

    class _ForcedRaceMap(PadValidityMap):
        def __init__(self):
            super().__init__()
            # Both threads must reach the hook for it to release; under the real
            # lock the 2nd can't enter the critical section, so it times out.
            self._barrier = threading.Barrier(2, timeout=0.5)

        def _race_hook(self):
            try:
                self._barrier.wait()
            except threading.BrokenBarrierError:
                pass        # the locked case: the 2nd thread never arrives

    pv = _ForcedRaceMap()
    winners = []
    start = threading.Barrier(2)

    def race(name: str) -> None:
        start.wait()
        if pv.claim(51, name):
            winners.append(name)

    threads = [threading.Thread(target=race, args=(n,))
               for n in ("alpha", "bravo")]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert len(winners) == 1, (
        f"exactly one drone may win a contested pad; got {winners} — the lock "
        f"is the single-winner guarantee")
    assert pv.claimed_by(51) == winners[0]


def test_concurrent_record_and_claim_same_beacon_consistent():
    """Records and claims hammer the SAME beacon from many threads at once; the
    map must end internally consistent: exactly one claim winner, and the
    validity readable (no exception, no torn state)."""
    pv = PadValidityMap()
    winners = []
    barrier = threading.Barrier(20)

    def worker(name: str, i: int) -> None:
        barrier.wait()
        pv.record(51, valid=True, drone_id=name, ts=T0 + i)
        if pv.claim(51, name):
            winners.append(name)

    threads = [threading.Thread(target=worker, args=(f"d{i}", i))
               for i in range(20)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert len(winners) == 1
    assert pv.claimed_by(51) == winners[0]
    assert pv.is_valid(51) is True


# ---------------- snapshot shape (JSON-serializable) ----------------
def test_snapshot_shape_and_json_serializable():
    pv = PadValidityMap()
    pv.record(67, valid=False, drone_id="alpha", ts=T0)      # red broadcast
    pv.record(11, valid=True, drone_id="alpha", ts=T0)
    pv.claim_with_ts(11, "alpha", T0 + 1)
    pv.claim_with_ts(51, "bravo", T0 + 1)
    snap = pv.snapshot(T0 + 2)

    assert snap["invalid_ids"] == [67]
    assert snap["claimed_by"] == {"11": "alpha", "51": "bravo"}
    b11 = snap["beacons"]["11"]
    assert b11["valid"] is True and b11["claimed_by"] == "alpha"
    assert b11["age_s"] == 2.0                               # now - read_ts
    # 51 was claimed but never recorded -> validity unknown, no read age.
    assert snap["beacons"]["51"]["valid"] is None
    assert snap["beacons"]["51"]["age_s"] is None
    json.dumps(snap)                                          # JSON-serializable


def test_snapshot_sorted_and_age_none_without_now():
    pv = PadValidityMap()
    for bid in (101, 11, 51):
        pv.record(bid, valid=True, drone_id="alpha", ts=T0)
    snap = pv.snapshot()                                      # no `now`
    assert list(snap["beacons"]) == ["11", "51", "101"]      # sorted by id
    assert all(b["age_s"] is None for b in snap["beacons"].values())


def test_empty_snapshot_is_clean():
    snap = PadValidityMap().snapshot()
    assert snap == {"beacons": {}, "invalid_ids": [], "claimed_by": {}}
    json.dumps(snap)


def test_invalid_ids_excludes_valid_and_unknown():
    pv = PadValidityMap()
    pv.record(11, valid=True, drone_id="alpha", ts=T0)       # valid
    pv.record(67, valid=False, drone_id="alpha", ts=T0)      # invalid
    pv.claim(51, "bravo")                                    # claimed, unread
    snap = pv.snapshot()
    assert snap["invalid_ids"] == [67]                       # ONLY the red one


# ---------------- len ----------------
def test_len_counts_seen_beacons():
    pv = PadValidityMap()
    assert len(pv) == 0
    pv.record(11, valid=True, drone_id="alpha", ts=T0)
    pv.claim(51, "bravo")
    assert len(pv) == 2


# ---------------- fail-loud validation ----------------
@pytest.mark.parametrize("bad", [None, "11", 11.0, True, False])
def test_bad_beacon_id_raises(bad):
    pv = PadValidityMap()
    with pytest.raises(PadValidityError):
        pv.record(bad, valid=True, drone_id="alpha", ts=T0)
    with pytest.raises(PadValidityError):
        pv.is_valid(bad)
    with pytest.raises(PadValidityError):
        pv.claim(bad, "alpha")


@pytest.mark.parametrize("bad", ["", None, 7, True])
def test_bad_drone_id_raises(bad):
    pv = PadValidityMap()
    with pytest.raises(PadValidityError):
        pv.record(11, valid=True, drone_id=bad, ts=T0)
    with pytest.raises(PadValidityError):
        pv.claim(11, bad)


@pytest.mark.parametrize("bad", [float("inf"), float("nan"), True, "now", None])
def test_bad_ts_raises(bad):
    pv = PadValidityMap()
    with pytest.raises(PadValidityError):
        pv.record(11, valid=True, drone_id="alpha", ts=bad)
    with pytest.raises(PadValidityError):
        pv.claim_with_ts(11, "alpha", bad)


@pytest.mark.parametrize("bad", [None, 1, 0, "yes", "True"])
def test_record_valid_must_be_bool(bad):
    pv = PadValidityMap()
    with pytest.raises(PadValidityError):
        pv.record(11, valid=bad, drone_id="alpha", ts=T0)   # type: ignore[arg-type]


def test_snapshot_now_validated():
    pv = PadValidityMap()
    with pytest.raises(PadValidityError):
        pv.snapshot(float("nan"))


def test_pad_validity_error_is_a_finals_error():
    # So a broad `except FinalsError` at the boundary catches map misuse.
    assert issubclass(PadValidityError, FinalsError)


def test_pad_record_is_a_dataclass_with_defaults():
    rec = PadRecord(beacon_id=11)
    assert rec.valid is None and rec.claimed_by is None
