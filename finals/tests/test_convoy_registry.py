"""finals.mission.convoy_registry — ConvoyRegistry, the C2 ownership authority.

100% pure: no gz, no SDK, no cv2, no clock reads (every `now` is hand-fed). Proves
the one property the whole dedup story rests on — a single winner per contested
claim — both logically (same-tick) and under real thread contention, plus the
heartbeat/expire liveness path and the 5-of-5 tally.
"""
from __future__ import annotations

import threading

import pytest

from finals.errors import FinalsError
from finals.mission.convoy_registry import (ConvoyRegistry, ConvoyStatus,
                                            RegistryError)

T0 = 1000.0   # an arbitrary monotonic base; the registry never reads the clock


# ---------------- single-winner CAS ----------------
def test_uncontested_claim_grants():
    reg = ConvoyRegistry()
    assert reg.claim("alpha", 7, T0) is True
    assert reg.owner_of(7, T0) == "alpha"


def test_same_tick_double_claim_one_winner():
    reg = ConvoyRegistry()
    assert reg.claim("alpha", 7, T0) is True
    assert reg.claim("bravo", 7, T0) is False     # a live owner holds it
    assert reg.owner_of(7, T0) == "alpha"


def test_concurrent_claim_exactly_one_winner_under_threads():
    """Hammer one UNCLAIMED id from many real threads; exactly one must win and
    it must be the recorded owner. Kills a 'drop the lock' mutation."""
    reg = ConvoyRegistry()
    winners = []
    barrier = threading.Barrier(24)

    def race(name: str) -> None:
        barrier.wait()                            # maximize the collision window
        if reg.claim(name, 7, T0):
            winners.append(name)

    threads = [threading.Thread(target=race, args=(f"d{i}",)) for i in range(24)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert len(winners) == 1, f"expected exactly one winner, got {winners}"
    assert reg.owner_of(7, T0) == winners[0]


def test_owner_reclaim_is_idempotent_and_refreshes_heartbeat():
    reg = ConvoyRegistry(lock_ttl_s=10.0)
    assert reg.claim("alpha", 7, T0) is True
    # Re-claim near the TTL edge refreshes the heartbeat, so it does NOT expire.
    assert reg.claim("alpha", 7, T0 + 9.0) is True
    assert reg.expire(T0 + 18.0) == [], "refreshed lock must not be stale yet"
    assert reg.owner_of(7, T0 + 18.0) == "alpha"


# ---------------- serviced finality ----------------
def test_serviced_is_never_reclaimable():
    reg = ConvoyRegistry()
    reg.claim("alpha", 7, T0)
    assert reg.release("alpha", 7, T0 + 1, serviced=True) is True
    assert reg.serviced_ids() == [7]
    assert reg.claim("bravo", 7, T0 + 2) is False
    assert reg.claimable_ids("bravo", T0 + 2, [7]) == []


def test_release_unserviced_returns_to_pool():
    reg = ConvoyRegistry()
    reg.claim("alpha", 7, T0)
    assert reg.release("alpha", 7, T0 + 1, serviced=False) is True
    assert reg.claim("bravo", 7, T0 + 2) is True   # back in the pool


def test_release_by_non_owner_is_a_noop_not_an_error():
    reg = ConvoyRegistry()
    reg.claim("alpha", 7, T0)
    # bravo never owned it: release is an expected-race no-op, returns False,
    # and must NOT disturb alpha's ownership.
    assert reg.release("bravo", 7, T0 + 1, serviced=True) is False
    assert reg.owner_of(7, T0 + 1) == "alpha"
    assert reg.serviced_ids() == []


# ---------------- heartbeat / expire / staleness ----------------
def test_renew_keeps_lock_alive_then_expire_after_silence():
    reg = ConvoyRegistry(lock_ttl_s=10.0)
    reg.claim("alpha", 7, T0)
    assert reg.renew("alpha", 7, T0 + 5) is True
    assert reg.expire(T0 + 14) == [], "5+9: within ttl of last renew"
    assert reg.expire(T0 + 16) == [7], "no renew for >10s -> LOST"
    assert reg.owner_of(7, T0 + 16) is None


def test_lost_id_is_reclaimable_by_another_drone():
    reg = ConvoyRegistry(lock_ttl_s=10.0)
    reg.claim("alpha", 7, T0)
    reg.expire(T0 + 11)
    assert reg.claim("bravo", 7, T0 + 11) is True
    assert reg.owner_of(7, T0 + 11) == "bravo"


def test_claim_succeeds_on_stale_lock_even_before_expire_runs():
    """Effective staleness: claim() must grant a stale CLAIMED id without relying
    on expire() having materialized it (defense in depth)."""
    reg = ConvoyRegistry(lock_ttl_s=10.0)
    reg.claim("alpha", 7, T0)
    assert reg.claim("bravo", 7, T0 + 11) is True   # no expire() call in between
    assert reg.owner_of(7, T0 + 11) == "bravo"


def test_renew_after_steal_returns_false():
    reg = ConvoyRegistry(lock_ttl_s=10.0)
    reg.claim("alpha", 7, T0)
    reg.expire(T0 + 11)
    reg.claim("bravo", 7, T0 + 11)                  # bravo stole the lost lock
    assert reg.renew("alpha", 7, T0 + 12) is False  # alpha must drop to re-acquire


def test_ttl_boundary_exact_vs_just_past():
    """Mutation kill: lock_ttl_s uses strict '>'. At exactly ttl it is NOT stale;
    a hair past, it is."""
    reg = ConvoyRegistry(lock_ttl_s=10.0)
    reg.claim("alpha", 7, T0)
    assert reg.expire(T0 + 10.0) == [], "age == ttl is not yet stale"
    assert reg.expire(T0 + 10.0001) == [7], "age just past ttl is stale"


def test_renew_unknown_or_unowned_returns_false():
    reg = ConvoyRegistry()
    assert reg.renew("alpha", 999, T0) is False      # never seen
    reg.claim("alpha", 7, T0)
    assert reg.renew("bravo", 7, T0) is False         # not bravo's


# ---------------- queries / tally ----------------
def test_claimable_ids_filters_correctly():
    reg = ConvoyRegistry(lock_ttl_s=10.0)
    reg.claim("alpha", 7, T0)                         # live other -> excluded
    reg.claim("bravo", 11, T0)
    reg.release("bravo", 11, T0 + 1, serviced=True)   # serviced -> excluded
    reg.claim("charlie", 23, T0)
    reg.expire(T0 + 11)                               # 7,23 now LOST -> claimable
    got = reg.claimable_ids("delta", T0 + 11, [7, 11, 23, 42])
    assert got == [7, 23, 42]                         # 11 serviced stays out


def test_claimable_includes_own_live_lock():
    reg = ConvoyRegistry()
    reg.claim("alpha", 7, T0)
    assert reg.claimable_ids("alpha", T0, [7, 11]) == [7, 11]
    assert reg.claimable_ids("bravo", T0, [7, 11]) == [11]


def test_seed_remaining_and_all_serviced():
    reg = ConvoyRegistry(known_ids=[7, 11, 23, 42, 88])
    assert reg.remaining_ids() == [7, 11, 23, 42, 88]
    assert reg.all_serviced() is False
    for cid in (7, 11, 23, 42, 88):
        reg.claim("alpha", cid, T0)
        reg.release("alpha", cid, T0, serviced=True)
    assert reg.remaining_ids() == []
    assert reg.all_serviced() is True


def test_all_serviced_false_when_nothing_known():
    reg = ConvoyRegistry()                            # no seed
    reg.claim("alpha", 7, T0)
    reg.release("alpha", 7, T0, serviced=True)
    assert reg.all_serviced() is False, "no denominator -> not 'done'"
    assert reg.all_serviced(known=[7]) is True        # explicit known overrides


def test_snapshot_shape_and_done_flag():
    reg = ConvoyRegistry(lock_ttl_s=10.0, known_ids=[7, 11, 23])
    reg.claim("alpha", 7, T0)
    reg.release("alpha", 7, T0 + 1, serviced=True)
    reg.claim("bravo", 11, T0 + 1)
    snap = reg.snapshot(T0 + 1)
    assert snap["serviced"] == [7]
    assert snap["in_flight"] == {"11": "bravo"}
    assert snap["remaining"] == [11, 23]
    assert snap["done"] is False
    # JSON-serializable: only str/int/list/dict/bool.
    import json
    json.dumps(snap)


def test_seed_is_idempotent_and_non_destructive():
    reg = ConvoyRegistry()
    reg.claim("alpha", 7, T0)
    reg.seed([7, 11])                                 # must not wipe alpha's claim
    assert reg.owner_of(7, T0) == "alpha"
    assert reg.remaining_ids() == [7, 11]


# ---------------- fail-loud validation ----------------
@pytest.mark.parametrize("ttl", [0, -1, float("inf"), float("nan"), True, "x"])
def test_bad_lock_ttl_raises(ttl):
    with pytest.raises(RegistryError):
        ConvoyRegistry(lock_ttl_s=ttl)


@pytest.mark.parametrize("drone", ["", None, 7, True])
def test_bad_drone_id_raises(drone):
    reg = ConvoyRegistry()
    with pytest.raises(RegistryError):
        reg.claim(drone, 7, T0)


@pytest.mark.parametrize("cid", [None, "7", 7.0, True])
def test_bad_convoy_id_raises(cid):
    reg = ConvoyRegistry()
    with pytest.raises(RegistryError):
        reg.claim("alpha", cid, T0)


@pytest.mark.parametrize("now", [float("inf"), float("nan"), True, "now", None])
def test_bad_now_raises(now):
    reg = ConvoyRegistry()
    with pytest.raises(RegistryError):
        reg.claim("alpha", 7, now)


def test_release_serviced_flag_must_be_bool():
    reg = ConvoyRegistry()
    reg.claim("alpha", 7, T0)
    with pytest.raises(RegistryError):
        reg.release("alpha", 7, T0, serviced="yes")   # type: ignore[arg-type]


def test_registry_error_is_a_finals_error():
    # So a broad `except FinalsError` at the boundary catches registry misuse.
    assert issubclass(RegistryError, FinalsError)


def test_status_enum_values_are_json_strings():
    assert ConvoyStatus.SERVICED.value == "serviced"
    assert ConvoyStatus.CLAIMED == "claimed"          # str-Enum equality
