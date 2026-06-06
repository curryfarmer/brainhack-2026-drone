"""finals.flight.mock_adapter + BenchAdapter — the contract suite the whole
test pyramid stands on.

No pytest-asyncio: coroutines are driven with asyncio.run() inside sync
tests (zero new deps). The mock's pipeline order (record -> connect gate ->
degraded gate -> flying gate -> fail injection -> latency/deadline ->
effects) is pinned here so S4/S5 can rely on it.
"""
from __future__ import annotations

import asyncio
import time

import pytest

from finals.errors import FlightError, FlightTimeout
from finals.flight.adapter import BenchAdapter
from finals.flight.dead_reckon import DeadReckoner
from finals.flight.mock_adapter import MockAdapter
from finals.types import Direction, PositionQuality

EPS = 1e-12


def approx(value: float):
    return pytest.approx(value, abs=EPS)


class FakeClock:
    """Deterministic monotonic source — tests set .t explicitly."""

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


async def _connected(drone_id: str = "alpha", **kw) -> MockAdapter:
    m = MockAdapter(drone_id, **kw)
    await m.connect()
    return m


async def _flying(drone_id: str = "alpha", **kw) -> MockAdapter:
    m = await _connected(drone_id, **kw)
    await m.takeoff(80)
    return m


# ============================================================
# 1. Contract happy path — .calls order + args EXACT
# ============================================================
def test_happy_path_calls_order_and_args_exact():
    async def script():
        m = MockAdapter("alpha")
        await m.connect()
        await m.takeoff(80)
        await m.move(Direction.FORWARD, 100)
        await m.rotate(90.0)
        await m.hover(0.5)
        await m.set_led(255, 0, 64)
        await m.land()
        await m.emergency_land()
        await m.disconnect()
        return m

    m = asyncio.run(script())
    assert m.calls == [
        ("connect", {"timeout_s": 10.0}),
        ("takeoff", {"height_cm": 80, "timeout_s": 30.0}),
        ("move", {"direction": Direction.FORWARD, "distance_cm": 100,
                  "timeout_s": 15.0}),
        ("rotate", {"angle_deg": 90.0, "timeout_s": 15.0}),
        ("hover", {"duration_s": 0.5}),
        ("set_led", {"r": 255, "g": 0, "b": 64}),
        ("land", {"timeout_s": 30.0}),
        ("emergency_land", {}),
        ("disconnect", {}),
    ]
    assert not m.degraded
    assert not m.is_flying


# ============================================================
# 2. Pre-connect gating (commands -> FlightError; attempts recorded)
# ============================================================
@pytest.mark.parametrize("name, call", [
    ("takeoff", lambda m: m.takeoff(80)),
    ("land", lambda m: m.land()),
    ("move", lambda m: m.move(Direction.FORWARD, 100)),
    ("rotate", lambda m: m.rotate(90.0)),
    ("hover", lambda m: m.hover(1.0)),
    ("set_led", lambda m: m.set_led(255, 0, 0)),
])
def test_pre_connect_command_raises_and_is_recorded(name, call):
    m = MockAdapter("alpha")
    with pytest.raises(FlightError, match=r"alpha.*not connected"):
        asyncio.run(call(m))
    assert [c[0] for c in m.calls] == [name]    # the ATTEMPT is on record


def test_pre_connect_telemetry_raises():
    m = MockAdapter("alpha")
    with pytest.raises(FlightError, match=r"alpha.*never connected"):
        m.telemetry()
    assert m.telemetry_calls == 1


def test_pre_connect_emergency_land_never_raises():
    m = MockAdapter("alpha")
    asyncio.run(m.emergency_land())             # must not raise
    assert m.calls == [("emergency_land", {})]
    assert not m.is_flying


# ============================================================
# 3. Scriptable failures
# ============================================================
def test_fail_on_raises_configured_type_and_message_every_call():
    injected = FlightError("alpha: move(FORWARD, 100 cm) failed — "
                           "scripted link drop — check the scenario")

    async def script():
        m = await _flying(fail_on={"move": injected})
        for _ in range(2):                      # EVERY call raises it
            with pytest.raises(FlightError, match="scripted link drop") as ei:
                await m.move(Direction.FORWARD, 100)
            assert ei.value is injected
        return m

    m = asyncio.run(script())
    assert not m.degraded                       # plain FlightError: clean fail
    assert [c[0] for c in m.calls].count("move") == 2
    assert m.dr.pose.north_m == 0.0             # no silent partial success


def test_injected_flighttimeout_degrades_then_refuses():
    injected = FlightTimeout("alpha: move(FORWARD, 100 cm) exceeded 15.0 s "
                             "— check Wi-Fi link / drone power")

    async def script():
        m = await _flying(fail_on={"move": injected})
        with pytest.raises(FlightTimeout):
            await m.move(Direction.FORWARD, 100)
        assert m.degraded
        # Second move is refused by the DEGRADED gate (FlightError, not the
        # injected timeout): the previous command may still be executing.
        with pytest.raises(FlightError, match="degraded") as ei:
            await m.move(Direction.FORWARD, 100)
        assert not isinstance(ei.value, FlightTimeout)
        return m

    asyncio.run(script())


def test_fail_at_exactly_third_move():
    async def script():
        m = await _flying(fail_at="move:3")
        await m.move(Direction.FORWARD, 100)    # 1: ok
        await m.move(Direction.FORWARD, 100)    # 2: ok
        with pytest.raises(FlightTimeout, match=r"alpha.*move\(FORWARD, 100 cm\)"
                                                r".*fail_at='move:3'") as ei:
            await m.move(Direction.FORWARD, 100)
        assert "check" in str(ei.value)
        return m

    m = asyncio.run(script())
    # State stays consistent: exactly the 2 COMPLETED moves are integrated.
    assert m.dr.pose.north_m == approx(2.0)
    assert m.degraded
    assert [c[0] for c in m.calls].count("move") == 3   # all 3 attempts logged


def test_gated_refusals_do_not_consume_fail_at_counter():
    async def script():
        m = MockAdapter("alpha", fail_at="move:2")
        with pytest.raises(FlightError, match="not connected"):
            await m.move(Direction.FORWARD, 100)        # refused, NOT counted
        await m.connect()
        await m.takeoff(80)
        await m.move(Direction.FORWARD, 100)            # true attempt 1: ok
        with pytest.raises(FlightTimeout):
            await m.move(Direction.FORWARD, 100)        # true attempt 2: fails
        return m

    m = asyncio.run(script())
    assert m.dr.pose.north_m == approx(1.0)


def test_flying_gate_refusal_does_not_consume_fail_at_counter():
    """Gate 4 (not flying) refusals must not count as attempts either."""
    async def script():
        m = await _connected(fail_at="rotate:1")
        with pytest.raises(FlightError, match="not flying"):
            await m.rotate(90.0)                        # refused, NOT counted
        await m.takeoff(80)
        with pytest.raises(FlightTimeout, match="fail_at='rotate:1'"):
            await m.rotate(90.0)                        # true attempt 1: fires

    asyncio.run(script())


def test_degraded_gate_refusal_does_not_consume_fail_at_counter():
    """Gate 3 (degraded) refusals must not count as attempts either."""
    async def script():
        m = await _flying(fail_at="hover:1")
        m.latency_s = 5.0                       # degrade via a real timeout
        with pytest.raises(FlightTimeout):
            await m.move(Direction.FORWARD, 100, timeout_s=2.0)
        m.latency_s = 0.0
        with pytest.raises(FlightError, match="degraded"):
            await m.hover(1.0)                  # refused by gate 3, NOT counted
        await m.connect()                       # clears degraded, still flying
        with pytest.raises(FlightTimeout, match="fail_at='hover:1'"):
            await m.hover(1.0)                  # true attempt 1: fires

    asyncio.run(script())


def test_argument_gate_refuses_impossible_magnitudes():
    """dead_reckon.py assigns 'refusing physically impossible sequences' to
    the adapter — pin that the mock holds up its side (a mock that completes
    a negative move lets phase-math sign bugs first surface on hardware)."""
    cases = [
        ("takeoff", lambda m: m.takeoff(-80)),
        ("takeoff", lambda m: m.takeoff(0)),
        ("move", lambda m: m.move(Direction.FORWARD, -100)),
        ("move", lambda m: m.move(Direction.FORWARD, 0)),
        ("rotate", lambda m: m.rotate(float("nan"))),
        ("hover", lambda m: m.hover(-1.0)),
    ]

    async def script():
        for name, call in cases:
            m = await _connected()
            if name != "takeoff":
                await m.takeoff(80)                     # pass the flying gate
            pose_before = m.dr.pose
            with pytest.raises(FlightError, match=r"alpha.*refused") as ei:
                await call(m)
            assert "check" in str(ei.value)
            assert m.dr.pose == pose_before             # nothing integrated

    asyncio.run(script())


def test_fail_on_connect_leaves_adapter_unconnected():
    injected = FlightTimeout("alpha: connect() exceeded 10.0 s — check "
                             "Wi-Fi / drone power")

    async def script():
        m = MockAdapter("alpha", fail_on={"connect": injected})
        with pytest.raises(FlightTimeout):
            await m.connect()
        assert m.degraded                       # timeout degrades, even here
        with pytest.raises(FlightError, match="not connected"):
            await m.takeoff(80)
        with pytest.raises(FlightError, match="never connected"):
            m.telemetry()
        return m

    asyncio.run(script())


# ============================================================
# 4. Latency vs deadline (immediate, deterministic) + message bar
# ============================================================
def test_latency_over_timeout_raises_immediately_with_actionable_message():
    async def script():
        m = await _flying()
        m.latency_s = 5.0                       # public + mutable on purpose
        start = time.perf_counter()
        with pytest.raises(FlightTimeout) as ei:
            await m.move(Direction.FORWARD, 100, timeout_s=2.0)
        elapsed = time.perf_counter() - start
        return m, ei.value, elapsed

    m, exc, elapsed = asyncio.run(script())
    assert elapsed < 1.5                        # immediate — no 2 s wait
    # (generous margin: only a >1.5 s machine stall could flake this, while
    # an implementation that waited out timeout_s would take >= 2 s)
    msg = str(exc)
    # The errors.py bar: WHAT, WHICH drone, against WHICH limit, WHAT to check.
    assert "alpha" in msg
    assert "move(FORWARD, 100 cm)" in msg
    assert "2.0 s" in msg
    assert "check" in msg
    assert m.degraded
    assert m.dr.pose.north_m == 0.0             # the move did NOT complete


def test_latency_within_timeout_succeeds_after_sleep():
    async def script():
        m = await _flying(latency_s=0.01)
        await m.move(Direction.FORWARD, 100, timeout_s=5.0)
        return m

    m = asyncio.run(script())
    assert m.dr.pose.north_m == approx(1.0)
    assert not m.degraded


def test_latency_equal_to_timeout_just_makes_it():
    """The deadline check is strictly >: equality means the command JUST
    completed in time (pins the boundary so > can never drift to >=)."""
    async def script():
        m = await _flying(latency_s=0.01)
        await m.move(Direction.FORWARD, 100, timeout_s=0.01)
        return m

    m = asyncio.run(script())
    assert m.dr.pose.north_m == approx(1.0)
    assert not m.degraded


def test_hover_and_set_led_have_no_deadline_and_never_sleep_duration():
    """hover()/set_led() carry no timeout_s in the ABC: exempt from the
    deadline comparison (duration < latency must NOT raise FlightTimeout)
    and the mock must never sleep duration_s wall-clock."""
    async def script():
        m = await _flying(latency_s=0.01)
        start = time.perf_counter()
        await m.hover(30.0)             # would take 30 s if duration slept
        await m.hover(0.001)            # duration < latency: still no timeout
        await m.set_led(1, 2, 3)
        return m, time.perf_counter() - start

    m, elapsed = asyncio.run(script())
    assert elapsed < 5.0                # generous CI margin, far below 30 s
    assert not m.degraded


def test_mutated_nonfinite_latency_raises_instead_of_hanging():
    """latency_s is public-mutable; a non-finite value reaching the
    deadline-less hover/set_led path would await forever — it must raise."""
    async def script():
        m = await _flying()
        m.latency_s = float("inf")
        with pytest.raises(ValueError, match=r"alpha.*latency_s"):
            await m.hover(1.0)
        m.latency_s = float("nan")      # NaN would silently act as zero
        with pytest.raises(ValueError, match=r"alpha.*latency_s"):
            await m.move(Direction.FORWARD, 100)

    asyncio.run(script())


def test_emergency_land_lands_the_dead_reckoner_too():
    """The documented effect (DR notes Land -> alt 0, track kept) — the
    never-tested-by-accident path: a DR still reporting altitude after an
    emergency landing would corrupt every later Sighting annotation."""
    async def script():
        m = await _flying()
        await m.move(Direction.FORWARD, 100)
        await m.emergency_land()
        return m

    m = asyncio.run(script())
    assert not m.is_flying
    assert m.dr.pose.alt_m == 0.0               # landed in the DR's eyes too
    assert m.dr.pose.north_m == approx(1.0)     # track kept (no teleport)


# ============================================================
# 5. Degraded matrix + reconnect semantics
# ============================================================
def test_degraded_matrix_and_reconnect_clears_but_pose_persists():
    async def script():
        m = await _flying(fail_at="move:2")
        await m.move(Direction.FORWARD, 100)            # ok -> north 1.0
        with pytest.raises(FlightTimeout):
            await m.move(Direction.FORWARD, 100)
        assert m.degraded

        # Refused while degraded: ordinary commands.
        for coro in (m.move(Direction.FORWARD, 100), m.rotate(90.0),
                     m.hover(1.0), m.set_led(0, 255, 0)):
            with pytest.raises(FlightError, match="degraded"):
                await coro
        # takeoff is degraded-refused too (flying gate is BEHIND it).
        with pytest.raises(FlightError, match="degraded"):
            await m.takeoff(80)

        # Still allowed while degraded: the safe-down surface.
        m.telemetry()                                   # no raise
        await m.land()                                  # no raise
        await m.emergency_land()                        # no raise
        await m.disconnect()                            # no raise

        # Reconnect = operator intervention: degraded clears, pose persists.
        await m.connect()
        assert not m.degraded
        assert m.dr.pose.north_m == approx(1.0)         # no teleport
        assert not m.is_flying                          # landed before
        await m.takeoff(80)                             # flies again
        return m

    asyncio.run(script())


def test_is_flying_persists_across_reconnect():
    async def script():
        m = await _flying()
        await m.disconnect()
        await m.connect()
        return m

    m = asyncio.run(script())
    assert m.is_flying                          # still airborne — no teleport


# ============================================================
# 6. Guard hooks: battery decay + telemetry freeze (deterministic)
# ============================================================
def test_battery_decays_per_completed_command_and_clamps_at_floor():
    async def script():
        m = await _connected(battery_start_pct=100.0,
                             battery_decay_pct_per_cmd=30.0,
                             battery_floor_pct=20.0)
        assert m.battery_pct == 100.0           # connect does not drain
        await m.takeoff(80)                     # -> 70
        await m.move(Direction.FORWARD, 100)    # -> 40
        await m.rotate(90.0)                    # -> 10, clamped -> 20
        await m.hover(0.1)                      # floor holds
        return m

    m = asyncio.run(script())
    assert m.battery_pct == 20.0
    assert m.telemetry().battery_pct == 20.0


def test_failed_command_does_not_drain_battery():
    async def script():
        m = await _connected(battery_decay_pct_per_cmd=10.0,
                             fail_at="move:1")
        await m.takeoff(80)                     # -> 90
        with pytest.raises(FlightTimeout):
            await m.move(Direction.FORWARD, 100)
        return m

    m = asyncio.run(script())
    assert m.battery_pct == 90.0                # no drain on failure


def test_battery_drain_set_membership_edges():
    """Pin both edges of the _DRAINING set: land DOES drain (motors run all
    the way down) and set_led does NOT (LED draw is negligible)."""
    async def script():
        m = await _connected(battery_decay_pct_per_cmd=10.0)
        await m.takeoff(80)                     # -> 90
        await m.set_led(255, 0, 0)              # not draining: stays 90
        assert m.battery_pct == 90.0
        await m.land()                          # draining: -> 80
        return m

    m = asyncio.run(script())
    assert m.battery_pct == 80.0


def test_telemetry_freeze_values_constant_ts_pinned_age_grows():
    clock = FakeClock(100.0)

    async def script():
        m = await _connected(freeze_telemetry_after_s=5.0, clock=clock)
        await m.takeoff(80)

        clock.t = 102.0                         # before the freeze threshold
        live = m.telemetry()
        assert live.ts == 102.0
        assert live.altitude_m == approx(0.8)

        clock.t = 105.0                         # at threshold: freezes NOW
        frozen = m.telemetry()
        assert frozen.ts == 105.0               # pinned at connect_ts + 5

        await m.move(Direction.FORWARD, 100)    # pose moves on...
        clock.t = 200.0
        later = m.telemetry()
        assert later is frozen                  # ...telemetry does NOT
        assert later.ts == 105.0
        assert later.position_m[0] == 0.0       # pre-move north, frozen
        assert later.age_s(now=clock.t) == approx(95.0)   # age keeps growing
        return m

    asyncio.run(script())


def test_freeze_latches_before_post_threshold_command_effects():
    """Commands completing AFTER the freeze instant must not leak into the
    frozen snapshot: the latch happens before effects are applied, so the
    frozen values are the state at the freeze instant — pairing the pinned
    freeze ts with later state would fabricate forensic history."""
    clock = FakeClock(100.0)

    async def script():
        m = await _connected(freeze_telemetry_after_s=5.0, clock=clock)
        clock.t = 102.0
        await m.takeoff(80)                     # before threshold: in snapshot
        clock.t = 107.0                         # threshold (105) has passed...
        await m.move(Direction.FORWARD, 100)    # ...latch fires BEFORE effects
        frozen = m.telemetry()
        return m, frozen

    m, frozen = asyncio.run(script())
    assert m.dr.pose.north_m == approx(1.0)     # the move really completed...
    assert frozen.ts == 105.0                   # ...but the snapshot is the
    assert frozen.position_m[0] == 0.0          # state AT the freeze instant
    assert frozen.altitude_m == approx(0.8)     # (takeoff was pre-threshold)


def test_reconnect_restarts_freeze_window():
    clock = FakeClock(100.0)

    async def script():
        m = await _connected(freeze_telemetry_after_s=5.0, clock=clock)
        clock.t = 110.0
        assert m.telemetry().ts == 105.0        # frozen
        await m.disconnect()
        clock.t = 300.0
        await m.connect()                       # window restarts at 300
        clock.t = 301.0
        live = m.telemetry()
        assert live.ts == 301.0                 # live again
        return m

    asyncio.run(script())


def test_post_disconnect_telemetry_returns_stale_final_snapshot():
    clock = FakeClock(100.0)

    async def script():
        m = await _connected(clock=clock)
        await m.takeoff(80)
        clock.t = 110.0
        await m.disconnect()
        clock.t = 150.0
        snap1 = m.telemetry()
        snap2 = m.telemetry()
        return snap1, snap2

    snap1, snap2 = asyncio.run(script())
    assert snap1 is snap2                       # one frozen final snapshot
    assert snap1.ts == 110.0                    # captured AT disconnect
    assert snap1.age_s(now=150.0) == approx(40.0)
    assert snap1.altitude_m == approx(0.8)


# ============================================================
# 7. telemetry() observability split
# ============================================================
def test_telemetry_not_in_calls_but_counted():
    async def script():
        m = await _flying()
        m.telemetry()
        m.telemetry()
        return m

    m = asyncio.run(script())
    assert m.telemetry_calls == 2
    assert all(name != "telemetry" for name, _ in m.calls)


# ============================================================
# 8. Constructor validation — typos must fail loudly, not never-fire
# ============================================================
@pytest.mark.parametrize("kwargs", [
    {"fail_on": {"disconnect": FlightError("x")}},      # never-raise contract
    {"fail_on": {"emergency_land": FlightError("x")}},  # never-raise contract
    {"fail_on": {"mvoe": FlightError("x")}},            # typo
    {"fail_on": {"move": "not an exception"}},
    {"fail_at": "move"},                                # no :N
    {"fail_at": "move:x"},
    {"fail_at": "move:0"},                              # 1-based
    {"fail_at": "move:-1"},
    {"fail_at": "frobnicate:1"},
    {"fail_at": "move:³"},                              # isdigit-but-not-int
    {"fail_on": {"move": FlightTimeout("x")}, "fail_at": "move:2"},  # masked
    {"latency_s": -0.5},
    {"latency_s": float("inf")},                        # would hang hover()
    {"latency_s": float("nan")},
    {"battery_start_pct": 50.0, "battery_floor_pct": 60.0},
    {"freeze_telemetry_after_s": -1.0},
])
def test_constructor_rejects_bad_config(kwargs):
    with pytest.raises(ValueError, match="alpha"):
        MockAdapter("alpha", **kwargs)


# ============================================================
# 9. Flying-state gates
# ============================================================
def test_double_takeoff_refused():
    async def script():
        m = await _flying()
        with pytest.raises(FlightError, match=r"alpha.*already flying"):
            await m.takeoff(80)

    asyncio.run(script())


@pytest.mark.parametrize("call", [
    lambda m: m.move(Direction.FORWARD, 100),
    lambda m: m.rotate(90.0),
    lambda m: m.hover(1.0),
])
def test_move_rotate_hover_require_flying(call):
    async def script():
        m = await _connected()
        with pytest.raises(FlightError, match=r"alpha.*not flying"):
            await call(m)

    asyncio.run(script())


def test_land_is_idempotent_even_when_never_flew():
    async def script():
        m = await _connected()
        await m.land()                          # never flew: still fine
        await m.takeoff(80)
        await m.land()
        await m.land()                          # repeat: still fine
        return m

    m = asyncio.run(script())
    assert not m.is_flying


# ============================================================
# 10. Mock + DR integration (single source of truth)
# ============================================================
def test_telemetry_position_comes_from_dead_reckoner():
    async def script():
        m = await _flying()
        await m.move(Direction.FORWARD, 100)
        await m.rotate(90.0)
        await m.move(Direction.FORWARD, 100)    # facing west now
        return m, m.telemetry()

    m, t = asyncio.run(script())
    pose = m.dr.pose
    assert t.position_m == (pose.north_m, pose.east_m, pose.alt_m)
    assert t.position_m[0] == approx(1.0)
    assert t.position_m[1] == approx(-1.0)
    assert t.position_m[2] == approx(0.8)
    assert t.yaw_deg == approx(90.0)
    assert t.altitude_m == approx(0.8)
    assert t.position_quality is PositionQuality.DEAD_RECKONING
    assert t.is_flying is True
    assert t.age_s() < 5.0                      # ts fresh (real clock; bound
    #                          generous so only a 5 s machine stall flakes it)


def test_injected_dead_reckoner_is_shared():
    dr = DeadReckoner()

    async def script():
        m = MockAdapter("alpha", dead_reckoner=dr)
        await m.connect()
        await m.takeoff(80)
        await m.move(Direction.FORWARD, 100)
        return m

    m = asyncio.run(script())
    assert m.dr is dr
    assert dr.pose.north_m == approx(1.0)       # mock wrote through it


# ============================================================
# 11. Concurrency / failure isolation (the S4 two-agent foundation)
# ============================================================
def test_two_adapters_gather_one_failure_does_not_poison_the_other():
    async def script():
        alpha = MockAdapter("alpha", latency_s=0.001)
        bravo = MockAdapter("bravo", latency_s=0.001, fail_at="move:2")

        async def fly_alpha() -> str:
            await alpha.connect()
            await alpha.takeoff(80)
            for _ in range(4):
                await alpha.move(Direction.FORWARD, 100)
                await alpha.rotate(90.0)
            await alpha.land()
            return "completed"

        async def fly_bravo() -> str:
            await bravo.connect()
            await bravo.takeoff(80)
            await bravo.move(Direction.FORWARD, 100)
            try:
                await bravo.move(Direction.FORWARD, 100)
            except FlightTimeout:
                await bravo.emergency_land()
                return "timed out, safed down"
            return "UNEXPECTED: injection never fired"

        return await asyncio.gather(fly_alpha(), fly_bravo()), alpha, bravo

    results, alpha, bravo = asyncio.run(script())
    assert results == ["completed", "timed out, safed down"]

    # alpha: untouched by bravo's failure — full square, clean log.
    assert not alpha.degraded
    assert [c[0] for c in alpha.calls] == (
        ["connect", "takeoff"] + ["move", "rotate"] * 4 + ["land"])
    assert alpha.dr.pose.north_m == approx(0.0)
    assert alpha.dr.pose.east_m == approx(0.0)

    # bravo: its own story only.
    assert bravo.degraded
    assert [c[0] for c in bravo.calls] == [
        "connect", "takeoff", "move", "move", "emergency_land"]
    assert bravo.dr.pose.north_m == approx(1.0)     # one completed move
    assert not bravo.is_flying


# ============================================================
# 12. BenchAdapter — flight refused, non-flight delegated
# ============================================================
@pytest.mark.parametrize("cmd_name, call", [
    ("takeoff", lambda b: b.takeoff(80)),
    ("land", lambda b: b.land()),
    ("move", lambda b: b.move(Direction.FORWARD, 100)),
    ("rotate", lambda b: b.rotate(90.0)),
    ("hover", lambda b: b.hover(1.0)),
])
def test_bench_refuses_every_flight_command(cmd_name, call):
    inner = MockAdapter("alpha")
    bench = BenchAdapter(inner)

    async def script():
        await bench.connect()
        with pytest.raises(FlightError, match="bench") as ei:
            await call(bench)
        return ei.value

    exc = asyncio.run(script())
    msg = str(exc)
    assert "alpha" in msg and cmd_name in msg and "props-off" in msg
    # The refused command NEVER reached the wrapped adapter.
    assert all(name != cmd_name for name, _ in inner.calls)


def test_bench_delegates_non_flight_surface():
    inner = MockAdapter("alpha")
    bench = BenchAdapter(inner)

    async def script():
        await bench.connect(timeout_s=3.0)
        await bench.set_led(0, 0, 255)
        t = bench.telemetry()
        await bench.disconnect()
        return t

    t = asyncio.run(script())
    assert bench.drone_id == "alpha"            # mirrors the inner adapter
    assert [c[0] for c in inner.calls] == ["connect", "set_led", "disconnect"]
    assert inner.calls[0] == ("connect", {"timeout_s": 3.0})
    assert t.position_quality is PositionQuality.DEAD_RECKONING  # inner's data


def test_bench_emergency_land_logged_noop_never_delegates(capsys):
    inner = MockAdapter("alpha")
    bench = BenchAdapter(inner)
    asyncio.run(bench.emergency_land())         # must not raise, even pre-connect
    assert inner.calls == []                    # NOT delegated
    err = capsys.readouterr().err
    assert "bench" in err and "no-op" in err and "alpha" in err


def test_bench_rejects_non_adapter_inner_with_actionable_message():
    """The S4 generic flight_cls(drone_id) wiring is the predicted misuse —
    it must die with the wiring note, not a bare AttributeError."""
    with pytest.raises(TypeError, match=r"INNER FlightAdapter.*'str'") as ei:
        BenchAdapter("alpha")                   # type: ignore[arg-type]
    assert "special case" in str(ei.value)
