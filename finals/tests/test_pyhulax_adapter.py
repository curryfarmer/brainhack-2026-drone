"""finals.flight.pyhulax_adapter — the real backend, tested WITHOUT pyhulax.

Everything here runs on the bare dev venv: PyhulaxAdapter is driven through a
FakeDroneAPI (injected via api=) whose fake SDK exceptions are NAMED exactly
like pyhulax's, so the name-based mapper, the executor deadline / degrade
latch, the battery-failsafe-always connect, the telemetry mapping, and the
never-raise safe-down paths are all exercised with the SDK absent.

That this module imports and constructs at all (its pyhulax imports are
method-local) is itself the seam under test.
"""
from __future__ import annotations

import asyncio
import time

import pytest

from finals.errors import FlightError, FlightTimeout
from finals.flight.pyhulax_adapter import (CommandRejected, CommandTimeout,
                                           DroneConnectionError, FakeDroneAPI,
                                           NotReady, PyhulaxAdapter)
from finals.types import Direction, PositionQuality


def make_adapter(api=None, **kw):
    if api is None:
        api = FakeDroneAPI()
    return PyhulaxAdapter("alpha", ip="192.168.1.50", api=api, **kw), api


def run_connected(adapter, check):
    """connect() -> run check(adapter) -> disconnect(), all on ONE loop so the
    executor pool + poller thread are set up and torn down cleanly."""
    async def _wrap():
        await adapter.connect()
        try:
            return await check(adapter)
        finally:
            await adapter.disconnect()
    return asyncio.run(_wrap())


# ============================================================
# Constructor (no I/O, no pyhulax)
# ============================================================
def test_constructor_no_io_no_pyhulax():
    a = PyhulaxAdapter("alpha")
    assert a.drone_id == "alpha"
    assert a.degraded is False
    assert a._ip is None


def test_constructor_rejects_empty_drone_id():
    with pytest.raises(ValueError, match="drone_id"):
        PyhulaxAdapter("")


@pytest.mark.parametrize("kw, match", [
    (dict(ip=""), "ip"),
    (dict(ip=123), "ip"),
    (dict(poll_hz=0), "poll_hz"),
    (dict(poll_hz=float("inf")), "poll_hz"),
    (dict(fresh_s=-1), "fresh_s"),
    (dict(hover_margin_s=0), "hover_margin_s"),
])
def test_constructor_rejects_bad_args(kw, match):
    with pytest.raises(ValueError, match=match):
        PyhulaxAdapter("alpha", **kw)


def test_telemetry_before_connect_raises():
    with pytest.raises(FlightError, match="never connected"):
        PyhulaxAdapter("alpha", ip="1.2.3.4").telemetry()


def test_connect_without_ip_refuses():
    with pytest.raises(FlightError, match="no target IP"):
        asyncio.run(PyhulaxAdapter("alpha").connect())


# ============================================================
# set_target_ip (S10: preflight P3 applies the discovered IP pre-connect)
# ============================================================
def test_set_target_ip_applies_before_connect():
    api = FakeDroneAPI()
    a = PyhulaxAdapter("alpha", api=api)          # no ip — discovery resolves it

    async def _wrap():
        a.set_target_ip("192.168.1.77")
        await a.connect()
        try:
            assert a._connected is True
            connect_calls = [c for c in api.calls if c[0] == "connect"]
            assert connect_calls[0][1]["ip"] == "192.168.1.77"
        finally:
            await a.disconnect()
    asyncio.run(_wrap())


def test_set_target_ip_rejects_empty():
    with pytest.raises(ValueError, match="non-empty"):
        PyhulaxAdapter("alpha").set_target_ip("")


def test_set_target_ip_refused_after_connect():
    a, _ = make_adapter(FakeDroneAPI())           # built with an ip already

    async def check(a):
        with pytest.raises(FlightError, match="already connected"):
            a.set_target_ip("9.9.9.9")
    run_connected(a, check)


def test_connect_is_idempotent_single_handshake():
    """Preflight P4 connects and leaves the link up; the agent's later
    connect() (agent.py) must be a NO-OP, not a second handshake — the
    S9-deferred connect-before-stream-start ordering relies on it."""
    a, api = make_adapter(FakeDroneAPI())

    async def check(a):
        await a.connect()                         # second call — must no-op
        assert [c[0] for c in api.calls].count("connect") == 1
        assert a._connected is True
    run_connected(a, check)


def test_degraded_connect_re_handshakes():
    """The idempotent guard EXCLUDES a degraded adapter: safe-down then
    re-connect() must clear the latch (the _gate_not_degraded contract)."""
    a, api = make_adapter(FakeDroneAPI())

    async def check(a):
        a.degraded = True                         # simulate a post-timeout latch
        await a.connect()                         # must actually reconnect
        assert [c[0] for c in api.calls].count("connect") == 2
        assert a.degraded is False
    run_connected(a, check)


# ============================================================
# connect()
# ============================================================
def test_connect_enables_battery_failsafe_always_and_polls():
    a, api = make_adapter(FakeDroneAPI(battery_pct=88))

    async def check(a):
        assert api.battery_failsafe_enabled is True
        assert "enable_battery_failsafe" in [c[0] for c in api.calls]
        assert a._connected is True
        assert a.degraded is False
    run_connected(a, check)


def test_connect_retries_once_via_robust_connect():
    api = FakeDroneAPI(fail_on={"connect": DroneConnectionError})
    a = PyhulaxAdapter("alpha", ip="1.2.3.4", api=api)

    async def check(a):
        names = [c[0] for c in api.calls]
        assert names.count("connect") == 1
        assert "robust_connect" in names      # the one audited retry
        assert a._connected is True
    run_connected(a, check)


def test_connect_fails_when_retry_also_fails():
    api = FakeDroneAPI(fail_on={"connect": DroneConnectionError,
                                "robust_connect": NotReady})
    a = PyhulaxAdapter("alpha", ip="1.2.3.4", api=api)
    try:
        with pytest.raises(FlightError) as ei:
            asyncio.run(a.connect())
        assert "NotReady" in str(ei.value)
        assert a._connected is False
    finally:
        asyncio.run(a.disconnect())


def test_connect_fails_loud_when_poller_dies_immediately():
    api = FakeDroneAPI(fail_on={"get_battery": DroneConnectionError})
    a = PyhulaxAdapter("alpha", ip="1.2.3.4", api=api)
    try:
        with pytest.raises(FlightError, match="poller"):
            asyncio.run(a.connect())
        assert a._connected is False
    finally:
        asyncio.run(a.disconnect())


# ============================================================
# telemetry mapping
# ============================================================
def test_telemetry_maps_units_and_position_none():
    a, _ = make_adapter(FakeDroneAPI(battery_pct=88, altitude_cm=150.0,
                                     yaw_deg=-30.0, is_flying=True))

    async def check(a):
        t = a.telemetry()
        assert t.battery_pct == 88
        assert t.altitude_m == pytest.approx(1.5)        # cm -> m at the boundary
        assert t.yaw_deg == pytest.approx(-30.0)
        assert t.is_flying is True
        assert t.position_m is None                      # no closed-loop position
        assert t.position_quality is PositionQuality.NONE
    run_connected(a, check)


# ============================================================
# the executor choke point: deadline + degrade latch + mapping
# ============================================================
def test_blocking_command_times_out_and_latches_degraded():
    a, _ = make_adapter(FakeDroneAPI(block_s={"takeoff": 0.5}))

    async def check(a):
        with pytest.raises(FlightTimeout):
            await a.takeoff(timeout_s=0.1)               # blocks 0.5 s > 0.1 s
        assert a.degraded is True
        # the degrade latch refuses the next command BEFORE touching the pool
        with pytest.raises(FlightError, match="degraded"):
            await a.move(Direction.FORWARD, 100, timeout_s=5.0)
    run_connected(a, check)


def test_sdk_command_rejected_maps_to_flight_error_no_degrade():
    a, _ = make_adapter(FakeDroneAPI(fail_on={"takeoff": CommandRejected}))

    async def check(a):
        with pytest.raises(FlightError) as ei:
            await a.takeoff(timeout_s=5.0)
        assert not isinstance(ei.value, FlightTimeout)
        assert "CommandRejected" in str(ei.value)
        assert a.degraded is False
    run_connected(a, check)


def test_sdk_command_timeout_name_maps_to_flighttimeout():
    a, _ = make_adapter(FakeDroneAPI(fail_on={"move": CommandTimeout}))

    async def check(a):
        await a.takeoff(timeout_s=5.0)                   # airborne first
        with pytest.raises(FlightTimeout):
            await a.move(Direction.FORWARD, 100, timeout_s=5.0)
        assert a.degraded is True
    run_connected(a, check)


# ============================================================
# command gates + happy-path passthrough
# ============================================================
def test_move_before_takeoff_refused():
    a, _ = make_adapter()

    async def check(a):
        with pytest.raises(FlightError, match="not flying"):
            await a.move(Direction.FORWARD, 100)
    run_connected(a, check)


def test_command_passthrough_to_sdk():
    a, api = make_adapter()

    async def check(a):
        await a.takeoff(height_cm=80)
        await a.move(Direction.FORWARD, 100)
        await a.rotate(90)
        await a.hover(0.0)
        await a.set_led(1, 2, 3)
        await a.land()
        names = [c[0] for c in api.calls]
        for cmd in ("takeoff", "move", "rotate", "hover", "set_led", "land"):
            assert cmd in names
        assert a._airborne is False                      # landed
    run_connected(a, check)


def test_stale_telemetry_aborts_command():
    a, _ = make_adapter(FakeDroneAPI(), fresh_s=2.0)

    async def check(a):
        await a.takeoff()
        # Stop the poller and backdate the stamp so nothing re-freshens it.
        a._poll_stop.set()
        if a._poll_thread is not None:
            a._poll_thread.join(timeout=1.0)
        with a._state.lock:
            a._state.ts = time.monotonic() - 10.0
        with pytest.raises(FlightError, match="STALE"):
            await a.move(Direction.FORWARD, 100)
    run_connected(a, check)


# ============================================================
# never-raise safe-down paths
# ============================================================
def test_emergency_land_never_raises_even_if_land_fails():
    a, _ = make_adapter(FakeDroneAPI(fail_on={"land": CommandRejected}))

    async def check(a):
        await a.emergency_land()                         # must NOT raise
        assert a._airborne is False
    run_connected(a, check)


def test_disconnect_never_raises_even_if_disconnect_fails():
    api = FakeDroneAPI(fail_on={"disconnect": DroneConnectionError})
    a = PyhulaxAdapter("alpha", ip="1.2.3.4", api=api)
    asyncio.run(a.connect())
    asyncio.run(a.disconnect())                          # must NOT raise
    assert a._connected is False


# ============================================================
# main._build_adapter bench wiring (BenchAdapter(PyhulaxAdapter))
# ============================================================
def bench_config(drones=None):
    return {
        "profile": "bench",
        "flight_backend": "bench",
        "frame_backend": "pyhulax",
        "video_channel_order": "rgb",
        "detector": {"backend": "none"},
        "drones": drones or [
            {"id": "alpha", "plane_id": 1, "led_rgb": [255, 0, 0],
             "altitude_band_m": 1.2, "phases": ["takeoff_demo"]}],
    }


def test_build_adapter_bench_wraps_pyhulax_and_refuses_flight(write_config):
    from finals.config import load_config
    from finals.flight.adapter import BenchAdapter
    from finals.main import _build_adapter

    cfg = load_config(write_config(bench_config()))
    a = _build_adapter(cfg, cfg.drones[0])
    assert isinstance(a, BenchAdapter)
    assert isinstance(a.inner, PyhulaxAdapter)
    assert a.drone_id == "alpha"
    # bench refuses flight commands...
    with pytest.raises(FlightError, match="bench"):
        asyncio.run(a.takeoff())
    # ...but connect DELEGATES to the inner PyhulaxAdapter, which refuses with
    # no IP set (discovery -> ip is S10) — proving the wrap reached the leaf.
    with pytest.raises(FlightError, match="no target IP"):
        asyncio.run(a.connect())
