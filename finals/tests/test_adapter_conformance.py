"""FlightAdapter conformance — the PX4->HULA switchover seam, pinned ONCE.

Today the flight backends are tested per-adapter (test_mock_adapter /
test_sitl_adapter / test_pyhulax_adapter). Those prove each backend in
isolation; NONE asserts that the THREE present the SAME contract to mission
logic, which is exactly the switchover risk: code that maps cleanly onto
MockAdapter / MavsdkSitlAdapter (PX4 SITL) but quietly diverges on
PyhulaxAdapter (the real HULA drones) — and surfaces only on hardware, in the
2-hour onsite window. This file is that single cross-backend contract.

SCOPE — the FLIGHT seam (the FlightAdapter ABC). Three tiers, because the
backends are not equally drivable on the bare dev venv:

  1. STRUCTURAL (all 4 adapters, no SDK): each is a concrete FlightAdapter and
     every contract method's signature (param names + defaults) MATCHES the ABC.
     Catches a default drifting in one backend (e.g. takeoff height) that mission
     logic would silently inherit differently per drone.

  2. PRE-SDK GATES (all 3 backends, no SDK): the gates that fire BEFORE any SDK
     import are identical — command-before-connect raises FlightError; telemetry()
     before connect raises "never connected"; emergency_land()/disconnect() never
     raise even never-connected (the one sanctioned swallow + the never-raise
     teardown). These run for SITL too because its gates short-circuit before the
     method-local `import mavsdk`.

  3. BEHAVIORAL (the SDK-drivable pair — MockAdapter + PyhulaxAdapter/FakeDroneAPI):
     the full complete-or-raise contract — happy path completes, move-before-
     takeoff / takeoff-while-flying / impossible-argument all raise FlightError,
     a command that overruns its deadline raises FlightTimeout AND latches
     `degraded`, and the safe-down path (land/emergency_land) stays allowed while
     degraded. MavsdkSitlAdapter is DELIBERATELY EXCLUDED from this tier: its
     command path needs mavsdk installed AND a live PX4 (that is the SIM
     integration tier, run on the VM — see sim/run_*.sh), not a unit check.
     test_sitl_is_the_known_behavioral_exclusion documents that on purpose, so a
     future fake-mavsdk gets wired in here rather than silently skipped.

  4. DIVERGENCE GUARDS (per backend): the known seam hazards are surfaced
     HONESTLY, never faked —
       - position_quality: mock DEAD_RECKONING, SITL MEASURED, pyhulax NONE with
         position_m is None. The HULA reality is PositionQuality.NONE; this is the
         single most load-bearing switchover fact (the landing-nav is built
         position-blind precisely because of it).
       - the "unit hop": pyhulax passes distance_cm STRAIGHT to the SDK (no hidden
         rescale) — so the onsite unit-verification gate ("unit hop", adapter.py
         docstring) is a ONE-LINE adapter-boundary fix, provable here to touch
         nothing else.
       - rotate sign: +deg == CCW passes through to the SDK unflipped.

OUT OF SCOPE (covered elsewhere, NOT faked here): the VISION-seam switchover
hazards — `.to_rgb()` channel order and `camera_hfov_deg == null -> bearing_deg
null` — are not FlightAdapter methods; they live behind the frame backend
(video_channel_order config + finals/vision/) and are pinned by config
validation + the vision tests. Asserting them here would be theatre.

Everything runs on the bare Windows/VM dev venv: mavsdk and pyhulax are both
absent (their imports are method-local), and FakeDroneAPI stands in for the
HULA SDK. Plain asyncio.run (no pytest-asyncio), mirroring the per-adapter
tests.

Session: WS-3.
"""
from __future__ import annotations

import asyncio
import inspect

import pytest

from finals.errors import FlightError, FlightTimeout
from finals.flight.adapter import BenchAdapter, FlightAdapter
from finals.flight.mock_adapter import MockAdapter
from finals.flight.pyhulax_adapter import FakeDroneAPI, PyhulaxAdapter
from finals.flight.sitl_adapter import DEFAULT_GRPC_PORT, MavsdkSitlAdapter
from finals.types import Direction, PositionQuality, Telemetry


# ============================================================
# Fresh-adapter factories (NOT connected). One per backend; the ctor args
# differ, so the seam is the FlightAdapter surface, not construction.
# ============================================================
def make_mock() -> MockAdapter:
    return MockAdapter("alpha")


def make_sitl() -> MavsdkSitlAdapter:
    return MavsdkSitlAdapter("alpha", sitl_address="udpin://0.0.0.0:14540",
                             grpc_port=DEFAULT_GRPC_PORT)


def make_pyhulax() -> PyhulaxAdapter:
    # A FakeDroneAPI is injected so the gate tests never reach the real SDK
    # factory; the pre-SDK gates short-circuit before it is touched anyway.
    return PyhulaxAdapter("alpha", ip="192.168.1.50", api=FakeDroneAPI())


#: (name, factory) for every backend that can be CONSTRUCTED on the dev venv.
ALL_BACKENDS = [
    ("mock", make_mock),
    ("sitl", make_sitl),
    ("pyhulax", make_pyhulax),
]

#: The contract surface every backend shares (ABC public methods).
CONTRACT_METHODS = [
    "connect", "disconnect", "takeoff", "land", "move", "rotate", "hover",
    "telemetry", "emergency_land", "set_led",
]


# ============================================================
# TIER 1 — Structural conformance (introspection only, no SDK)
# ============================================================
def make_bench() -> BenchAdapter:
    return BenchAdapter(MockAdapter("alpha"))


STRUCTURAL_BACKENDS = ALL_BACKENDS + [("bench", make_bench)]


@pytest.mark.parametrize("name, factory", STRUCTURAL_BACKENDS)
def test_is_concrete_flight_adapter(name, factory):
    """Each backend is a concrete FlightAdapter — no abstractmethod left
    unimplemented (which would make construction itself raise TypeError)."""
    adapter = factory()
    assert isinstance(adapter, FlightAdapter)
    assert not inspect.isabstract(type(adapter))


@pytest.mark.parametrize("name, factory", STRUCTURAL_BACKENDS)
@pytest.mark.parametrize("method", CONTRACT_METHODS)
def test_method_signature_matches_abc(name, factory, method):
    """Param names + defaults match the ABC exactly. A backend that quietly
    changed a default (e.g. takeoff(height_cm=100)) would fly that drone to a
    different altitude than the mission logic and the other two drones intend —
    a switchover bug invisible until hardware. SITL inherits set_led from the
    ABC (documented no-op), so it matches by construction."""
    abc_sig = inspect.signature(getattr(FlightAdapter, method))
    impl_sig = inspect.signature(getattr(type(factory()), method))
    assert impl_sig == abc_sig, (
        f"{name}.{method}{impl_sig} diverges from the FlightAdapter "
        f"contract {abc_sig} — a per-backend signature/default drift")


@pytest.mark.parametrize("method", CONTRACT_METHODS)
def test_async_shape_matches_abc(method):
    """telemetry() is the ONE synchronous method (non-blocking latest-known
    state); every other contract method is a coroutine. A backend that made
    telemetry async (or a command sync) would break the agent's await shape."""
    is_async = asyncio.iscoroutinefunction(getattr(FlightAdapter, method))
    for name, factory in ALL_BACKENDS:
        impl = getattr(type(factory()), method)
        assert asyncio.iscoroutinefunction(impl) is is_async, (
            f"{name}.{method} async-ness diverges from the ABC")
    assert (method == "telemetry") != is_async   # telemetry is the sole sync one


# ============================================================
# TIER 2 — Pre-SDK gate conformance (all 3 backends, no SDK)
# ============================================================
# These gates fire before any method-local SDK import, so SITL runs here too.
FLIGHT_COMMANDS = [
    ("takeoff", lambda a: a.takeoff()),
    ("land", lambda a: a.land()),
    ("move", lambda a: a.move(Direction.FORWARD, 100)),
    ("rotate", lambda a: a.rotate(90.0)),
    ("hover", lambda a: a.hover(1.0)),
]


@pytest.mark.parametrize("name, factory", ALL_BACKENDS)
@pytest.mark.parametrize("cmd, call", FLIGHT_COMMANDS)
def test_command_before_connect_refuses_loud(name, factory, cmd, call):
    """Every flight command before connect() raises FlightError naming the
    drone — never a bare AttributeError on a None SDK handle, never a silent
    no-op. (land is gated too: you cannot safe-down a link you never opened.)"""
    adapter = factory()
    with pytest.raises(FlightError) as ei:
        asyncio.run(call(adapter))
    assert "alpha" in str(ei.value)


@pytest.mark.parametrize("name, factory", ALL_BACKENDS)
def test_telemetry_before_connect_refuses_loud(name, factory):
    """telemetry() before the first connect() raises 'never connected' — a
    fabricated fresh-ts Telemetry would sail through the staleness guards."""
    with pytest.raises(FlightError, match="never connected"):
        factory().telemetry()


@pytest.mark.parametrize("name, factory", ALL_BACKENDS)
def test_emergency_land_never_raises_before_connect(name, factory):
    """emergency_land() is the one sanctioned swallow site — even with nothing
    connected it must return quietly (nothing airborne to command)."""
    asyncio.run(factory().emergency_land())          # must NOT raise


@pytest.mark.parametrize("name, factory", ALL_BACKENDS)
def test_disconnect_never_raises_before_connect(name, factory):
    """disconnect() is never-raise by contract, including on a link that was
    never opened."""
    asyncio.run(factory().disconnect())              # must NOT raise


# ============================================================
# TIER 3 — Behavioral contract (the SDK-drivable pair: mock + pyhulax/fake)
# ============================================================
# Each entry yields (adapter, recorder) where recorder.calls is a list of
# (command_name, kwargs) — the SAME shape for MockAdapter (records on itself)
# and FakeDroneAPI (records the SDK calls the adapter made). A backend that
# can drive a full takeoff->...->land with NO real SDK belongs here.
def drivable_mock(**kw):
    a = MockAdapter("alpha", **kw)
    return a, a                                      # mock records on itself


def drivable_pyhulax(api=None, **kw):
    api = api or FakeDroneAPI()
    return PyhulaxAdapter("alpha", ip="1.2.3.4", api=api, **kw), api


DRIVABLE = [
    ("mock", drivable_mock),
    ("pyhulax", drivable_pyhulax),
]


def run_connected(adapter, check):
    """connect() -> check(adapter) -> disconnect() on ONE event loop, so the
    pyhulax executor pool + poller thread are set up and torn down cleanly
    (mock's connect/disconnect are trivial; the shape is shared)."""
    async def _wrap():
        await adapter.connect()
        try:
            return await check(adapter)
        finally:
            await adapter.disconnect()
    return asyncio.run(_wrap())


def cmd_names(recorder):
    return [c[0] for c in recorder.calls]


@pytest.mark.parametrize("name, build", DRIVABLE)
def test_happy_path_completes(name, build):
    """takeoff -> move -> rotate -> hover -> land all COMPLETE (return without
    raising), every command reaches the SDK in order, and telemetry() is
    readable before and after. NOTE: telemetry().is_flying is deliberately NOT
    asserted here — it is the LATEST-KNOWN polled value, which on PyhulaxAdapter
    lags the command by up to one 2 Hz poll tick (command gating uses the
    adapter's authoritative airborne flag, not this field). Reading
    telemetry().is_flying immediately after takeoff() is therefore a switchover
    race that works in SITL/mock and is flaky on HULA — mission logic relies on
    command COMPLETION, never on the polled flying flag."""
    adapter, recorder = build()

    async def check(a):
        assert isinstance(a.telemetry(), Telemetry)   # readable once connected
        await a.takeoff(height_cm=80)
        await a.move(Direction.FORWARD, 100)
        await a.rotate(90.0)
        await a.hover(0.0)
        await a.land()
        names = cmd_names(recorder)
        assert [c for c in names if c in
                ("takeoff", "move", "rotate", "hover", "land")] == \
            ["takeoff", "move", "rotate", "hover", "land"], \
            f"{name}: commands missing or out of order — got {names}"
        assert isinstance(a.telemetry(), Telemetry)   # still readable after land
    run_connected(adapter, check)


@pytest.mark.parametrize("name, build", DRIVABLE)
def test_move_before_takeoff_refused(name, build):
    adapter, _ = build()

    async def check(a):
        with pytest.raises(FlightError, match="not flying"):
            await a.move(Direction.FORWARD, 100)
    run_connected(adapter, check)


@pytest.mark.parametrize("name, build", DRIVABLE)
def test_takeoff_while_flying_refused(name, build):
    adapter, _ = build()

    async def check(a):
        await a.takeoff()
        with pytest.raises(FlightError, match="already flying"):
            await a.takeoff()
    run_connected(adapter, check)


@pytest.mark.parametrize("name, build", DRIVABLE)
@pytest.mark.parametrize("bad_call", [
    lambda a: a.move(Direction.FORWARD, 0),          # zero distance
    lambda a: a.move(Direction.FORWARD, -50),        # negative distance
    lambda a: a.rotate(float("nan")),                # non-finite yaw
    lambda a: a.hover(-1.0),                          # negative duration
])
def test_impossible_argument_refused_while_flying(name, build, bad_call):
    """Physically impossible magnitudes are REFUSED with FlightError — the
    adapter does NOT clamp or complete a command a real backend would reject
    (that lets a sign-error first surface on hardware). Tested airborne so the
    not-flying gate is already satisfied and the argument gate is what fires."""
    adapter, _ = build()

    async def check(a):
        await a.takeoff()
        with pytest.raises(FlightError):
            await bad_call(a)
    run_connected(adapter, check)


# --- deadline overrun -> FlightTimeout + degrade latch + safe-down still works
def timeout_mock():
    # latency 0.5 s: connect (timeout 10 s) is fine; takeoff(timeout_s=0.1) trips.
    return MockAdapter("alpha", latency_s=0.5)


def timeout_pyhulax():
    # the SDK call blocks 0.5 s in the executor; takeoff(timeout_s=0.1) trips.
    return PyhulaxAdapter("alpha", ip="1.2.3.4",
                          api=FakeDroneAPI(block_s={"takeoff": 0.5}))


@pytest.mark.parametrize("name, builder", [
    ("mock", timeout_mock),
    ("pyhulax", timeout_pyhulax),
])
def test_deadline_overrun_times_out_degrades_and_allows_safe_down(name, builder):
    """A command that overruns its deadline raises FlightTimeout and LATCHES
    `degraded`; the next flight command is then refused BEFORE touching the SDK
    ('degraded'); but the safe-down path (land + emergency_land) stays allowed
    so the drone can always be brought down. Identical contract across backends
    — this is what lets the agent's safe-down logic be backend-blind."""
    adapter = builder()

    async def check(a):
        with pytest.raises(FlightTimeout):
            await a.takeoff(timeout_s=0.1)
        assert a.degraded is True
        with pytest.raises(FlightError, match="degraded"):
            await a.move(Direction.FORWARD, 100, timeout_s=5.0)
        # safe-down is NOT gated by degraded:
        await a.land()                                # must not raise
        await a.emergency_land()                      # must not raise
    run_connected(adapter, check)


def test_sitl_is_the_known_behavioral_exclusion():
    """MavsdkSitlAdapter is intentionally absent from the DRIVABLE behavioral
    tier: its command path needs mavsdk installed AND a live PX4 (the SIM
    integration tier on the VM), so it cannot be unit-driven here. This test
    PINS that exclusion as deliberate — if a fake-mavsdk stub is ever added,
    wire SITL into DRIVABLE and delete this. Failing loudly beats a silent gap."""
    drivable_types = {build()[0].__class__ for _, build in DRIVABLE}
    assert MockAdapter in drivable_types
    assert PyhulaxAdapter in drivable_types
    assert MavsdkSitlAdapter not in drivable_types


# ============================================================
# TIER 4 — Divergence guards (the switchover hazards, surfaced honestly)
# ============================================================
def _mock_telemetry() -> Telemetry:
    """A flown mock reports a dead-reckoned pose."""
    a = MockAdapter("alpha")

    async def _fly():
        await a.connect()
        await a.takeoff(80)
        await a.move(Direction.FORWARD, 100)
        t = a.telemetry()
        await a.disconnect()
        return t
    return asyncio.run(_fly())


def _sitl_telemetry() -> Telemetry:
    """SITL telemetry built from injected stream state — NO mavsdk, NO PX4
    (the same direct-state path test_sitl_adapter uses). MEASURED because PX4
    publishes a fused EKF position."""
    import time as _time
    a = make_sitl()
    a._ever_connected = True
    a._connected = True
    st = a._state
    st.north_m, st.east_m, st.down_m = 3.0, -2.0, -1.5      # 1.5 m UP
    st.psi_deg = -30.0
    st.pos_ts = _time.monotonic()
    st.psi_ts = st.pos_ts
    st.in_air = True
    st.battery_pct = 87.0
    return a.telemetry()


def _pyhulax_telemetry() -> Telemetry:
    a = PyhulaxAdapter("alpha", ip="1.2.3.4",
                       api=FakeDroneAPI(altitude_cm=150.0, yaw_deg=-30.0,
                                        is_flying=True))

    async def _fly():
        await a.connect()
        t = a.telemetry()
        await a.disconnect()
        return t
    return asyncio.run(_fly())


@pytest.mark.parametrize("name, provider, expected_quality, has_position", [
    ("mock", _mock_telemetry, PositionQuality.DEAD_RECKONING, True),
    ("sitl", _sitl_telemetry, PositionQuality.MEASURED, True),
    ("pyhulax", _pyhulax_telemetry, PositionQuality.NONE, False),
])
def test_position_quality_is_honest_per_backend(name, provider, expected_quality,
                                                has_position):
    """The three backends report DIFFERENT, HONEST position quality — none
    fakes a fix it doesn't have. PyhulaxAdapter (the real HULA) is NONE with
    position_m is None: mission logic must already work position-blind, and
    THIS is the assertion that fails loud if a refactor ever makes pyhulax
    pretend it has a measured position."""
    t = provider()
    assert t.position_quality is expected_quality
    if has_position:
        assert t.position_m is not None
    else:
        assert t.position_m is None


def test_pyhulax_move_does_not_silently_rescale_distance():
    """The 'unit hop' hazard (adapter.py docstring): the contract is
    distance_cm, but hula_connection.py shows move(FORWARD, 0.5). The adapter
    passes distance_cm STRAIGHT to the SDK — no hidden scale factor — so the
    onsite unit-verification gate is a ONE-LINE adapter-boundary fix. This pins
    that there is nothing else to find: 137 in, 137 to the SDK."""
    api = FakeDroneAPI()
    a = PyhulaxAdapter("alpha", ip="1.2.3.4", api=api)

    async def check(a):
        await a.takeoff()
        await a.move(Direction.FORWARD, 137)
        move = next(c for c in api.calls if c[0] == "move")
        assert move[1]["distance_cm"] == 137         # not 1.37, not 13700
    run_connected(a, check)


def test_pyhulax_rotate_sign_passes_through_unflipped():
    """+deg == CCW (pyhulax convention) reaches the SDK UNFLIPPED — the sign is
    an onsite-verify gate, never silently inverted at the adapter."""
    api = FakeDroneAPI()
    a = PyhulaxAdapter("alpha", ip="1.2.3.4", api=api)

    async def check(a):
        await a.takeoff()
        await a.rotate(90.0)
        rot = next(c for c in api.calls if c[0] == "rotate")
        assert rot[1]["angle_degrees"] == 90.0
    run_connected(a, check)
