"""finals.flight.sitl_adapter — the PURE parts, tested WITHOUT mavsdk.

The frame math is the part of the SITL backend a sign error would fly
mirror-imaged (the replay-plot bug class SIM-2 exists to catch visually);
here it is pinned BEFORE any flight:

- hypothesis property (the SIM-1 handover gate): for arbitrary yaw and EVERY
  Direction, _body_offset_to_ned's (dN, dE) equals DeadReckoner's documented
  deltas under psi_NED = -yaw_deg — cross-implementation agreement, the same
  pattern simulation.md Tier 0 prescribes.
- a hand-computed grid (yaw 0 / 30 / 90 / -120), values worked on paper from
  the dead_reckon.py convention before trusting any trig.

Everything in this file runs on the Windows dev venv where mavsdk is NOT
installed — that the module imports and constructs at all is itself the
method-local-import contract under test.

Angle comparisons go through angular distance (normalize_yaw_deg(a - b)),
never raw equality: both sides are normalized into (-180, 180] and values at
the +/-180 seam are the same heading with different signs.
"""
from __future__ import annotations

import math

import pytest
from hypothesis import given, strategies as st

from finals.config import load_config, resolve_sitl_endpoint
from finals.errors import ConfigError, FlightError
from finals.flight.dead_reckon import DeadReckoner, DRPose, normalize_yaw_deg
from finals.flight.sitl_adapter import (DEFAULT_GRPC_PORT, MavsdkSitlAdapter,
                                        _body_offset_to_ned,
                                        _rotate_target_psi, _yaw_error_deg)
from finals.types import Direction, Move, Rotate

# Yaw bounded to +/-1e4 deg: plenty of wraparound coverage while keeping
# math.radians' argument-reduction error orders below the 1e-9 tolerance
# (radians(1e6) alone loses ~1e-8 of precision and would test float noise,
# not the frame math).
finite_yaw = st.floats(min_value=-1e4, max_value=1e4,
                       allow_nan=False, allow_infinity=False)
distances_cm = st.integers(min_value=1, max_value=100_000)


def angles_close(a_deg: float, b_deg: float, tol: float = 1e-9) -> bool:
    return abs(normalize_yaw_deg(a_deg - b_deg)) <= tol


def dr_move_delta(yaw_ccw_deg: float, direction: Direction,
                  distance_cm: int):
    """Oracle: DeadReckoner's (dN, dE, dAlt) for one Move at a fixed yaw."""
    dr = DeadReckoner(DRPose(0.0, 0.0, 0.0, yaw_ccw_deg))
    dr.note_action_complete(Move(direction=direction, distance_cm=distance_cm))
    p = dr.pose
    return p.north_m, p.east_m, p.alt_m


# ============================================================
# The property gate: _body_offset_to_ned == DeadReckoner under psi = -yaw
# ============================================================
@given(yaw_ccw_deg=finite_yaw, distance_cm=distances_cm,
       direction=st.sampled_from(list(Direction)))
def test_body_offset_matches_dead_reckoner(yaw_ccw_deg, distance_cm, direction):
    dn_m, de_m, dd_m = _body_offset_to_ned(
        direction, distance_cm, -yaw_ccw_deg)        # psi_NED = -yaw_deg
    exp_n, exp_e, exp_alt = dr_move_delta(yaw_ccw_deg, direction, distance_cm)
    assert math.isclose(dn_m, exp_n, rel_tol=1e-9, abs_tol=1e-9)
    assert math.isclose(de_m, exp_e, rel_tol=1e-9, abs_tol=1e-9)
    # NED down is the NEGATION of the up-positive alt delta.
    assert math.isclose(-dd_m, exp_alt, rel_tol=1e-9, abs_tol=1e-9)


@given(yaw_ccw_deg=finite_yaw, distance_cm=distances_cm)
def test_up_down_never_translate(yaw_ccw_deg, distance_cm):
    for direction, sign in ((Direction.UP, -1.0), (Direction.DOWN, 1.0)):
        dn_m, de_m, dd_m = _body_offset_to_ned(
            direction, distance_cm, -yaw_ccw_deg)
        assert (dn_m, de_m) == (0.0, 0.0)
        assert math.isclose(dd_m, sign * distance_cm / 100.0, rel_tol=1e-12)


# ============================================================
# Hand-computed grid (paper values from the dead_reckon.py convention)
# ============================================================
SQRT3_2 = math.sqrt(3.0) / 2.0       # cos(30 deg) = 0.8660254037844387

# (yaw_ccw_deg, direction, expected (dN, dE, dDown)) for distance 100 cm.
HAND_GRID = [
    (0.0,    Direction.FORWARD, (1.0, 0.0, 0.0)),
    (0.0,    Direction.RIGHT,   (0.0, 1.0, 0.0)),
    (0.0,    Direction.UP,      (0.0, 0.0, -1.0)),
    # yaw +30 CCW -> psi = -30: FORWARD (cos30, -sin30); RIGHT (sin30, cos30)
    (30.0,   Direction.FORWARD, (SQRT3_2, -0.5, 0.0)),
    (30.0,   Direction.RIGHT,   (0.5, SQRT3_2, 0.0)),
    # yaw +90 (facing WEST per dead_reckon.py spot check): FORWARD (0, -1),
    # body-right points north (+1, 0)
    (90.0,   Direction.FORWARD, (0.0, -1.0, 0.0)),
    (90.0,   Direction.RIGHT,   (1.0, 0.0, 0.0)),
    # yaw -120 -> psi = +120: FORWARD (cos(-120), -sin(-120)) = (-0.5, +sin60)
    (-120.0, Direction.FORWARD, (-0.5, SQRT3_2, 0.0)),
    (-120.0, Direction.RIGHT,   (-SQRT3_2, -0.5, 0.0)),
    (-120.0, Direction.DOWN,    (0.0, 0.0, 1.0)),
]


@pytest.mark.parametrize("yaw_ccw_deg, direction, expected", HAND_GRID)
def test_hand_grid(yaw_ccw_deg, direction, expected):
    got = _body_offset_to_ned(direction, 100, -yaw_ccw_deg)
    for g, e in zip(got, expected):
        assert g == pytest.approx(e, abs=1e-12)


@pytest.mark.parametrize("direction", [Direction.BACK, Direction.LEFT])
def test_back_left_are_exact_negations(direction):
    fwd = {Direction.BACK: Direction.FORWARD,
           Direction.LEFT: Direction.RIGHT}[direction]
    for psi in (0.0, 17.0, -133.0, 90.0):
        pos = _body_offset_to_ned(fwd, 250, psi)
        neg = _body_offset_to_ned(direction, 250, psi)
        assert neg == tuple(-v for v in pos)


# ============================================================
# Rotation target + yaw error
# ============================================================
@given(yaw_ccw_deg=finite_yaw, angle_ccw_deg=finite_yaw)
def test_rotate_target_matches_dead_reckoner(yaw_ccw_deg, angle_ccw_deg):
    """target_psi after a CCW+ rotation == the NEGATION of DeadReckoner's
    yaw after the same Rotate (psi_NED = -yaw_deg, both normalized)."""
    dr = DeadReckoner(DRPose(0.0, 0.0, 0.0, yaw_ccw_deg))
    dr.note_action_complete(Rotate(angle_deg=angle_ccw_deg))
    expected_psi = normalize_yaw_deg(-dr.pose.yaw_deg)
    got_psi = _rotate_target_psi(normalize_yaw_deg(-yaw_ccw_deg),
                                 angle_ccw_deg)
    assert angles_close(got_psi, expected_psi)


@pytest.mark.parametrize("cur_psi, angle_ccw, expected_psi", [
    (0.0, 90.0, -90.0),       # CCW+ contract DECREASES NED psi
    (0.0, -90.0, 90.0),
    (170.0, -30.0, -160.0),   # wraparound: 200 -> -160
    (-170.0, 30.0, 160.0),    # wraparound the other way: -200 -> 160
    (0.0, 180.0, 180.0),      # -180 normalizes to +180 (single representation)
])
def test_rotate_target_hand_cases(cur_psi, angle_ccw, expected_psi):
    assert _rotate_target_psi(cur_psi, angle_ccw) == pytest.approx(
        expected_psi, abs=1e-12)


def test_yaw_error_is_shortest_signed():
    assert _yaw_error_deg(10.0, 350.0) == pytest.approx(20.0)
    assert _yaw_error_deg(-170.0, 170.0) == pytest.approx(20.0)
    assert _yaw_error_deg(170.0, -170.0) == pytest.approx(-20.0)
    assert _yaw_error_deg(90.0, 90.0) == 0.0


# ============================================================
# Loud refusals in the pure layer
# ============================================================
@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_yaw_raises(bad):
    with pytest.raises(ValueError, match="finite"):
        _body_offset_to_ned(Direction.FORWARD, 100, bad)
    with pytest.raises(ValueError, match="finite"):
        _rotate_target_psi(bad, 90.0)
    with pytest.raises(ValueError, match="finite"):
        _rotate_target_psi(0.0, bad)


@pytest.mark.parametrize("bad_cm", [0, -100, float("nan")])
def test_bad_distance_raises(bad_cm):
    with pytest.raises(ValueError):
        _body_offset_to_ned(Direction.FORWARD, bad_cm, 0.0)


def test_unknown_direction_raises():
    with pytest.raises(ValueError, match="Direction"):
        _body_offset_to_ned("FORWARD", 100, 0.0)    # type: ignore[arg-type]


# ============================================================
# Constructor + telemetry gates (no I/O, no mavsdk import)
# ============================================================
def make_adapter(**kw):
    args = dict(sitl_address="udpin://0.0.0.0:14540",
                grpc_port=DEFAULT_GRPC_PORT)
    args.update(kw)
    return MavsdkSitlAdapter("alpha", **args)


def test_constructor_holds_endpoint_without_mavsdk():
    a = make_adapter(grpc_port=50052)
    assert a.drone_id == "alpha"
    assert a._grpc_port == 50052
    assert a._sitl_address == "udpin://0.0.0.0:14540"
    assert a.degraded is False


@pytest.mark.parametrize("kw, match", [
    (dict(sitl_address=""), "sitl_address"),
    (dict(sitl_address=14540), "sitl_address"),
    (dict(grpc_port=80), "grpc_port"),
    (dict(grpc_port=70000), "grpc_port"),
    (dict(grpc_port=True), "grpc_port"),
    (dict(grpc_port="50051"), "grpc_port"),
    (dict(arrival_m=0.0), "arrival_m"),
    (dict(fresh_s=float("inf")), "fresh_s"),
    (dict(yaw_tol_deg=-1.0), "yaw_tol_deg"),
    (dict(poll_period_s=0), "poll_period_s"),
])
def test_constructor_rejects_bad_args(kw, match):
    with pytest.raises(ValueError, match=match):
        make_adapter(**kw)


def test_constructor_rejects_empty_drone_id():
    with pytest.raises(ValueError, match="drone_id"):
        MavsdkSitlAdapter("", sitl_address="udpin://0.0.0.0:14540")


def test_telemetry_before_connect_raises_flight_error():
    with pytest.raises(FlightError, match="never connected"):
        make_adapter().telemetry()


def test_snapshot_negates_ned_into_contract_frame():
    """The NED->contract negations (yaw = -psi, alt = -down) are the two
    signs nothing else in the suite pins — a flip here would pass 470+ tests
    and only surface at the SIM-1 visual gate (review finding 14)."""
    import time as _time

    a = make_adapter()
    a._ever_connected = True
    a._connected = True
    st = a._state
    st.north_m, st.east_m, st.down_m = 3.0, -2.0, -1.5     # 1.5 m UP
    st.psi_deg = -30.0          # NED CW+ -30 -> contract CCW+ +30
    st.pos_ts = _time.monotonic()
    st.psi_ts = st.pos_ts - 0.01                           # older stamp
    st.in_air = True
    st.battery_pct = 87.0
    t = a.telemetry()
    assert t.position_m == (3.0, -2.0, 1.5)
    assert t.altitude_m == pytest.approx(1.5)
    assert t.yaw_deg == pytest.approx(30.0)
    assert t.is_flying is True
    assert t.battery_pct == 87.0
    from finals.types import PositionQuality
    assert t.position_quality is PositionQuality.MEASURED
    assert t.ts == st.psi_ts            # min of the two stamps — honest age


# ============================================================
# Endpoint resolution + factory wiring (still no mavsdk)
# ============================================================
def sitl_config(drones):
    return {
        "profile": "sitl",
        "flight_backend": "mavsdk_sitl",
        "frame_backend": "gazebo",
        "sitl_address": "udpin://0.0.0.0:14540",
        "detector": {"backend": "none"},
        "drones": drones,
    }


THREE_DRONES = [
    {"id": "alpha", "phases": ["takeoff_demo"], "altitude_band_m": 1.2,
     "sitl_address": "udpin://0.0.0.0:14540", "mavsdk_grpc_port": 50051},
    {"id": "bravo", "phases": ["takeoff_demo"], "altitude_band_m": 1.7,
     "sitl_address": "udpin://0.0.0.0:14541", "mavsdk_grpc_port": 50052},
    {"id": "charlie", "phases": ["takeoff_demo"], "altitude_band_m": 2.2,
     "sitl_address": "udpin://0.0.0.0:14542", "mavsdk_grpc_port": 50053},
]


def test_single_drone_falls_back_to_top_level(write_config):
    cfg = load_config(write_config(
        sitl_config([{"id": "alpha", "phases": ["takeoff_demo"]}])))
    address, port = resolve_sitl_endpoint(cfg, cfg.drones[0])
    assert address == "udpin://0.0.0.0:14540"
    assert port == 50051
    # The config fallback and the adapter default are the same number by
    # CONTRACT — this assertion is what keeps the two literals from drifting.
    assert port == DEFAULT_GRPC_PORT


def test_sitl_top_level_address_validated(write_config):
    cfg_dict = sitl_config([{"id": "alpha", "phases": ["takeoff_demo"]}])
    cfg_dict["sitl_address"] = ""
    with pytest.raises(ConfigError, match="sitl_address"):
        load_config(write_config(cfg_dict))


def test_per_drone_endpoints_win(write_config):
    cfg = load_config(write_config(sitl_config(THREE_DRONES)))
    got = [resolve_sitl_endpoint(cfg, d) for d in cfg.drones]
    assert got == [("udpin://0.0.0.0:14540", 50051),
                   ("udpin://0.0.0.0:14541", 50052),
                   ("udpin://0.0.0.0:14542", 50053)]


def test_build_adapter_constructs_sitl_backend(write_config):
    from finals.main import _build_adapter
    cfg = load_config(write_config(sitl_config(THREE_DRONES)))
    adapters = [_build_adapter(cfg, d) for d in cfg.drones]
    assert all(isinstance(a, MavsdkSitlAdapter) for a in adapters)
    assert [(a._sitl_address, a._grpc_port) for a in adapters] == [
        ("udpin://0.0.0.0:14540", 50051),
        ("udpin://0.0.0.0:14541", 50052),
        ("udpin://0.0.0.0:14542", 50053)]
    assert [a.drone_id for a in adapters] == ["alpha", "bravo", "charlie"]


def test_build_adapter_single_drone_fallback(write_config):
    from finals.main import _build_adapter
    cfg = load_config(write_config(
        sitl_config([{"id": "alpha", "phases": ["takeoff_demo"]}])))
    a = _build_adapter(cfg, cfg.drones[0])
    assert isinstance(a, MavsdkSitlAdapter)
    assert (a._sitl_address, a._grpc_port) == ("udpin://0.0.0.0:14540", 50051)


# ============================================================
# Multi-drone sitl validation (the schema gate)
# ============================================================
def test_multi_drone_requires_endpoints(write_config):
    drones = [dict(d) for d in THREE_DRONES]
    del drones[1]["mavsdk_grpc_port"]
    with pytest.raises(ConfigError, match="bravo"):
        load_config(write_config(sitl_config(drones)))


def test_multi_drone_duplicate_ports_rejected(write_config):
    drones = [dict(d) for d in THREE_DRONES]
    drones[1]["mavsdk_grpc_port"] = 50051            # collides with alpha
    with pytest.raises(ConfigError, match="DISTINCT"):
        load_config(write_config(sitl_config(drones)))


def test_multi_drone_duplicate_addresses_rejected(write_config):
    drones = [dict(d) for d in THREE_DRONES]
    drones[2]["sitl_address"] = "udpin://0.0.0.0:14540"
    with pytest.raises(ConfigError, match="DISTINCT"):
        load_config(write_config(sitl_config(drones)))


def test_multi_drone_requires_distinct_bands(write_config):
    drones = [dict(d) for d in THREE_DRONES]
    drones[1]["altitude_band_m"] = 1.2               # collides with alpha
    with pytest.raises(ConfigError, match="altitude_band_m"):
        load_config(write_config(sitl_config(drones)))
    del drones[1]["altitude_band_m"]                 # missing entirely
    with pytest.raises(ConfigError, match="altitude_band_m"):
        load_config(write_config(sitl_config(drones)))


def test_bad_grpc_port_rejected(write_config):
    drones = [{"id": "alpha", "phases": ["takeoff_demo"],
               "mavsdk_grpc_port": 80}]
    with pytest.raises(ConfigError, match="mavsdk_grpc_port"):
        load_config(write_config(sitl_config(drones)))


def test_bad_sitl_address_rejected(write_config):
    drones = [{"id": "alpha", "phases": ["takeoff_demo"],
               "sitl_address": ""}]
    with pytest.raises(ConfigError, match="sitl_address"):
        load_config(write_config(sitl_config(drones)))


def test_single_drone_may_omit_endpoints(write_config):
    cfg = load_config(write_config(
        sitl_config([{"id": "alpha", "phases": ["takeoff_demo"]}])))
    assert cfg.drones[0].sitl_address is None
    assert cfg.drones[0].mavsdk_grpc_port is None
