"""Step 0 shared-contract pins — the seams the six Roboverse-landing build
sessions code against. Each contract is pinned HERE so a parallel session can
rely on it without re-deriving it; a session that later hardens its own
dataclass/validator extends these, never weakens them. Pure (no cv2/numpy)."""
from __future__ import annotations

import dataclasses

import pytest

from finals.config import VALID_DEPTH_BACKENDS, load_config
from finals.errors import ConfigError
from finals.mission.planning.types import ArenaMap, Gate, Marker
from finals.types import FrameStamped, Sighting


# ======================= types.py: pad source + depth seam ===================
def test_sighting_accepts_pad_source():
    """PAD-DETECT emits a colour-blob sighting: source 'pad', marker_id None."""
    s = Sighting(drone_id="alpha", ts=1.0, source="pad", class_name="pad",
                 marker_id=None, bbox_xyxy=(0.0, 0.0, 20.0, 20.0),
                 confidence=1.0, frame_shape=(480, 640))
    assert s.source == "pad" and s.marker_id is None


def test_frame_stamped_depth_seam_defaults_none():
    """SENSE-IR depth seam degrades absent: monocular frames carry no depth."""
    fr = FrameStamped(image=None, ts=1.0, frame_number=1, source_id="alpha")
    assert fr.depth is None
    assert "depth" in {f.name for f in dataclasses.fields(FrameStamped)}


# ======================= arena: markers (NAV-FIX anchors) ====================
# Organizer frame: north = the long (~11.3 m) axis = y, east = short (~5.3 m)
# axis = x. point_m is [north_m, east_m] = [y, x]. These mirror the real field
# beacons (11 -> x1.35/y4.40, 51 -> x4.40/y4.40).
_BOUNDS = [0.0, 0.0, 11.3, 5.3]
_ORIGIN = [0.5, 0.5]


def _arena(**extra) -> ArenaMap:
    raw = {"bounds_m": _BOUNDS, "c2_origin_m": _ORIGIN, "c2_heading_deg": 0.0}
    raw.update(extra)
    return ArenaMap.from_dict(raw, name="step0")


def test_arena_parses_markers_with_known_coords():
    a = _arena(markers=[{"id": 11, "point_m": [4.40, 1.35]},
                        {"id": 51, "point_m": [4.40, 4.40]}])
    assert isinstance(a.markers[0], Marker)
    assert {m.id for m in a.markers} == {11, 51}
    assert a.markers[0].point_m == (4.40, 1.35)


def test_arena_marker_outside_bounds_is_loud():
    with pytest.raises(ConfigError, match="OUTSIDE bounds"):
        _arena(markers=[{"id": 11, "point_m": [99.0, 1.0]}])


def test_arena_duplicate_marker_id_is_loud():
    with pytest.raises(ConfigError, match="duplicate marker id"):
        _arena(markers=[{"id": 11, "point_m": [1.0, 1.0]},
                        {"id": 11, "point_m": [2.0, 2.0]}])


def test_arena_marker_id_must_be_int():
    with pytest.raises(ConfigError, match="must be an int"):
        _arena(markers=[{"id": "11", "point_m": [1.0, 1.0]}])


# ======================= arena: gates (NAV-ARCH openings) ====================
# NAV-ARCH cross-check: a gate span must touch a keep-out (the arch posts). The
# arch posts straddle the span line; this _ARCH keep-out covers the span's east
# extent so the gate parses. (The parse-shape tests only need ONE touching
# keep-out; the through-gate PLANNING is exercised in test_visibility_graph.)
_ARCH = [{"id": "post", "polygon_m": [[1.8, 0.5], [2.2, 0.5],
                                      [2.2, 2.5], [1.8, 2.5]]}]


def test_arena_parses_gate_with_clearance():
    a = _arena(keep_out=_ARCH,
               gates=[{"id": "arch1", "span_m": [[2.0, 1.0], [2.0, 2.0]],
                       "clearance_m": 0.9}])
    assert isinstance(a.gates[0], Gate)
    assert a.gates[0].id == "arch1"
    assert a.gates[0].clearance_m == pytest.approx(0.9)


def test_arena_gate_clearance_optional_defaults_zero():
    a = _arena(keep_out=_ARCH,
               gates=[{"id": "g", "span_m": [[2.0, 1.0], [2.0, 2.0]]}])
    assert a.gates[0].clearance_m == 0.0


def test_arena_gate_needs_two_endpoints():
    with pytest.raises(ConfigError, match="span_m"):
        _arena(keep_out=_ARCH, gates=[{"id": "g", "span_m": [[1.0, 1.0]]}])


def test_arena_duplicate_gate_id_is_loud():
    with pytest.raises(ConfigError, match="duplicate gate id"):
        _arena(keep_out=_ARCH,
               gates=[{"id": "g", "span_m": [[2.0, 1.0], [2.0, 2.0]]},
                      {"id": "g", "span_m": [[2.0, 1.0], [2.0, 2.0]]}])


def test_arena_without_markers_or_gates_defaults_empty():
    a = _arena()
    assert a.markers == () and a.gates == ()


def test_arena_unknown_key_still_rejected():
    # 'beacons' is a typo for 'markers' — the loud loader must still catch it.
    with pytest.raises(ConfigError, match="unknown key"):
        _arena(beacons=[])


# ======================= config: reserved knobs ==============================
def test_config_reserved_knobs_default(write_config, minimal_mock_config):
    cfg = load_config(write_config(minimal_mock_config))
    assert cfg.marker_dict == "DICT_7X7_1000"
    assert cfg.aruco_detector_params is None
    assert cfg.depth_backend == "none"


def test_config_marker_dict_override_accepted(write_config, minimal_mock_config):
    minimal_mock_config["marker_dict"] = "DICT_6X6_250"   # the sim/fixture pin
    cfg = load_config(write_config(minimal_mock_config))
    assert cfg.marker_dict == "DICT_6X6_250"


def test_config_empty_marker_dict_is_loud(write_config, minimal_mock_config):
    minimal_mock_config["marker_dict"] = ""
    with pytest.raises(ConfigError, match="marker_dict"):
        load_config(write_config(minimal_mock_config))


def test_config_bad_aruco_params_is_loud(write_config, minimal_mock_config):
    minimal_mock_config["aruco_detector_params"] = [1, 2, 3]   # not an object
    with pytest.raises(ConfigError, match="aruco_detector_params"):
        load_config(write_config(minimal_mock_config))


def test_config_bad_depth_backend_is_loud(write_config, minimal_mock_config):
    minimal_mock_config["depth_backend"] = "realsense"   # not yet wired
    with pytest.raises(ConfigError, match="depth_backend"):
        load_config(write_config(minimal_mock_config))
    assert "none" in VALID_DEPTH_BACKENDS
