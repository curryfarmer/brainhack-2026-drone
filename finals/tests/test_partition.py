"""finals.mission.planning.partition — WS-6 spatial partition (filled).

region_to_keep_outs turns the OTHER drones' keep-in regions into keep-outs for
`mine`'s planner, so the inflated transit corridors never overlap = plan-time
spatial deconfliction (the only kind a POSITION-BLIND drone can honour). Pins the
conversion + that the produced keep-out actually forces the planner to detour
around a neighbour's territory. Pure — stdlib + pytest only.
"""
from __future__ import annotations

import pytest

from finals.mission.planning.partition import DroneRegion, region_to_keep_outs
from finals.mission.planning.types import ArenaMap
from finals.mission.planning.visibility_graph import plan

_TRI_A = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))
_TRI_B = ((2.0, 0.0), (3.0, 0.0), (3.0, 1.0))


def test_drone_region_is_frozen_and_holds_the_contract_fields():
    r = DroneRegion(drone_id="alpha", keep_in_polygon_m=_TRI_A)
    assert r.drone_id == "alpha"
    assert r.keep_in_polygon_m == _TRI_A
    with pytest.raises(Exception):           # frozen dataclass -> no mutation
        r.drone_id = "bravo"                 # type: ignore[misc]


# ============================================================
# region_to_keep_outs
# ============================================================
def test_other_region_becomes_a_named_keep_out():
    mine = DroneRegion("alpha", _TRI_A)
    other = DroneRegion("bravo", _TRI_B)
    kos = region_to_keep_outs(mine, (other,))
    assert len(kos) == 1
    assert kos[0].id == "region_bravo"
    assert kos[0].polygon_m == _TRI_B


def test_my_own_region_is_skipped():
    mine = DroneRegion("alpha", _TRI_A)
    kos = region_to_keep_outs(mine, (mine, DroneRegion("bravo", _TRI_B)))
    assert [k.id for k in kos] == ["region_bravo"]   # never wall myself in


def test_multiple_others_sorted_by_id():
    mine = DroneRegion("alpha", _TRI_A)
    others = (DroneRegion("charlie", ((5.0, 5.0), (6.0, 5.0), (6.0, 6.0))),
              DroneRegion("bravo", _TRI_B))
    kos = region_to_keep_outs(mine, others)
    assert [k.id for k in kos] == ["region_bravo", "region_charlie"]


def test_non_region_args_fail_loud():
    mine = DroneRegion("alpha", _TRI_A)
    with pytest.raises(ValueError):
        region_to_keep_outs(object(), ())            # type: ignore[arg-type]
    with pytest.raises(ValueError):
        region_to_keep_outs(mine, (object(),))       # type: ignore[arg-type]


def test_degenerate_region_polygon_fails_loud():
    mine = DroneRegion("alpha", _TRI_A)
    # bravo's region is a line (2 distinct vertices) -> the reused arena rule
    # rejects it loudly rather than emitting an empty keep-out.
    bad = DroneRegion("bravo", ((0.0, 0.0), (1.0, 1.0)))
    with pytest.raises(Exception):
        region_to_keep_outs(mine, (bad,))


def test_partition_keep_out_forces_a_detour_around_a_neighbour():
    """End-to-end: bravo's region sits squarely on alpha's straight C2->goal line.
    Converted to a keep-out and planned over, alpha must detour AROUND it."""
    mine = DroneRegion("alpha", ((0.0, -2.0), (0.0, 2.0), (1.0, 2.0), (1.0, -2.0)))
    # bravo owns the band north 2..4, blocking a straight (0,0)->(6,0) shot.
    bravo = DroneRegion("bravo", ((2.0, -2.0), (2.0, 2.0), (4.0, 2.0), (4.0, -2.0)))
    kos = region_to_keep_outs(mine, (bravo,))
    arena = ArenaMap(bounds_m=(-5.0, -5.0, 10.0, 5.0), keep_out=kos, pads=(),
                     lanes=(), c2_origin_m=(0.0, 0.0), c2_heading_deg=0.0)
    legs = plan((0.0, 0.0), (6.0, 0.0), arena, inflation_m=0.3, max_leg_cm=10_000.0)
    assert len(legs) >= 2                              # not a straight shot
