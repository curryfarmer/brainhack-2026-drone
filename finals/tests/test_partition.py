"""Pins the NAV map-partition CONTRACT + stub (mission/planning/partition.py).

Kept as a stub by design ("keep as stub first", 2026-06-10): the advisory
SectorGuard is the active spatial-deconfliction mechanism. These tests fix the
contract shape so a future session can fill region_to_keep_outs without
re-litigating the interface, and assert the stub fails loud with the module_map
pointer (the repo stub convention). Pure — stdlib + pytest only.
"""
from __future__ import annotations

import pytest

from finals.mission.planning.partition import DroneRegion, region_to_keep_outs

_TRI_A = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))
_TRI_B = ((2.0, 0.0), (3.0, 0.0), (3.0, 1.0))


def test_region_to_keep_outs_is_a_documented_stub():
    a = DroneRegion(drone_id="alpha", keep_in_polygon_m=_TRI_A)
    b = DroneRegion(drone_id="bravo", keep_in_polygon_m=_TRI_B)
    with pytest.raises(NotImplementedError, match="module_map.md"):
        region_to_keep_outs(a, (b,))


def test_drone_region_is_frozen_and_holds_the_contract_fields():
    r = DroneRegion(drone_id="alpha", keep_in_polygon_m=_TRI_A)
    assert r.drone_id == "alpha"
    assert r.keep_in_polygon_m == _TRI_A
    with pytest.raises(Exception):           # frozen dataclass -> no mutation
        r.drone_id = "bravo"                 # type: ignore[misc]
