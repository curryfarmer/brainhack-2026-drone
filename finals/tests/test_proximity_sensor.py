"""finals.flight.proximity — the IR sensor seam feeding ProximityGuard.

The SyntheticProximitySensor is the honest degrade-absent feed (the live
pyhulax IR read is an onsite gate); the PyhulaxProximitySensor is a deliberate
stub. These tests pin: the default (reading None -> the guard SKIPS), the
scriptable feeds (a fixed reading + a callable), the re-stamping (ts tracks the
injected clock), and that the live stub raises a module-map-pointing
NotImplementedError so it can never be silently wired half-built.
"""
from __future__ import annotations

import pytest

from finals.flight.proximity import (ProximitySensor, PyhulaxProximitySensor,
                                     SyntheticProximitySensor)
from finals.guards import ProximityReading


class FakeClock:
    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


def test_synthetic_default_reads_none():
    """The production default: no live IR -> read() is None -> the guard
    SKIPS (never a fabricated clear lane)."""
    sensor = SyntheticProximitySensor("alpha")
    assert isinstance(sensor, ProximitySensor)
    assert sensor.read() is None


def test_synthetic_fixed_reading_is_restamped():
    clock = FakeClock(100.0)
    base = ProximityReading(ts=1.0, front_cm=30.0, right_cm=42.0)
    sensor = SyntheticProximitySensor("alpha", reading=base, clock=clock)
    r = sensor.read()
    assert r is not None
    assert r.front_cm == 30.0 and r.right_cm == 42.0
    assert r.back_cm is None and r.left_cm is None
    assert r.ts == 100.0                         # re-stamped to the agent clock
    clock.t = 105.0
    assert sensor.read().ts == 105.0             # tracks the clock each read


def test_synthetic_callable_reading_varies():
    """A callable lets a rehearsal vary the range over time (an approach)."""
    ranges = iter([ProximityReading(ts=0.0, front_cm=80.0),
                   ProximityReading(ts=0.0, front_cm=40.0),
                   ProximityReading(ts=0.0, front_cm=20.0)])
    sensor = SyntheticProximitySensor("alpha", reading=lambda: next(ranges))
    assert sensor.read().front_cm == 80.0
    assert sensor.read().front_cm == 40.0
    assert sensor.read().front_cm == 20.0


def test_synthetic_callable_returning_none_is_none():
    sensor = SyntheticProximitySensor("alpha", reading=lambda: None)
    assert sensor.read() is None


def test_synthetic_rejects_bad_args():
    with pytest.raises(ValueError, match="drone_id"):
        SyntheticProximitySensor("")
    with pytest.raises(ValueError, match="reading"):
        SyntheticProximitySensor("alpha", reading=42)


def test_pyhulax_proximity_sensor_is_a_stub():
    """The LIVE read is an ONSITE GATE — the stub raises a module-map-pointing
    NotImplementedError (it can never be silently wired half-built)."""
    sensor = PyhulaxProximitySensor("alpha", api=object())
    assert isinstance(sensor, ProximitySensor)
    with pytest.raises(NotImplementedError, match="ONSITE GATE"):
        sensor.read()


def test_pyhulax_proximity_stub_points_at_module_map():
    sensor = PyhulaxProximitySensor("alpha", api=object())
    with pytest.raises(NotImplementedError, match="module_map.md"):
        sensor.read()
