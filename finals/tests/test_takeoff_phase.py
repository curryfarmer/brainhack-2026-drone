"""finals.mission.phases.takeoff (NAV-8) — the minimal liftoff-and-HOLD phase.

The whole point of this phase vs takeoff_demo: it does NOT land at the end, so
[takeoff, navigate, land_on_pad] can compose (navigate assumes airborne). The
tests pin: the plan is exactly [Takeoff] + Done with NO Land; from_config
tunables + the altitude-band height rule + the typo no-op-trap; Abort on a
failed prior action; and that "takeoff" is in the registry.
"""
from __future__ import annotations

import time

import pytest

from finals.config import DroneConfig, load_config
from finals.errors import ConfigError
from finals.mission.phase import AgentContext
from finals.mission.phases import PHASE_REGISTRY, resolve_phase
from finals.mission.phases.takeoff import TakeoffHold
from finals.types import Abort, Done, Land, Takeoff, Telemetry


def _ctx(*, last_action_ok=None, last_action=None, last_action_error=None,
         drone_id="alpha"):
    return AgentContext(
        drone_id=drone_id, now=1.0, mission_elapsed_s=1.0,
        telemetry=Telemetry(ts=1.0), last_action=last_action,
        last_action_ok=last_action_ok, last_action_error=last_action_error)


def _drive(phase: TakeoffHold):
    """Step the phase to completion the way the agent would, returning the
    sequence of Actions it emitted."""
    out = []
    ok = None
    for _ in range(10):
        action = phase.step(_ctx(last_action_ok=ok))
        out.append(action)
        if isinstance(action, (Done, Abort)):
            break
        ok = True       # the agent reports the command succeeded
    return out


# ============================================================
# 1. Takeoff -> Done, STAYS AIRBORNE (no Land)
# ============================================================
def test_takeoff_then_done_no_land():
    phase = TakeoffHold(height_cm=120)
    actions = _drive(phase)
    assert isinstance(actions[0], Takeoff)
    assert actions[0].height_cm == 120
    assert isinstance(actions[-1], Done)
    # The load-bearing invariant: NO Land anywhere in the plan.
    assert not any(isinstance(a, Land) for a in actions), (
        "takeoff must HOLD airborne — a Land defeats the whole phase")
    assert "HOLDING" in actions[-1].reason


# ============================================================
# 2. from_config tunables + altitude-band height
# ============================================================
def test_from_config_height_cm_tunable():
    cfg = object()      # cfg unused by from_config
    d = DroneConfig(id="alpha", phases=["takeoff"],
                    zone={"takeoff": {"height_cm": 150}})
    phase = TakeoffHold.from_config(d, cfg)
    assert phase.height_cm == 150


def test_from_config_altitude_band_is_height():
    cfg = object()
    d = DroneConfig(id="alpha", phases=["takeoff"], altitude_band_m=1.7)
    phase = TakeoffHold.from_config(d, cfg)
    assert phase.height_cm == 170      # band * 100, the shared rule


def test_from_config_explicit_height_beats_band():
    cfg = object()
    d = DroneConfig(id="alpha", phases=["takeoff"], altitude_band_m=1.2,
                    zone={"takeoff": {"height_cm": 90}})
    phase = TakeoffHold.from_config(d, cfg)
    assert phase.height_cm == 90       # explicit wins over the band


def test_from_config_default_height():
    cfg = object()
    d = DroneConfig(id="alpha", phases=["takeoff"])
    phase = TakeoffHold.from_config(d, cfg)
    assert phase.height_cm == 80       # the pyhulax default


def test_from_config_rejects_typo():
    cfg = object()
    d = DroneConfig(id="alpha", phases=["takeoff"],
                    zone={"takeoff": {"heigh_cm": 100}})   # typo
    with pytest.raises(ConfigError, match="unknown key"):
        TakeoffHold.from_config(d, cfg)


def test_from_config_ignores_comment_keys():
    cfg = object()
    d = DroneConfig(id="alpha", phases=["takeoff"],
                    zone={"takeoff": {"_comment": "set onsite", "height_cm": 110}})
    phase = TakeoffHold.from_config(d, cfg)
    assert phase.height_cm == 110


# ============================================================
# 3. No-op-trap on bad height (validated on the ground)
# ============================================================
@pytest.mark.parametrize("bad", [0, -1, 80.0, True, "100"])
def test_bad_height_dies_on_the_ground(bad):
    with pytest.raises(ConfigError, match="height_cm"):
        TakeoffHold(height_cm=bad)


# ============================================================
# 4. Abort on a failed prior action
# ============================================================
def test_abort_on_last_action_failed():
    phase = TakeoffHold(height_cm=100)
    action = phase.step(_ctx(last_action_ok=False, last_action=Takeoff(100),
                             last_action_error="rejected"))
    assert isinstance(action, Abort)
    assert "rejected" in action.reason


# ============================================================
# 5. Registry
# ============================================================
def test_registered():
    assert "takeoff" in PHASE_REGISTRY
    assert resolve_phase("takeoff") is TakeoffHold


# ============================================================
# 6. Wired end-to-end through main's _build_phases (mock arena config)
# ============================================================
def test_takeoff_then_navigate_then_land_compose(repo_root):
    """The composition this phase exists for: a real landing-style sequence
    resolves [takeoff, navigate, land_on_pad] with takeoff FIRST (airborne)
    and navigate (which assumes airborne) following it cleanly."""
    import os
    from finals.main import _build_phases
    cfg = load_config(os.path.join(repo_root, "finals", "configs",
                                   "landing_real.json"))
    phases = _build_phases(cfg.drones[0], cfg)
    names = [p.name for p in phases]
    assert names == ["takeoff", "navigate", "land_on_pad"]
    # takeoff emits exactly one Takeoff then holds (no Land) — so navigate
    # really does start airborne.
    tk = phases[0]
    assert isinstance(tk.step(_ctx()), Takeoff)
    assert isinstance(tk.step(_ctx(last_action_ok=True)), Done)
