"""Typed exception hierarchy for the finals package.

One tree so callers can distinguish "refuse to start" (ConfigError,
PreflightError) from "this drone failed" (FlightError) from "a sensor went
quiet" (SensorError) from "the operator pulled the plug" (AbortRequested).

Derives from: the failure-mode audit of the official examples — everything
mapping_drone.py / dola.py / UWBParserThread.py swallow silently becomes a
typed, message-carrying exception here.

Conventions (binding, enforced by tests/test_conventions.py):
- Messages must be actionable: say WHAT was attempted, on WHICH drone,
  for HOW long, and what to check. e.g.
  FlightTimeout("alpha: move(FORWARD, 100 cm) exceeded 15.0 s — check Wi-Fi link / drone power")
- Never raise bare Exception; never catch these without either handling the
  specific type or re-raising.

Session: S1 (implemented).
"""
from __future__ import annotations


class FinalsError(Exception):
    """Base for every error this package raises on purpose."""


class ConfigError(FinalsError):
    """Bad or missing configuration — refuse to start, name the exact key/file."""


class PreflightError(FinalsError):
    """A preflight check failed — refuse to fly, include the failing check's report."""


class FlightError(FinalsError):
    """A flight command failed. Message carries drone_id + action + cause."""


class FlightTimeout(FlightError):
    """A flight command exceeded its timeout_s deadline (the SDK may still be
    executing it — adapter marks itself degraded; agent must safe-down)."""


class SensorError(FinalsError):
    """A sensor source (video, telemetry, UWB serial) failed."""


class SensorTimeout(SensorError):
    """A sensor did not deliver data within its deadline (e.g. no first video
    frame within start() timeout)."""


class AbortRequested(FinalsError):
    """Operator abort (kill key / Ctrl+C). Triggers land-all, never ignored."""


class PlanningError(FinalsError):
    """The 2-D path planner could not produce a transit plan — refuse to fly
    that leg of the mission. Like ConfigError this is a "refuse to start (this
    transit)" failure, not a per-drone in-flight fault, so it derives straight
    from FinalsError rather than FlightError.

    Message must be ACTIONABLE: name WHAT was attempted (plan start->goal),
    WHICH start/goal points (with the keep-out id when one is the cause), WHY
    it failed (goal inside an inflated keep-out / start trapped / no
    collision-free path), and what to CHECK (the arena keep_out polygons, the
    inflation_m margin, or the chosen start/goal). The visibility_graph.plan
    raise sites carry exactly that — never a bare 'no path'."""
