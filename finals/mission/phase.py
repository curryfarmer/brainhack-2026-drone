"""MissionPhase — the pure state-machine framework for mission logic.

This is the hula_connection.py lines 46-50 official advice ("break your plan
into states and use if/else ... a state variable per drone") formalized so it
is unit-testable and non-blocking:

- A phase NEVER does I/O, never sleeps, never touches an SDK. It looks at an
  AgentContext (assembled by DroneAgent each tick) and returns ONE Action.
- step() is called ONLY when no command is in flight for that drone, so a
  phase never has to reason about concurrency.
- Whole missions therefore run in plain pytest against MockAdapter with zero
  hardware — the property the entire test pyramid stands on.

Concrete phases register in finals.mission.phases.PHASE_REGISTRY and are
referenced BY NAME from config (DroneConfig.phases), so briefing-day mission
changes are config edits, not code surgery.

Session: S1 (contract implemented; exercised by agent/orchestrator from S4).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional

from finals.types import Action, Sighting, Telemetry


@dataclass(frozen=True)
class AgentContext:
    """READ-ONLY view of one drone's world, rebuilt by DroneAgent each tick."""

    drone_id: str
    now: float                                   # time.monotonic()
    mission_elapsed_s: float
    telemetry: Telemetry
    sightings: List[Sighting] = field(default_factory=list)   # new since last step()
    last_action: Optional[Action] = None
    last_action_ok: Optional[bool] = None        # None until the first action resolves
    last_action_error: Optional[str] = None      # actionable message when ok is False


class MissionPhase(ABC):
    """One mission behavior (takeoff demo, search, land-on-pad, ...).

    Subclasses keep their own internal counters/state as instance attributes —
    a fresh instance is built per drone per mission, so no cross-drone leakage.
    """

    #: Registry name; concrete phases set this and appear in PHASE_REGISTRY.
    name: str = "abstract"

    @abstractmethod
    def step(self, ctx: AgentContext) -> Action:
        """Decide the next Action. PURE: no I/O, no sleep, no SDK calls.
        Called only when no command is in flight. Return:
        - a flight Action (Takeoff/Move/Rotate/Hover/Land) to command the drone,
        - Wait(t) to idle without commanding,
        - Done(reason) to advance the agent to its next phase,
        - Abort(reason) to fail this drone loudly (agent safes it down)."""

    def on_enter(self, ctx: AgentContext) -> None:
        """Hook when the agent enters this phase. Default no-op. Still pure."""
        return None

    def on_exit(self, ctx: AgentContext) -> None:
        """Hook when the agent leaves this phase. Default no-op. Still pure."""
        return None
