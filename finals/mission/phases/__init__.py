"""PHASE_REGISTRY — phase name -> class, so configs reference phases BY NAME
and briefing-day mission changes are config edits, not code surgery.

Implemented in S1 (the registry mechanism); phases populate it as their
sessions land. Stub phases ARE registered so `--dry-run` name-validation
works from day one — instantiating one raises its session pointer loudly.

Session: S1 (implemented).
"""
from __future__ import annotations

from typing import Dict, Type

from finals.errors import ConfigError
from finals.mission.phase import MissionPhase

PHASE_REGISTRY: Dict[str, Type[MissionPhase]] = {}


def register_phase(cls: Type[MissionPhase]) -> Type[MissionPhase]:
    """Class decorator. Registers cls under cls.name; duplicate names are a
    programming error and fail immediately."""
    name = getattr(cls, "name", None)
    if not isinstance(name, str) or not name or name == "abstract":
        raise ConfigError(
            f"phase class {cls.__name__} must define a non-empty class attribute "
            f"`name` before registration"
        )
    if name in PHASE_REGISTRY:
        raise ConfigError(
            f"duplicate phase name {name!r}: {PHASE_REGISTRY[name].__name__} "
            f"vs {cls.__name__}"
        )
    PHASE_REGISTRY[name] = cls
    return cls


def resolve_phase(name: str) -> Type[MissionPhase]:
    """Phase name -> class; ConfigError listing available names on miss."""
    try:
        return PHASE_REGISTRY[name]
    except KeyError:
        raise ConfigError(
            f"unknown phase {name!r} — available: {sorted(PHASE_REGISTRY)} "
            f"(phases register in finals/mission/phases/)"
        ) from None


# Importing the phase modules populates the registry (stubs included, so
# config name-validation works before the phases are implemented).
from finals.mission.phases import takeoff_demo as _takeoff_demo  # noqa: E402,F401
from finals.mission.phases import takeoff as _takeoff            # noqa: E402,F401
from finals.mission.phases import search as _search              # noqa: E402,F401
from finals.mission.phases import navigate as _navigate          # noqa: E402,F401
from finals.mission.phases import track_convoy as _track_convoy  # noqa: E402,F401
from finals.mission.phases import land_on_pad as _land_on_pad    # noqa: E402,F401
