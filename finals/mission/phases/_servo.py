"""_servo — shared PURE visual-servo + heading math for the navigation phases.

Reused by land_on_pad (NAV-6, pad centering) and the navigate phase (NAV-5,
absolute-heading re-orient). track_convoy keeps its OWN private servo math until
the s11-track-and-photo branch merges; this file becomes the DRY home for both
afterwards (do NOT refactor track_convoy now — collision rule, see the spec).

All functions PURE (no I/O, no SDK, no top-level numpy): each maps a target
bearing / pixel offset to ONE Action (Rotate or Move) or None when the error is
inside the deadband. Sign convention matches vision/perception.py (bearing =
yaw MINUS pixel offset, CCW-positive) — a left/right swap MUST fail a NAV-3 test.

Derives from: vision/perception.py bearing convention (test-pinned sign);
track_convoy.py private _wrap180/_clamp/_steer (the math this generalizes).

STUB — session S11 (NAV-3). See finals/docs/module_map.md.
"""
from __future__ import annotations

from typing import Optional, Tuple

from finals.types import Move, Rotate

_STUB = ("finals.mission.phases._servo: session S11 (NAV-3) — "
         "see finals/docs/module_map.md")


def wrap180(deg: float) -> float:
    """Wrap an angle to the half-open interval (-180, 180]. (NAV-3)"""
    raise NotImplementedError(_STUB)


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value to [lo, hi]. (NAV-3)"""
    raise NotImplementedError(_STUB)


def bearing_error_to_rotate(target_deg: float, yaw_deg: float, tol_deg: float,
                            max_step_deg: float) -> Optional[Rotate]:
    """Rotate that turns from yaw_deg toward the absolute target_deg (CCW+),
    capped at max_step_deg per step; None when |wrap180(target-yaw)| <= tol_deg. (NAV-3)"""
    raise NotImplementedError(_STUB)


def pixel_offset_to_move(bbox_xyxy: Tuple[float, float, float, float],
                         frame_w: int, altitude_m: float, k: float,
                         min_cm: float, max_cm: float,
                         tol_px: float) -> Optional[Move]:
    """Lateral Move (LEFT/RIGHT) driving the bbox centroid toward the frame
    centre; step scaled by altitude so a pixel error maps to a roughly-constant
    ground step, clamped to [min_cm, max_cm]; None inside tol_px. (NAV-3)"""
    raise NotImplementedError(_STUB)
