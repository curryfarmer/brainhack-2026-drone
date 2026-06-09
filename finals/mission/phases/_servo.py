"""_servo — shared PURE visual-servo + heading math for the navigation phases.

Reused by land_on_pad (NAV-6, pad centering) and the navigate phase (NAV-5,
absolute-heading re-orient). track_convoy keeps its OWN private servo math until
the s11-track-and-photo branch merges; this file becomes the DRY home for both
afterwards (do NOT refactor track_convoy now — collision rule, see the spec).

All functions PURE (no I/O, no SDK, no top-level numpy): each maps a target
bearing / pixel offset to ONE Action (Rotate or Move) or None when the error is
inside the deadband.

Sign conventions (LOAD-BEARING — the integrator wires NAV-5/NAV-6 to these;
they are pinned by tests/test_servo.py, and a swap MUST fail a test):

- Heading (bearing_error_to_rotate): the BINDING CCW-positive yaw frame
  (finals/flight/dead_reckon.py: Rotate(+90) turns the nose 90 deg
  counter-clockwise viewed from above; finals/types.py Rotate.angle_deg
  "+ve = CCW"). error = wrap180(target_deg - yaw_deg); a target that lies
  COUNTER-CLOCKWISE of the current nose has error > 0 and is reached by a
  POSITIVE (CCW) Rotate. This is the same frame vision/perception.py
  bearing_from_bbox emits its bearing_deg in (yaw MINUS the pixel offset,
  CCW+, test-pinned S7) — so a bearing fed in as target_deg steers the right
  way without any sign juggling at the call site.

- Lateral (pixel_offset_to_move): a DOWN-LOOKING camera centering a pad. With
  pixel x growing rightward, px = cx - frame_w/2 > 0 means the target's
  centroid is RIGHT of frame centre, i.e. to the drone's RIGHT in the body
  frame; the recentering Move is therefore Direction.RIGHT (move the body
  TOWARD the target). px < 0 -> Direction.LEFT. (This is body-frame "chase the
  blob", distinct from the perception bearing sign, which maps a right-pixel to
  a CLOCKWISE world bearing; both are correct in their own frame.)

Why the altitude scale (pixel_offset_to_move): by similar triangles the GROUND
distance subtended by one pixel grows linearly with height above the target
(ground_per_px = altitude / focal_px). Scaling the normalized pixel error by
altitude_m therefore makes a fixed pixel offset map to a roughly-constant
ground step in cm regardless of height — k folds in the (frame_w / focal_px)
camera constant, so it stays a single config tunable.

Derives from: vision/perception.py bearing convention (test-pinned sign, S7);
finals/types.py Rotate/Move/Direction (CCW+, body-frame). The planned DRY home
for track_convoy.py's private _wrap180/_clamp/_steer once s11 merges.

Implemented — session S11 (NAV-3). See finals/docs/module_map.md.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

from finals.types import Direction, Move, Rotate


def _require_finite(name: str, value: float) -> float:
    """Reject NaN/inf/non-number loudly — a silent NaN poisons every
    downstream servo step (the dead_reckon.py bug class). bool is rejected
    because True/False sneaking in as an angle is always a wiring bug."""
    if (not isinstance(value, (int, float)) or isinstance(value, bool)
            or not math.isfinite(value)):
        raise ValueError(
            f"_servo: {name} must be a finite number, got {value!r} — a NaN/"
            f"inf here would silently poison the visual-servo step; check the "
            f"telemetry/bbox/config value feeding {name}")
    return float(value)


def wrap180(deg: float) -> float:
    """Wrap an angle to the half-open interval (-180, 180]. (NAV-3)

    Boundary (pinned): +180 -> +180, -180 -> +180, +540 -> +180,
    -540 -> +180 (the open end is at -180, the closed end at +180). NaN/inf
    raise ValueError (fail loud — a wrapped NaN looks like a tiny heading
    error and would silently stall a re-orient).
    """
    deg = _require_finite("deg", deg)
    # Python's % always lands in [0, 360) for a positive modulus, so every
    # multiple-of-180 boundary input (-180, 180, 540, -540, ...) collapses to
    # exactly 180.0 here; `> 180` (strict) then KEEPS it, giving the closed +180
    # end for free — there is no separate -180 case to fix up. Anything in
    # (180, 360) is the lower half, shifted into (-180, 0).
    wrapped = deg % 360.0
    if wrapped > 180.0:
        wrapped -= 360.0
    return wrapped


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value to [lo, hi]. (NAV-3)

    Asserts lo <= hi (a swapped bound is a caller bug that would silently
    pin every value to the wrong rail).
    """
    value = _require_finite("clamp value", value)
    lo = _require_finite("clamp lo", lo)
    hi = _require_finite("clamp hi", hi)
    if lo > hi:
        raise ValueError(
            f"clamp: lo ({lo}) must be <= hi ({hi}) — a swapped bound would "
            f"silently pin every value to the wrong rail; check the min/max "
            f"args at the call site")
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def bearing_error_to_rotate(target_deg: float, yaw_deg: float, tol_deg: float,
                            max_step_deg: float) -> Optional[Rotate]:
    """Rotate that turns from yaw_deg toward the absolute target_deg (CCW+),
    capped at max_step_deg per step; None when |wrap180(target-yaw)| <= tol_deg. (NAV-3)

    error = wrap180(target_deg - yaw_deg). |error| <= tol_deg -> None
    (inside the deadband, boundary INCLUSIVE so a target exactly tol away does
    not chatter). Otherwise Rotate(clamp(error, -max_step_deg, +max_step_deg)):
    a positive error (target CCW of the nose) -> positive (CCW) Rotate, which
    is the shortest turn because wrap180 already picked the <=180 direction.
    """
    target_deg = _require_finite("target_deg", target_deg)
    yaw_deg = _require_finite("yaw_deg", yaw_deg)
    tol_deg = _require_finite("tol_deg", tol_deg)
    max_step_deg = _require_finite("max_step_deg", max_step_deg)
    if tol_deg < 0.0:
        raise ValueError(
            f"bearing_error_to_rotate: tol_deg must be >= 0, got {tol_deg} — "
            f"a negative deadband can never be satisfied; check the config")
    if max_step_deg <= 0.0:
        raise ValueError(
            f"bearing_error_to_rotate: max_step_deg must be > 0, got "
            f"{max_step_deg} — a zero/negative step would never (or "
            f"backwards) close the heading error; check the config")
    error = wrap180(target_deg - yaw_deg)
    if abs(error) <= tol_deg:
        return None
    return Rotate(angle_deg=clamp(error, -max_step_deg, max_step_deg))


def pixel_offset_to_move(bbox_xyxy: Tuple[float, float, float, float],
                         frame_w: int, altitude_m: float, k: float,
                         min_cm: float, max_cm: float,
                         tol_px: float) -> Optional[Move]:
    """Lateral Move (LEFT/RIGHT) driving the bbox centroid toward the frame
    centre; step scaled by altitude so a pixel error maps to a roughly-constant
    ground step, clamped to [min_cm, max_cm]; None inside tol_px. (NAV-3)

    cx = (x0 + x2) / 2; px = cx - frame_w/2. |px| <= tol_px -> None (deadband,
    boundary INCLUSIVE). Else:
        error_norm = px / frame_w
        step_cm = clamp(|k * error_norm * altitude_m * 100|, min_cm, max_cm)
    Direction: px > 0 (target RIGHT of centre, i.e. to the drone's right in
    body frame) -> Direction.RIGHT; px < 0 -> Direction.LEFT. The body moves
    TOWARD the target to recentre it. distance_cm is rounded to the nearest int
    (Move.distance_cm is an int per the FlightAdapter cm contract); altitude
    scaling is documented in the module docstring (similar triangles).
    """
    frame_w_in = frame_w
    if (not isinstance(frame_w_in, int) or isinstance(frame_w_in, bool)
            or frame_w_in <= 0):
        raise ValueError(
            f"pixel_offset_to_move: frame_w must be an int > 0, got "
            f"{frame_w_in!r} — check Sighting.frame_shape[1]")
    altitude_m = _require_finite("altitude_m", altitude_m)
    if altitude_m < 0.0:
        raise ValueError(
            f"pixel_offset_to_move: altitude_m must be >= 0, got {altitude_m} "
            f"— a negative height is a telemetry/sign bug; check the ToF/"
            f"-down_m source")
    k = _require_finite("k", k)
    min_cm = _require_finite("min_cm", min_cm)
    max_cm = _require_finite("max_cm", max_cm)
    tol_px = _require_finite("tol_px", tol_px)
    if min_cm < 0.0:
        raise ValueError(
            f"pixel_offset_to_move: min_cm must be >= 0, got {min_cm} — a "
            f"negative floor would let the clamp emit a backwards step; check "
            f"the config")
    if min_cm > max_cm:
        raise ValueError(
            f"pixel_offset_to_move: min_cm ({min_cm}) must be <= max_cm "
            f"({max_cm}) — swapped step bounds; check the config")
    if tol_px < 0.0:
        raise ValueError(
            f"pixel_offset_to_move: tol_px must be >= 0, got {tol_px} — a "
            f"negative deadband can never be satisfied; check the config")
    for i in (0, 2):
        _require_finite(f"bbox_xyxy[{i}]", bbox_xyxy[i])

    cx = (float(bbox_xyxy[0]) + float(bbox_xyxy[2])) / 2.0
    px = cx - frame_w_in / 2.0
    if abs(px) <= tol_px:
        return None
    error_norm = px / frame_w_in
    step_cm = clamp(abs(k * error_norm * altitude_m * 100.0), min_cm, max_cm)
    direction = Direction.RIGHT if px > 0.0 else Direction.LEFT
    return Move(direction=direction, distance_cm=int(round(step_cm)))
