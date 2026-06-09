"""land_on_pad — visual-servo landing on a VALID H-pad (the precision-landing
primitive).

Planned shape (S11), internal sub-states of this single phase:
- PAD_ACQUIRE: hover; pad seen in >= 3 of last 5 frames. Not found -> rotate
  scan (8x45 deg) -> bounded lateral steps -> acquire_timeout_s -> fallback.
- PAD_CENTER: requires camera pitched down (pyhulax set_camera_angle); step =
  clamp(k * offset_norm * altitude_m, min/max); deadband 10% of frame for 3
  consecutive ticks -> centered. Image-axis -> body-direction mapping behind
  a config sign-flip (verified onsite gate).
- PAD_DESCEND: confirm-before-descend — Move(DOWN, descend_step_cm) ONLY if
  centered AND pad in N-of-M recent frames; drift -> back to PAD_CENTER; pad
  lost -> ascend one step, re-acquire (max 2 retries).
- LAND_COMMIT: ToF altitude <= commit_alt_m (~0.5) -> Land (final blind drop;
  the marker leaves the FOV anyway).
- Fallback: total landing wall budget exceeded at ANY stage -> Land in place
  + loud UNVERIFIED_LANDING event. Never hover until the battery dies.

PadClassifier seam (briefing-dependent): ArucoPadClassifier primary
(valid_marker_ids from config); shape-based ("H" on circle) classifier stub.
The pure servo math (compute_centering_step, descend_gate) is unit-tested
with synthetic candidate sequences; SITL-tested with a scripted classifier.

Serialized landings: the orchestrator grants ONE landing slot at a time —
descending crosses the other drones' altitude bands.

Derives from: ArUco pattern of potential_detection_targets.py (audited);
relative-move vocabulary only.

================================ AS BUILT (S11 / NAV-6) ======================
PURE MissionPhase — no I/O, no SDK, no top-level numpy. The drone is already
AIRBORNE over the pad vicinity (handed off by `navigate`) with the camera
looking down; this phase only emits relative Move/Land Actions and reads the
AgentContext. It NEVER detects markers itself — the PerceptionLoop publishes
Sightings to the bus and the agent surfaces the NEW ones via ctx.sightings.

State machine (internal sub-states, instance attributes):

  PAD_ACQUIRE  Keep a bounded recent window of the last M frames (M =
               acquire_window_frames). Each step() is one "frame": we record
               whether THIS step saw a valid pad (a Sighting whose marker_id
               is in valid_marker_ids — a non-valid id or a flicker below the
               threshold is NOT a frame-hit). When >= N of the last M frames
               hit (N = acquire_min_hits), we have the pad and remember the
               BEST (largest-bbox-area, then lowest marker_id for a
               deterministic tie-break — two valid pads in frame is resolved
               HERE) sighting to centre on -> PAD_CENTER. While not acquired
               we emit a bounded Rotate scan (acquire_scan_step_deg, default
               from the search.py convention) to sweep the FOV. If the pad is
               not acquired by acquire_timeout_s (a per-phase wall deadline,
               not a per-attempt one) -> Fallback (loud).

  PAD_CENTER   pixel_offset_to_move(bbox, frame_w=frame_shape[1],
               altitude_m=telemetry.altitude_m, k=k_lateral, min_cm, max_cm,
               tol_px) — the SHARED servo (no duplicated math). A Move means
               not-yet-centred (reset the centered streak, emit it). None
               (inside the deadband) increments a consecutive-frame streak;
               at center_persist_frames in a row -> centred -> PAD_DESCEND.
               If the valid pad is LOST this step the streak resets and we
               drop back to PAD_ACQUIRE (bounded — same budget/timeout).

  PAD_DESCEND  Emit Move(DOWN, descend_step_cm) ONLY while still centred AND
               the pad is in >= descend_persist_frames of the recent window.
               After EACH descend step we RE-GATE centering: the next step
               re-runs the servo, and any drift back outside the deadband
               returns to PAD_CENTER. A pad lost mid-descend -> Move(UP,
               descend_step_cm) + back to PAD_ACQUIRE, counted against
               max_loss_retries; exhausted -> Fallback.

  LAND_COMMIT  Checked FIRST every step (it is the success funnel): when
               telemetry.altitude_m <= commit_alt_m the marker has left the
               FOV / is below the depth floor anyway -> Land(). Once is_flying
               becomes False the descent is verified -> Done(reason=...).

  Fallback     Reached when total_budget_s is exceeded at ANY stage, or the
               acquire timeout fires, or the loss retries are exhausted. We
               command Land() in place; once is_flying is False ->
               Done(reason="UNVERIFIED_LANDING: ..."), an ACTIONABLE reason the
               orchestrator/agent logs as the landing event. This is the
               anti-battery-death guarantee: EVERY non-terminal path checks
               total_budget_s before doing anything else, so the phase can
               never hover/scan forever — it always converges to a Land within
               the budget (and the LAND_COMMIT funnel converges sooner on a
               good approach).

  last_action_ok is False at any step -> Abort (WHAT/WHICH/WHY/CHECK): a
  failed Move/Land means the airframe is in an unknown attitude and continuing
  to servo from it is unsafe.

Tunables come from DroneConfig.zone["land_on_pad"] via from_config (validated
like search.py _zone_kwargs: drops _comment keys, rejects typos, no-op-trap on
degenerate values — an empty valid_marker_ids would NEVER acquire, a
descend_step_cm of 0 would never descend, so they die on the ground). The
camera HFOV folded into k_lateral and the commit_alt_m depth floor are
ONSITE-CALIBRATED values — they are config tunables, never hardcoded (gate F
measures the marker read range and the body-frame cm scale).

NOT this phase's job (boundaries): the SafetyController landing SLOT
(serialized one-descent-at-a-time, because descending crosses the other
drones' altitude bands) is an ORCHESTRATOR concern wired by NAV-8 around the
Land Action — this phase stays PURE and just emits the descend/Land Actions;
it does not build or call the slot.

Sources: finals/mission/phases/_servo.py pixel_offset_to_move (the LOCKED
lateral sign convention: px = cx - w/2 > 0 => target RIGHT of centre =>
Direction.RIGHT; altitude×similar-triangles scaling; deadband INCLUSIVE);
finals/mission/phases/search.py SentryScan (the _zone_kwargs validated-tunable
/ from_config / no-op-trap template); finals/mission/phase.py (the pure
MissionPhase contract + AgentContext). Implemented — session S11 (NAV-6).
"""
from __future__ import annotations

import enum
import math
from collections import deque
from typing import TYPE_CHECKING, Deque, List, Optional

from finals.errors import ConfigError
from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases import register_phase
from finals.mission.phases._servo import pixel_offset_to_move
from finals.mission.phases.search import _zone_kwargs
from finals.types import (Abort, Action, Direction, Done, Hover, Land, Move,
                          Rotate, Sighting)

if TYPE_CHECKING:  # type hints only — keeps the import graph minimal
    from finals.config import DroneConfig, FinalsConfig


class _SubState(enum.Enum):
    """Internal sub-states of the single land_on_pad phase."""

    PAD_ACQUIRE = "PAD_ACQUIRE"
    PAD_CENTER = "PAD_CENTER"
    PAD_DESCEND = "PAD_DESCEND"
    FALLBACK = "FALLBACK"


@register_phase
class LandOnPad(MissionPhase):
    """Visual-servo precision landing on a valid pad. See the module docstring
    for the full AS-BUILT state machine + sources."""

    name = "land_on_pad"

    #: Constructor keywords settable from DroneConfig.zone["land_on_pad"].
    _TUNABLES = (
        "valid_marker_ids", "k_lateral", "tol_px", "min_step_cm",
        "max_step_cm", "descend_step_cm", "descend_persist_frames",
        "center_persist_frames", "acquire_window_frames", "acquire_min_hits",
        "commit_alt_m", "acquire_timeout_s", "total_budget_s",
        "max_loss_retries", "acquire_scan_step_deg", "scan_dwell_s",
    )

    def __init__(self, *, valid_marker_ids: Optional[List[int]] = None,
                 k_lateral: float = 1.0, tol_px: float = 30.0,
                 min_step_cm: int = 5, max_step_cm: int = 50,
                 descend_step_cm: int = 30,
                 descend_persist_frames: int = 2,
                 center_persist_frames: int = 3,
                 acquire_window_frames: int = 5, acquire_min_hits: int = 3,
                 commit_alt_m: float = 0.5,
                 acquire_timeout_s: float = 20.0,
                 total_budget_s: float = 90.0,
                 max_loss_retries: int = 3,
                 acquire_scan_step_deg: float = 30.0,
                 scan_dwell_s: float = 0.5):
        # Config-shaped values are validated HERE, loudly, before any flight —
        # a no-op lander (empty valid_marker_ids that never acquires, a 0
        # descend step that never descends, persist counters that can never be
        # met) is a config trap that must die on the ground, not waste the
        # mission servoing over nothing or hovering until the battery dies.
        def _bad(key: str, value, why: str) -> ConfigError:
            return ConfigError(
                f"land_on_pad: {key}={value!r} invalid — {why} — check "
                f'zone["land_on_pad"]')

        def _pos_int(key: str, value, why: str) -> int:
            if (not isinstance(value, int) or isinstance(value, bool)
                    or value < 1):
                raise _bad(key, value, why)
            return value

        def _pos_num(key: str, value, why: str, *, allow_zero: bool = False
                     ) -> float:
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value)
                    or value < 0 or (value == 0 and not allow_zero)):
                raise _bad(key, value, why)
            return float(value)

        # valid_marker_ids: the green/valid pad markers. EMPTY is the killer
        # no-op trap — it would never acquire and burn the whole budget into a
        # Fallback blind land; refuse it on the ground.
        if (not isinstance(valid_marker_ids, (list, tuple))
                or not valid_marker_ids
                or not all(isinstance(m, int) and not isinstance(m, bool)
                           for m in valid_marker_ids)):
            raise _bad(
                "valid_marker_ids", valid_marker_ids,
                "must be a non-empty list of ints (the valid/green pad marker "
                "ids); empty would NEVER acquire and waste the whole budget")
        self.valid_marker_ids = frozenset(valid_marker_ids)

        # Lateral servo tunables (passed straight to pixel_offset_to_move,
        # which re-validates; we validate here too so the failure names this
        # phase + its config key, not the servo internals).
        self.k_lateral = _pos_num(
            "k_lateral", k_lateral,
            "must be a finite number > 0 (the altitude×HFOV servo gain — an "
            "ONSITE-CALIBRATED config value, gate F)")
        self.tol_px = _pos_num(
            "tol_px", tol_px,
            "must be a finite number >= 0 (the centering deadband in px); 0 "
            "would demand a perfect centre and rarely converge",
            allow_zero=True)
        self.min_step_cm = _pos_int(
            "min_step_cm", min_step_cm,
            "must be an int >= 1 (cm; the minimum lateral correction)")
        self.max_step_cm = _pos_int(
            "max_step_cm", max_step_cm,
            "must be an int >= 1 (cm; the maximum lateral correction)")
        if self.min_step_cm > self.max_step_cm:
            raise _bad(
                "min_step_cm", min_step_cm,
                f"must be <= max_step_cm ({max_step_cm}) — swapped step bounds "
                f"would pin every correction to the wrong rail")

        self.descend_step_cm = _pos_int(
            "descend_step_cm", descend_step_cm,
            "must be an int >= 1 (cm per DOWN step); 0 would never descend")
        self.descend_persist_frames = _pos_int(
            "descend_persist_frames", descend_persist_frames,
            "must be an int >= 1 (frames the pad must be seen in before a "
            "descend step — confirm-before-descend)")
        self.center_persist_frames = _pos_int(
            "center_persist_frames", center_persist_frames,
            "must be an int >= 1 (consecutive in-deadband frames to call it "
            "centred)")
        self.acquire_window_frames = _pos_int(
            "acquire_window_frames", acquire_window_frames,
            "must be an int >= 1 (the M of the N-of-M acquire window)")
        self.acquire_min_hits = _pos_int(
            "acquire_min_hits", acquire_min_hits,
            "must be an int >= 1 (the N of the N-of-M acquire window)")
        if self.acquire_min_hits > self.acquire_window_frames:
            raise _bad(
                "acquire_min_hits", acquire_min_hits,
                f"must be <= acquire_window_frames "
                f"({self.acquire_window_frames}) — N > M can NEVER be met and "
                f"would never acquire")
        if self.descend_persist_frames > self.acquire_window_frames:
            raise _bad(
                "descend_persist_frames", descend_persist_frames,
                f"must be <= acquire_window_frames "
                f"({self.acquire_window_frames}) — the recent window only "
                f"holds {self.acquire_window_frames} frames, so a larger "
                f"descend gate can never be met")

        self.commit_alt_m = _pos_num(
            "commit_alt_m", commit_alt_m,
            "must be a finite number > 0 (m; the depth floor below which we "
            "blind-Land — an ONSITE-CALIBRATED value, gate F)")
        self.acquire_timeout_s = _pos_num(
            "acquire_timeout_s", acquire_timeout_s,
            "must be a finite number > 0 (s; the wall deadline to first "
            "acquire before Fallback)")
        self.total_budget_s = _pos_num(
            "total_budget_s", total_budget_s,
            "must be a finite number > 0 (s; the whole-phase wall budget — at "
            "this point we ALWAYS converge to a Land, never hover till dead)")
        if self.acquire_timeout_s > self.total_budget_s:
            raise _bad(
                "acquire_timeout_s", acquire_timeout_s,
                f"must be <= total_budget_s ({self.total_budget_s:g}) — an "
                f"acquire timeout past the total budget can never fire (the "
                f"budget Fallback pre-empts it)")
        self.max_loss_retries = _pos_int(
            "max_loss_retries", max_loss_retries,
            "must be an int >= 1 (mid-descend lost-marker ascend+re-acquire "
            "retries before Fallback)")
        self.acquire_scan_step_deg = _pos_num(
            "acquire_scan_step_deg", acquire_scan_step_deg,
            "must be a finite number > 0 (deg, +ve = CCW; the per-step rotate "
            "scan while acquiring)")
        self.scan_dwell_s = _pos_num(
            "scan_dwell_s", scan_dwell_s,
            "must be a finite number > 0 (s; the hover dwell between scan "
            "rotates so a frame can be observed)")

        # ---- runtime state (per drone, per mission) ----
        self._sub = _SubState.PAD_ACQUIRE
        # bounded recent-frame hit window (convention 3: bounded, not growing).
        self._recent: Deque[bool] = deque(maxlen=self.acquire_window_frames)
        self._center_streak = 0
        self._loss_retries = 0
        self._target_marker_id: Optional[int] = None
        # Acquire deadline is captured on first step() (mission_elapsed_s is
        # the phase's clock; the phase may start mid-mission).
        self._t0_elapsed_s: Optional[float] = None
        self._scan_pending_dwell = False   # alternate Rotate / Hover while scanning
        self._fallback_reason: Optional[str] = None

    @classmethod
    def from_config(cls, drone_cfg: "DroneConfig",
                    cfg: "FinalsConfig") -> "LandOnPad":
        """Build from config (`cfg` unused — keeps the factory signature
        uniform across phases). All tunables validated in __init__; the camera
        HFOV folded into k_lateral and the commit_alt_m depth floor are
        ONSITE-CALIBRATED (gate F), so they are config, NOT hardcoded."""
        kwargs = _zone_kwargs(drone_cfg, "land_on_pad", cls._TUNABLES)
        return cls(**kwargs)

    # ---------------- helpers ----------------
    def _valid_sightings(self, ctx: AgentContext) -> List[Sighting]:
        """The NEW sightings this step whose marker_id is a valid pad. A
        non-valid id (e.g. an INVALID/red pad marker) is deliberately dropped
        here so it never drives centering."""
        return [s for s in ctx.sightings
                if s.marker_id is not None
                and s.marker_id in self.valid_marker_ids]

    @staticmethod
    def _bbox_area(s: Sighting) -> float:
        x0, y0, x1, y1 = s.bbox_xyxy
        return abs((float(x1) - float(x0)) * (float(y1) - float(y0)))

    def _pick_target(self, valid: List[Sighting]) -> Sighting:
        """Two valid pads in frame -> pick ONE deterministically: the largest
        bbox area (closest/most reliable), tie-broken by the lowest marker_id.
        Determinism matters — a flapping target choice would never centre."""
        return max(valid, key=lambda s: (self._bbox_area(s), -s.marker_id))

    def _recent_hits(self) -> int:
        return sum(1 for hit in self._recent if hit)

    def _go_fallback(self, reason: str) -> None:
        self._sub = _SubState.FALLBACK
        if self._fallback_reason is None:
            self._fallback_reason = reason

    @staticmethod
    def _alt_finite(alt_m) -> bool:
        """altitude_m is TRUSTED for the servo altitude-scaling + the commit
        gate; a None/NaN/inf would poison the servo step (the dead_reckon NaN
        class). True only for a real finite number."""
        return (isinstance(alt_m, (int, float)) and not isinstance(alt_m, bool)
                and math.isfinite(alt_m))

    def _abort_bad_altitude(self, ctx: AgentContext, where: str) -> Abort:
        return Abort(
            f"land_on_pad[{ctx.drone_id}]: altitude_m="
            f"{ctx.telemetry.altitude_m!r} is not a finite number during "
            f"{where} — the visual-servo step scales by altitude, so a "
            f"NaN/None/inf would poison every lateral correction; aborting "
            f"rather than servoing on poisoned telemetry — check the ToF/"
            f"-down_m source and the telemetry poller")

    # ---------------- the phase ----------------
    def step(self, ctx: AgentContext) -> Action:
        # 1. A failed prior action means the airframe is in an UNKNOWN attitude
        #    — never servo from it. WHAT/WHICH/WHY/CHECK.
        if ctx.last_action_ok is False:
            return Abort(
                f"land_on_pad[{ctx.drone_id}]: {ctx.last_action!r} failed "
                f"({ctx.last_action_error}) — aborting the visual-servo "
                f"landing instead of continuing from an unknown attitude; "
                f"check the link / the previous command")

        # Phase clock starts on the first step (the phase may begin mid-mission
        # after navigate); from here everything bounds against this t0.
        if self._t0_elapsed_s is None:
            self._t0_elapsed_s = ctx.mission_elapsed_s
        phase_elapsed_s = ctx.mission_elapsed_s - self._t0_elapsed_s

        alt_m = ctx.telemetry.altitude_m

        # 2. LAND_COMMIT funnel — checked FIRST every step (the success exit).
        #    Once we are on the deck (is_flying False) the landing is VERIFIED.
        if ctx.telemetry.is_flying is False:
            reason = self._fallback_reason
            if reason is not None:
                return Done(f"land_on_pad[{ctx.drone_id}] {reason}")
            return Done(
                f"land_on_pad[{ctx.drone_id}] VERIFIED_LANDING: on a valid pad "
                f"(marker {self._target_marker_id}); is_flying=False after the "
                f"descend/commit")

        # If we have already committed to a blind Fallback land, keep landing
        # until is_flying flips (Land is idempotent per the adapter contract).
        if self._sub is _SubState.FALLBACK:
            return Land()

        # 3. Whole-phase wall budget — the anti-battery-death guarantee. Every
        #    non-terminal path passes through here, so the phase can NEVER
        #    hover/scan forever: at the budget we converge to a Land.
        if phase_elapsed_s >= self.total_budget_s:
            self._go_fallback(
                f"UNVERIFIED_LANDING: total landing budget "
                f"{self.total_budget_s:g} s exceeded in sub-state "
                f"{self._sub.value} (phase elapsed {phase_elapsed_s:.1f} s) — "
                f"blind-landing in place; check the pad acquire/centre tuning "
                f"(k_lateral / tol_px / valid_marker_ids) and the marker read "
                f"range")
            return Land()

        # 4. Commit on the depth floor — below commit_alt_m the marker has left
        #    the FOV / is below the usable depth anyway, so we Land. A NaN/None
        #    altitude is NOT trusted to commit (fail loud, do not silently
        #    blind-land on poisoned telemetry).
        if (isinstance(alt_m, (int, float)) and not isinstance(alt_m, bool)
                and math.isfinite(alt_m) and alt_m <= self.commit_alt_m):
            return Land()

        # Record THIS frame's valid-pad hit into the bounded recent window.
        valid = self._valid_sightings(ctx)
        seen_valid = bool(valid)
        self._recent.append(seen_valid)

        # 5. Sub-state machine.
        if self._sub is _SubState.PAD_ACQUIRE:
            return self._step_acquire(ctx, valid, phase_elapsed_s)
        if self._sub is _SubState.PAD_CENTER:
            return self._step_center(ctx, valid, alt_m)
        if self._sub is _SubState.PAD_DESCEND:
            return self._step_descend(ctx, valid, alt_m)
        # Defensive: every _SubState is handled above.
        raise RuntimeError(
            f"land_on_pad[{ctx.drone_id}]: unreachable sub-state "
            f"{self._sub!r} — phase state-machine bug")

    # ---------------- sub-state steps ----------------
    def _step_acquire(self, ctx: AgentContext, valid: List[Sighting],
                      phase_elapsed_s: float) -> Action:
        # Acquire requires BOTH N-of-M recent hits AND a valid sighting THIS
        # frame to lock onto — you centre on a marker you can see right now.
        # (Without the current-frame guard, a center->acquire bounce with an
        # empty `valid` would re-acquire on the stale window and bounce back,
        # never converging — the recursion bug class.)
        if valid and self._recent_hits() >= self.acquire_min_hits:
            # Acquired: lock the target deterministically, reset centering.
            self._target_marker_id = self._pick_target(valid).marker_id
            self._center_streak = 0
            self._sub = _SubState.PAD_CENTER
            return self._step_center(ctx, valid, ctx.telemetry.altitude_m)

        # Not yet acquired — has the acquire deadline fired?
        if phase_elapsed_s >= self.acquire_timeout_s:
            self._go_fallback(
                f"UNVERIFIED_LANDING: no valid pad acquired within "
                f"{self.acquire_timeout_s:g} s (saw "
                f"{self._recent_hits()}/{len(self._recent)} of the last frames "
                f"with a valid marker, need {self.acquire_min_hits}) — "
                f"blind-landing in place; check valid_marker_ids "
                f"{sorted(self.valid_marker_ids)} vs the pad markers and the "
                f"marker read range / altitude")
            return Land()

        # Bounded rotate scan to sweep the FOV: alternate Rotate then Hover so
        # a frame can actually be observed between turns (the SentryScan idea,
        # in miniature). Both are bounded by the acquire deadline above.
        if self._scan_pending_dwell:
            self._scan_pending_dwell = False
            return Hover(duration_s=self.scan_dwell_s)
        self._scan_pending_dwell = True
        return Rotate(angle_deg=self.acquire_scan_step_deg)

    def _step_center(self, ctx: AgentContext, valid: List[Sighting],
                     alt_m) -> Action:
        if not valid:
            # Marker lost mid-centre -> back to acquire (bounded by the same
            # acquire timeout + total budget; the recent window keeps decaying).
            # Resume the scan THIS tick directly: with no current-frame
            # sighting there is nothing to lock onto, so re-dispatching into
            # acquire's re-acquire branch is both pointless and the recursion
            # trap — the bounded scan is the right behaviour.
            self._center_streak = 0
            self._sub = _SubState.PAD_ACQUIRE
            return self._step_acquire(ctx, valid,
                                      ctx.mission_elapsed_s - self._t0_elapsed_s)

        if not self._alt_finite(alt_m):
            return self._abort_bad_altitude(ctx, "PAD_CENTER")
        target = self._pick_target(valid)
        self._target_marker_id = target.marker_id
        frame_w = target.frame_shape[1]
        move = pixel_offset_to_move(
            bbox_xyxy=target.bbox_xyxy, frame_w=frame_w, altitude_m=alt_m,
            k=self.k_lateral, min_cm=float(self.min_step_cm),
            max_cm=float(self.max_step_cm), tol_px=self.tol_px)
        if move is not None:
            # Not centred yet — chase the blob; reset the centered streak.
            self._center_streak = 0
            return move
        # Inside the deadband this frame.
        self._center_streak += 1
        if self._center_streak >= self.center_persist_frames:
            self._center_streak = 0
            self._sub = _SubState.PAD_DESCEND
            return self._step_descend(ctx, valid, alt_m)
        # Hold for another centering frame (no lateral command needed).
        return Hover(duration_s=self.scan_dwell_s)

    def _step_descend(self, ctx: AgentContext, valid: List[Sighting],
                      alt_m) -> Action:
        if not valid:
            # Lost the pad mid-descend: ascend one step to widen the FOV and
            # re-acquire, bounded by max_loss_retries.
            self._loss_retries += 1
            if self._loss_retries > self.max_loss_retries:
                self._go_fallback(
                    f"UNVERIFIED_LANDING: lost the valid pad mid-descend "
                    f"{self._loss_retries - 1} times (limit "
                    f"{self.max_loss_retries}) — blind-landing in place; the "
                    f"descend step ({self.descend_step_cm} cm) may be too "
                    f"large or the marker read range too short")
                return Land()
            self._center_streak = 0
            self._sub = _SubState.PAD_ACQUIRE
            return Move(direction=Direction.UP, distance_cm=self.descend_step_cm)

        if not self._alt_finite(alt_m):
            return self._abort_bad_altitude(ctx, "PAD_DESCEND")
        # RE-GATE centering after the prior step (drift -> back to PAD_CENTER):
        # re-run the servo; any out-of-deadband correction drops us back.
        target = self._pick_target(valid)
        self._target_marker_id = target.marker_id
        frame_w = target.frame_shape[1]
        move = pixel_offset_to_move(
            bbox_xyxy=target.bbox_xyxy, frame_w=frame_w, altitude_m=alt_m,
            k=self.k_lateral, min_cm=float(self.min_step_cm),
            max_cm=float(self.max_step_cm), tol_px=self.tol_px)
        if move is not None:
            self._center_streak = 0
            self._sub = _SubState.PAD_CENTER
            return move

        # Centred AND the pad is in >= descend_persist_frames recent frames
        # (confirm-before-descend) -> step DOWN.
        # BY DESIGN (S-LAND R1/R2 batch-2, accepted disposition): `_recent` is
        # ONE shared bounded window, never reset on CENTER->DESCEND entry, so
        # the center-hold (and acquire) frames count toward this FIRST descend
        # confirm. When center_persist_frames >= descend_persist_frames (the
        # common config) the first DOWN can fire immediately on entry — this is
        # the SHARED center+descend confidence, not a descend-local count. Judged
        # soft/bounded/not-a-hover-risk (still requires centred + a valid pad
        # THIS tick, and the whole phase is bounded by total_budget_s); the
        # descent LOGIC is deliberately NOT changed right before a SITL rehearsal.
        if self._recent_hits() >= self.descend_persist_frames:
            return Move(direction=Direction.DOWN,
                        distance_cm=self.descend_step_cm)
        # Centred but not yet persistently seen — hold for another frame to
        # build the descend confidence (bounded by the total budget).
        return Hover(duration_s=self.scan_dwell_s)
