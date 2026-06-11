"""flight_monitor.py — props-off instrumented flight + live monitor (ONE link).

Our bench drones fly with the PROPELLERS REMOVED: motors spin, commands are
accepted, telemetry responds (yaw turns on rotate, `is_flying` flips), but the
airframe does NOT climb or translate. This harness runs a real mission over a
SINGLE pyhulax link while showing, live:

  * a HULA camera window with ArUco + YOLO overlay (reuses live_view's viewer),
  * a 2 Hz terminal readback of altitude + yaw + is_flying,
  * a forward DEPTH poll ("OBJECT AHEAD <x>m" when something is within the
    threshold) from a SEPARATE forward-facing sensor (Intel RealSense over USB;
    `--depth-backend fake` for dev). The HULA cam is monocular — depth is a
    second camera, not the HULA feed,
  * a camera-tilt control: the HULA main camera DOES pitch 0-90 deg up/down via
    pyhulax `set_camera_angle(CameraPitchMode, angle)` (confirmed on hardware) —
    `--camera-tilt-deg N` commands it (N>0 = look down), else it just probes.

Two test profiles (both fully instrumented):
  --test takeoff_land : Takeoff -> Hover -> Land. The basic plumbing check.
  --test scan_land    : Takeoff -> ~3x3 m OpenLoopLawnmower -> decode ArUco ->
                        BROADCAST each sighting to the in-process SightingBus ->
                        LandOnPad servoes onto the nearest (largest-bbox) valid
                        pad. The full perception -> broadcast -> landing-decision
                        chain, monitored live.

HONEST SCOPE (props off): this proves command acceptance, the telemetry/
perception/broadcast/landing-servo plumbing, and yaw response — NOT physical
motion, height gain, lawnmower coverage, or a real touchdown. Those are props-on
or SITL (sim/run_landing.sh).

Threading: cv2's GUI owns the MAIN thread (camera + perception + HUD + the 2 Hz
prints + the depth poll + publishing ArUco sightings to the bus). A WORKER
thread runs the flight: it steps the mission phase(s) and applies each Action to
the adapter, draining the SAME bus into ctx.sightings for LandOnPad. The two
share one connected PyhulaxAdapter (no event-loop affinity — connect on the main
thread, command on the worker's loop). 'q' (window or stdin AbortListener) sets
abort_event; the worker safes down (emergency_land) and disconnects on teardown.

This module imports NO SDK at the top level (numpy/cv2/ultralytics/pyhulax/
pyrealsense2 are lazy, inside the live functions) so the pure logic — phase
assembly, the flight runner, the 2 Hz formatting, the depth reduction — imports
and unit-tests on the BARE venv. It is on test_conventions' `except Exception`
whitelist: the flight worker must not die silently, and the camera-tilt probe
hits an open SDK error set.

Run (dev dry-run, no hardware):
  python -m finals.tools.flight_monitor --test scan_land --mock --headless --duration-s 8
Run (real drone, PROPS OFF):
  python -m finals.tools.flight_monitor --test takeoff_land --ip 192.168.100.1 \
      --plane-id 6 --props-off-confirmed --depth-backend realsense
"""
from __future__ import annotations

import argparse
import asyncio
import sys
import threading
import time
from types import SimpleNamespace
from typing import List, Optional

from finals.errors import (FinalsError, FlightError, SensorError, SensorTimeout)
from finals.events import EventLog
from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases.land_on_pad import LandOnPad
from finals.mission.phases.search import OpenLoopLawnmower
from finals.sightings import SightingBus
from finals.types import (Abort, Action, Direction, Done, Hover, Land, Move,
                          Rotate, Sighting, Takeoff, Wait)

# ---- tunables -------------------------------------------------------------
_MONITOR_PERIOD_S = 0.5                      # 2 Hz readback
_DEPTH_OBJECT_THRESHOLD_M = 1.0              # "OBJECT AHEAD" when nearer than this
_FIELD_MARKER_IDS = [11, 45, 51, 67, 101]    # the fixed field pad ids (DICT_7X7_1000)
_DEFAULT_ARUCO_DICT = "DICT_7X7_1000"
_DEFAULT_TAKEOFF_CM = 80
_CMD_TIMEOUT_S = 20.0                         # per-command timeout (props-off bench)
_MISSION_BUDGET_S = 240.0                     # hard wall on the whole flight runner
_TELEM_STALE_S = 8.0
_LINGER_S = 2.0                               # keep the window up briefly after landing


class FlightMonitorError(FinalsError):
    """A flight_monitor wiring/usage fault (bad --test, bad action). Loud."""


# ===========================================================================
# Phase assembly (PURE — bare-venv testable)
# ===========================================================================
class _FixedPlanPhase(MissionPhase):
    """Yield a fixed list of Actions then Done — the Test-1 plan. Mirrors the
    OpenLoopLawnmower step contract (Abort if the last action failed) so the
    runner treats it identically."""

    name = "fixed_plan"

    def __init__(self, actions: List[Action]):
        self._plan = list(actions)
        self._idx = 0

    def step(self, ctx: AgentContext) -> Action:
        if ctx.last_action_ok is False:
            return Abort(
                f"fixed_plan[{ctx.drone_id}]: {ctx.last_action!r} failed "
                f"({ctx.last_action_error}) — aborting")
        if self._idx >= len(self._plan):
            return Done("fixed plan complete")
        action = self._plan[self._idx]
        self._idx += 1
        return action


def _lawnmower_no_land(**kwargs) -> OpenLoopLawnmower:
    """OpenLoopLawnmower with its trailing Land() popped, so it CHAINS into
    LandOnPad (the drone must stay airborne for the landing servo)."""
    lm = OpenLoopLawnmower(**kwargs)
    if lm._plan and isinstance(lm._plan[-1], Land):
        lm._plan.pop()
    return lm


def build_test_phases(test: str, *, marker_ids: Optional[List[int]] = None,
                      takeoff_cm: int = _DEFAULT_TAKEOFF_CM,
                      lawnmower_kw: Optional[dict] = None,
                      land_kw: Optional[dict] = None) -> List[MissionPhase]:
    """Return the phase list for a test profile. PURE — constructs phases only."""
    marker_ids = list(marker_ids if marker_ids is not None else _FIELD_MARKER_IDS)
    if test == "takeoff_land":
        return [_FixedPlanPhase([Takeoff(height_cm=takeoff_cm),
                                 Hover(duration_s=2.0), Land()])]
    if test == "scan_land":
        lm_kw = dict(height_cm=takeoff_cm, lanes=3, leg_cm=300, lane_cm=150,
                     turn_deg=90.0, scan_pause_s=1.0)
        lm_kw.update(lawnmower_kw or {})
        lk = dict(valid_marker_ids=marker_ids, servo_on="marker")
        lk.update(land_kw or {})
        return [_lawnmower_no_land(**lm_kw), LandOnPad(**lk)]
    raise FlightMonitorError(
        f"unknown --test {test!r} — want 'takeoff_land' or 'scan_land'")


# ===========================================================================
# 2 Hz formatting + depth reduction (PURE — bare-venv testable)
# ===========================================================================
def format_monitor_line(*, alt_m: Optional[float], yaw_deg: Optional[float],
                        is_flying: Optional[bool], depth_m: Optional[float],
                        threshold_m: float = _DEPTH_OBJECT_THRESHOLD_M) -> str:
    """The one-line 2 Hz readback. depth_m None -> 'depth n/a'; <= threshold ->
    'OBJECT AHEAD'; else 'clear'."""
    a = " n/a " if alt_m is None else f"{alt_m:5.2f}m"
    y = "  n/a  " if yaw_deg is None else f"{yaw_deg:+6.1f}deg"
    fl = "?" if is_flying is None else ("FLYING" if is_flying else "ground")
    if depth_m is None:
        d = "depth n/a"
    elif depth_m <= threshold_m:
        d = f"OBJECT AHEAD {depth_m:.2f}m"
    else:
        d = f"clear {depth_m:.2f}m"
    return f"[2Hz] alt {a}  yaw {y}  {fl}  | {d}"


def nearest_object_m(depth_frame, *, region_frac: float = 0.34) -> Optional[float]:
    """Smallest positive distance over a central box (region_frac of W and H) of
    a DepthFrame's coarse metres grid. None when no frame / no valid return."""
    if depth_frame is None:
        return None
    w = getattr(depth_frame, "width", 0)
    h = getattr(depth_frame, "height", 0)
    if not w or not h:
        return None
    rx = max(0, int(w * region_frac / 2.0))
    ry = max(0, int(h * region_frac / 2.0))
    cx, cy = w // 2, h // 2
    best: Optional[float] = None
    for yy in range(max(0, cy - ry), min(h, cy + ry + 1)):
        for xx in range(max(0, cx - rx), min(w, cx + rx + 1)):
            d = depth_frame.distance_at(xx, yy)
            if d is not None and d > 0 and (best is None or d < best):
                best = d
    return best


def _action_fields(action: Action) -> dict:
    if isinstance(action, Takeoff):
        return {"height_cm": action.height_cm}
    if isinstance(action, Move):
        return {"direction": action.direction.name, "distance_cm": action.distance_cm}
    if isinstance(action, Rotate):
        return {"angle_deg": action.angle_deg}
    if isinstance(action, Hover):
        return {"duration_s": action.duration_s}
    return {}


# ===========================================================================
# Flight runner (async; pure except for the injected adapter — bare-venv
# testable with a stub adapter, no SDK)
# ===========================================================================
async def _apply_action(adapter, action: Action, timeout_s: float) -> None:
    """Dispatch one flight Action to the adapter (mirrors DroneAgent._execute)."""
    if isinstance(action, Takeoff):
        await adapter.takeoff(height_cm=action.height_cm, timeout_s=timeout_s)
    elif isinstance(action, Move):
        await adapter.move(action.direction, action.distance_cm, timeout_s=timeout_s)
    elif isinstance(action, Rotate):
        await adapter.rotate(action.angle_deg, timeout_s=timeout_s)
    elif isinstance(action, Hover):
        await adapter.hover(action.duration_s)
    elif isinstance(action, Land):
        await adapter.land(timeout_s=timeout_s)
    else:
        raise FlightMonitorError(f"_apply_action: unsupported action {action!r}")


async def _abortable_sleep(duration_s: float, abort_event) -> None:
    """Sleep up to duration_s (real wall clock), returning early if aborted."""
    end = time.monotonic() + max(0.0, float(duration_s))
    while time.monotonic() < end:
        if abort_event is not None and abort_event.is_set():
            return
        await asyncio.sleep(min(0.05, max(0.0, end - time.monotonic())))


async def run_phases(adapter, phases: List[MissionPhase], drone_id: str, *,
                     bus: Optional[SightingBus] = None,
                     events: Optional[EventLog] = None,
                     abort_event=None,
                     command_timeout_s: float = _CMD_TIMEOUT_S,
                     mission_budget_s: float = _MISSION_BUDGET_S,
                     clock=time.monotonic) -> dict:
    """Step `phases` to completion, applying each Action to the (already
    connected) adapter. Drains `bus` into ctx.sightings each step (the swarm
    broadcast LandOnPad consumes). Bounded by mission_budget_s. Always safes the
    drone down (emergency_land) on the way out. Returns a summary dict.

    The adapter is NOT connected or disconnected here — the caller owns the link
    (so the camera can share it). props-off: motors spin, no motion."""
    summary: dict = {"drone_id": drone_id, "phases": [], "completed": False,
                     "aborted": False, "error": None}
    airborne = False
    bus_cursor = 0
    t_start = clock()

    def _ev(event: str, **data) -> None:
        if events is not None:
            events.log(drone_id, event, **data)

    try:
        for phase in phases:
            _ev("phase_enter", phase=phase.name)
            entered = False
            last_action: Optional[Action] = None
            last_ok: Optional[bool] = None
            last_err: Optional[str] = None
            while True:
                if abort_event is not None and abort_event.is_set():
                    summary["aborted"] = True
                    _ev("monitor_abort", phase=phase.name, reason="operator 'q'")
                    return summary
                now = clock()
                if now - t_start > mission_budget_s:
                    summary["error"] = (f"mission budget {mission_budget_s:.0f}s "
                                        f"exceeded in phase {phase.name}")
                    _ev("budget_exceeded", phase=phase.name,
                        budget_s=mission_budget_s)
                    return summary
                try:
                    telem = adapter.telemetry()
                except FlightError as e:
                    summary["error"] = f"telemetry read failed: {e}"
                    _ev("telemetry_failed", phase=phase.name, error=str(e))
                    return summary
                sightings: List[Sighting] = []
                if bus is not None:
                    bus_cursor, sightings = bus.drain_after(bus_cursor,
                                                            drone_id=drone_id)
                ctx = AgentContext(
                    drone_id=drone_id, now=now, mission_elapsed_s=now - t_start,
                    telemetry=telem, sightings=sightings,
                    last_action=last_action, last_action_ok=last_ok,
                    last_action_error=last_err)
                if not entered:
                    phase.on_enter(ctx)
                    entered = True
                action = phase.step(ctx)
                if isinstance(action, Done):
                    phase.on_exit(ctx)
                    _ev("phase_done", phase=phase.name, reason=action.reason)
                    summary["phases"].append(
                        {"phase": phase.name, "result": "done",
                         "reason": action.reason})
                    break
                if isinstance(action, Abort):
                    _ev("phase_abort", phase=phase.name, reason=action.reason)
                    summary["aborted"] = True
                    summary["error"] = action.reason
                    summary["phases"].append(
                        {"phase": phase.name, "result": "abort",
                         "reason": action.reason})
                    return summary
                if isinstance(action, Wait):
                    await _abortable_sleep(action.duration_s, abort_event)
                    last_action, last_ok, last_err = action, True, None
                    continue
                if isinstance(action, (Takeoff, Move, Rotate, Hover, Land)):
                    try:
                        await _apply_action(adapter, action, command_timeout_s)
                    except FlightError as e:
                        # feed the failure back to the phase; it Aborts next step
                        last_action, last_ok, last_err = action, False, str(e)
                        _ev("action_failed", phase=phase.name,
                            action=type(action).__name__, error=str(e))
                        continue
                    if isinstance(action, Takeoff):
                        airborne = True
                    elif isinstance(action, Land):
                        airborne = False
                    _ev("action_complete", phase=phase.name,
                        action=type(action).__name__, **_action_fields(action))
                    last_action, last_ok, last_err = action, True, None
                    continue
                summary["error"] = (f"phase {phase.name} returned non-flight "
                                    f"Action {action!r}")
                _ev("bad_action", phase=phase.name, action=repr(action))
                return summary
        if airborne:
            # Test 1 ends airborne (no LandOnPad) — land it.
            await _apply_action(adapter, Land(), command_timeout_s)
            _ev("final_land", reason="phases complete, was airborne")
            airborne = False
        summary["completed"] = True
        return summary
    finally:
        # props-off safe-down: best-effort, never raises (whitelisted).
        try:
            await adapter.emergency_land()
        except Exception as e:        # noqa: BLE001 — never-raise safe-down
            _ev("emergency_land_failed", error=f"{type(e).__name__}: {e}")


# ===========================================================================
# SDK-touching glue (lazy imports; only reached on a real/mock run)
# ===========================================================================
def _sightings_from_corners(corners, ids, *, drone_id, ts, frame_number,
                            frame_shape) -> List[Sighting]:
    """Build ArUco Sightings from a detectMarkers (corners, ids) result — the
    bus payload + LandOnPad input. numpy only here (the live path)."""
    import numpy as np
    out: List[Sighting] = []
    if ids is None:
        return out
    id_arr = np.asarray(ids).flatten()
    for marker_corners, marker_id in zip(corners, id_arr):
        pts = np.asarray(marker_corners).reshape(-1, 2)
        xs, ys = pts[:, 0], pts[:, 1]
        mid = int(marker_id)
        out.append(Sighting(
            drone_id=drone_id, ts=ts, source="aruco",
            class_name=f"aruco_{mid}", marker_id=mid,
            bbox_xyxy=(float(xs.min()), float(ys.min()),
                       float(xs.max()), float(ys.max())),
            confidence=1.0, frame_shape=frame_shape, frame_number=frame_number))
    return out


def probe_camera_tilt(api, tilt_deg, log, events, drone_id) -> bool:
    """Answer 'can we tilt the camera up/down?' at runtime, and command it when
    --camera-tilt-deg is given. The REAL pyhulax DroneAPI DOES expose
    set_camera_angle(CameraPitchMode, angle 0-90): the HULA main camera pitches
    0-90 deg up/down (confirmed on hardware 2026-06-11). FakeDroneAPI lacks it
    (probe NEGATIVE). Convention: tilt_deg > 0 = look DOWN, < 0 = look UP, 0 =
    straight ahead. Returns True iff the method exists."""
    setter = getattr(api, "set_camera_angle", None)
    if not callable(setter):
        log.line("CAMERA TILT: this DroneAPI has no set_camera_angle — camera "
                 "FIXED (probe NEGATIVE; e.g. the --mock FakeDroneAPI)")
        events.log(drone_id, "camera_tilt_probe", supported=False,
                   requested_deg=tilt_deg, moved=False)
        return False
    if tilt_deg is None:
        log.line("CAMERA TILT: set_camera_angle PRESENT — the HULA camera pitches "
                 "0-90 deg up/down (probe POSITIVE). Pass --camera-tilt-deg N "
                 "(N>0 = look DOWN, N<0 = look UP) to command it; NOT moved now.")
        events.log(drone_id, "camera_tilt_probe", supported=True,
                   requested_deg=None, moved=False)
        return True
    try:
        from pyhulax.core.types import CameraPitchMode
        deg = int(max(-90, min(90, tilt_deg)))
        if deg >= 0:
            mode, angle, facing = CameraPitchMode.DOWN_ABSOLUTE, deg, f"DOWN {deg} deg"
        else:
            mode, angle, facing = CameraPitchMode.UP_ABSOLUTE, -deg, f"UP {-deg} deg"
        setter(mode, angle)
        log.line(f"CAMERA TILT: commanded {facing} via set_camera_angle "
                 f"(probe POSITIVE)")
        events.log(drone_id, "camera_tilt_probe", supported=True,
                   requested_deg=tilt_deg, moved=True, facing=facing)
        return True
    except Exception as e:            # noqa: BLE001 — open SDK error set on a probe
        log.warn(f"CAMERA TILT: set_camera_angle present but command failed "
                 f"({type(e).__name__}: {e})")
        events.log(drone_id, "camera_tilt_probe", supported=True,
                   requested_deg=tilt_deg, moved=False, error=str(e))
        return True


def _build_depth(args, log):
    """Start the forward depth source (RealSense / fake / none). A depth failure
    is NON-fatal: log + continue depthless (the 2 Hz line shows 'depth n/a')."""
    backend = args.depth_backend
    if backend == "none":
        log.line("depth backend: none (no forward obstacle poll)")
        return None
    try:
        if backend == "fake":
            from finals.vision.depth import FakeDepthSource
            # alternate clear (1.5 m) / near (0.6 m) so both the 'clear' and the
            # 'OBJECT AHEAD' line are demonstrated under --mock.
            clear = [[1.5] * 8 for _ in range(6)]
            near = [[0.6] * 8 for _ in range(6)]
            src = FakeDepthSource("depth-fake", [clear, near], fps=8.0, loop=True)
        elif backend == "realsense":
            from finals.vision.depth import RealSenseDepthSource
            src = RealSenseDepthSource("depth-rs")
        else:
            raise FlightMonitorError(f"unknown --depth-backend {backend!r}")
        src.start(timeout_s=args.depth_timeout)
        log.line(f"depth backend {backend!r} started ({src.source_id})")
        return src
    except (SensorTimeout, SensorError, OSError, RuntimeError, ValueError,
            ImportError) as e:
        log.error(f"depth backend {backend!r} failed to start "
                  f"({type(e).__name__}: {e}) — continuing WITHOUT depth "
                  f"(2 Hz line shows 'depth n/a'). For RealSense check the USB "
                  f"sensor + pyrealsense2 install.")
        return None


def _emit_monitor(log, events, viewer, depth_src, drone_id) -> None:
    """One 2 Hz readback: telemetry + forward depth -> terminal + JSONL."""
    alt_m = yaw_deg = None
    is_flying = None
    try:
        t = viewer.adapter.telemetry()
        alt_m, yaw_deg, is_flying = t.altitude_m, t.yaw_deg, t.is_flying
    except FlightError as e:
        log.warn(f"telemetry read failed ({type(e).__name__}: {e})")
    depth_m = None
    if depth_src is not None:
        try:
            depth_m = nearest_object_m(depth_src.read())
        except (OSError, RuntimeError, ValueError) as e:
            log.warn(f"depth read failed ({type(e).__name__}: {e})")
    log.line(format_monitor_line(alt_m=alt_m, yaw_deg=yaw_deg,
                                 is_flying=is_flying, depth_m=depth_m))
    events.log(drone_id, "monitor_2hz", altitude_m=alt_m, yaw_deg=yaw_deg,
               is_flying=is_flying, depth_ahead_m=depth_m)


def _monitor_loop(log, events, viewer, depth_src, bus, drone_id, args,
                  abort_event, flight_done_event) -> int:
    """MAIN-thread loop: camera + perception + HUD window + 2 Hz prints + depth
    poll + publish ArUco sightings to the bus. Runs until the flight worker
    finishes (+ linger), abort, or --duration-s. Returns frames seen."""
    import cv2
    from finals.tools import live_view as lv

    window = not args.headless
    win = f"flight_monitor [{drone_id}] {args.test}  (q = abort)"
    if window:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        log.line("window open — press q to ABORT (safes down + lands)")
    start = time.monotonic()
    next_print = start
    seen = 0
    done_at: Optional[float] = None
    try:
        while True:
            now = time.monotonic()
            if abort_event.is_set():
                log.line("ABORT requested — closing monitor")
                break
            if flight_done_event.is_set():
                if done_at is None:
                    done_at = now
                    log.line("flight worker finished — lingering to show landing")
                elif now - done_at > _LINGER_S:
                    break
            if args.duration_s > 0 and now - start > args.duration_s:
                log.line("duration reached — closing monitor")
                break

            fs = lv._next_frame(log, viewer)
            if fs is not None:
                seen += 1
                corners, ids, boxes = viewer.detect(
                    fs.image, run_aruco=not args.no_aruco,
                    run_yolo=not args.no_yolo, yolo_conf=args.yolo_conf)
                if ids is not None:
                    fshape = (fs.image.shape[0], fs.image.shape[1])
                    for s in _sightings_from_corners(
                            corners, ids, drone_id=drone_id, ts=fs.ts,
                            frame_number=fs.frame_number, frame_shape=fshape):
                        seq = bus.publish(s)
                        events.log(drone_id, "broadcast",
                                   marker_id=s.marker_id, seq=seq)
                if window:
                    stats = lv._frame_stats(
                        viewer, fps=lv._fps(seen, start), frames=seen,
                        channel_order=viewer.channel_order,
                        aruco_on=not args.no_aruco, yolo_on=not args.no_yolo)
                    annotated = lv._annotate_frame(
                        viewer, fs.image, corners, ids, boxes, stats,
                        show_aruco=not args.no_aruco, show_yolo=not args.no_yolo,
                        show_votes=True, yolo_conf_bright=lv._YOLO_CONF_BRIGHT)
                    try:
                        cv2.imshow(win, annotated)
                    except cv2.error as e:
                        log.error(f"cv2.imshow failed ({type(e).__name__}: {e}) "
                                  f"— no display? rerun --headless")
                        window = False

            if now >= next_print:
                next_print += _MONITOR_PERIOD_S
                _emit_monitor(log, events, viewer, depth_src, drone_id)

            if window:
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    log.line("q pressed — ABORT")
                    abort_event.set()
            else:
                time.sleep(0.005)
    finally:
        if window:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
    return seen


def _flight_worker(adapter, phases, drone_id, bus, events, abort_event,
                   holder, done_event, budget_s, log) -> None:
    """WORKER thread: run the flight on its own asyncio loop over the shared
    (already connected) adapter."""
    try:
        if sys.platform == "win32":
            asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())
        holder["summary"] = asyncio.run(run_phases(
            adapter, phases, drone_id, bus=bus, events=events,
            abort_event=abort_event, mission_budget_s=budget_s))
    except Exception as e:            # noqa: BLE001 — worker must not die silently
        holder["error"] = f"{type(e).__name__}: {e}"
        log.exc("flight_worker")
    finally:
        done_event.set()


# ===========================================================================
# CLI
# ===========================================================================
def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog="flight_monitor",
        description="props-off instrumented flight + live monitor over one "
                    "pyhulax link")
    p.add_argument("--test", required=True, choices=("takeoff_land", "scan_land"),
                   help="takeoff_land = basic; scan_land = lawnmower + ArUco "
                        "broadcast + land_on_pad")
    p.add_argument("--ip", default=None, help="drone IP (skips discovery)")
    p.add_argument("--plane-id", type=int, default=None,
                   help="drone plane_id (label + discovery filter)")
    p.add_argument("--mock", action="store_true",
                   help="no hardware: FakeDroneAPI + synthetic marker frames + "
                        "fake depth (dev dry-run; needs cv2/numpy)")
    p.add_argument("--props-off-confirmed", action="store_true",
                   help="REQUIRED for a real drone: confirms the propellers are "
                        "removed (motors spin, no lift)")
    p.add_argument("--depth-backend", choices=("none", "fake", "realsense"),
                   default=None,
                   help="forward depth source (default: fake under --mock, else "
                        "realsense)")
    p.add_argument("--depth-timeout", type=float, default=10.0,
                   help="depth first-frame timeout s (default 10)")
    p.add_argument("--camera-tilt-deg", type=float, default=None,
                   help="command the HULA camera pitch (it DOES tilt 0-90 deg via "
                        "set_camera_angle): N>0 = look DOWN, N<0 = look UP, 0 = "
                        "ahead. Omit to only PROBE support without moving it.")
    p.add_argument("--mission-budget-s", type=float, default=_MISSION_BUDGET_S,
                   help=f"hard wall on the flight runner (default "
                        f"{_MISSION_BUDGET_S:.0f})")
    # perception / window (names shared with live_view's viewer + loop helpers)
    p.add_argument("--aruco-dict", default=_DEFAULT_ARUCO_DICT,
                   help=f"ArUco dict (default {_DEFAULT_ARUCO_DICT}, the field "
                        f"dict)")
    p.add_argument("--all-dicts", action="store_true",
                   help="scan all candidate dicts (bring-up sweep)")
    p.add_argument("--no-aruco", action="store_true", help="disable ArUco overlay")
    p.add_argument("--no-yolo", action="store_true", help="disable YOLO overlay")
    p.add_argument("--yolo-conf", type=float, default=0.25,
                   help="YOLO confidence threshold (default 0.25)")
    p.add_argument("--weights", default=None,
                   help="YOLO .pt path (default: auto-detect)")
    p.add_argument("--fake-marker-id", type=int, default=11,
                   help="--mock synthetic marker id (default 11, a field id)")
    p.add_argument("--headless", "--no-window", dest="headless",
                   action="store_true",
                   help="no cv2 window (SSH/CI); still prints 2 Hz + broadcasts")
    p.add_argument("--duration-s", type=float, default=0.0,
                   help="hard cap on the monitor window (0 = until flight done)")
    p.add_argument("--frames", type=int, default=0, help="(reserved)")
    p.add_argument("--no-abort-key", action="store_true",
                   help="do not arm the stdin 'q' AbortListener")
    p.add_argument("--connect-timeout", type=float, default=15.0,
                   help="per-drone connect timeout s (default 15)")
    p.add_argument("--video-timeout", type=float, default=25.0,
                   help="video first-frame timeout s (default 25)")
    p.add_argument("--discover-secs", type=float, default=15.0,
                   help="Dola discovery window when no --ip (default 15)")
    p.add_argument("--out", default=None, help="output dir (default runs\\...)")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)

    # props-off safety gate (real drone only).
    if not args.mock and not args.props_off_confirmed:
        print("REFUSING: this issues real flight commands. Remove the propellers "
              "and pass --props-off-confirmed (or --mock for a no-hardware "
              "dry-run). Motors spin; without props there is no lift.",
              file=sys.stderr)
        return 2
    if not args.mock and not args.ip and args.plane_id is None:
        print("REFUSING: real run needs --ip (or --plane-id for discovery).",
              file=sys.stderr)
        return 2

    # default depth backend: fake under --mock, realsense on the bench.
    if args.depth_backend is None:
        args.depth_backend = "fake" if args.mock else "realsense"
    # live_view._build_viewer keys off args.fake; --mock IS its fake path.
    args.fake = args.mock
    if not hasattr(args, "snapshot_on_exit"):
        args.snapshot_on_exit = False

    # lazy SDK-side imports (kept out of module top so the bare venv stays clean).
    from finals.tools import live_view as lv
    from finals.tools import hula_smoke as hs

    import os
    ts = time.strftime("%Y%m%dT%H%M%S")
    outdir = args.out or os.path.join("runs", f"flight_monitor_{ts}")
    os.makedirs(outdir, exist_ok=True)
    log = lv._Log(os.path.join(outdir, "flight_monitor.log"))
    events = EventLog(outdir)

    log.line(f"flight_monitor  test={args.test}  "
             f"{'MOCK (no hardware)' if args.mock else 'REAL props-off'}")
    log.line("HONEST SCOPE: props off proves command/telemetry/perception/"
             "broadcast/landing-servo plumbing — NOT motion, climb, or touchdown.")

    weights = args.weights or hs._find_weights()
    viewer = None
    depth_src = None
    abort_listener = None
    rc = 0
    try:
        # connect + start video + build detectors (no flight) — main thread.
        viewer = lv._build_viewer(log, args, weights)
        if viewer is None:
            log.error("could not bring up the drone link / video — aborting")
            return 3
        drone_id = viewer.drone_id

        # answer the camera-tilt question at runtime.
        probe_camera_tilt(getattr(viewer.adapter, "_api", None),
                          args.camera_tilt_deg, log, events, drone_id)

        depth_src = _build_depth(args, log)
        bus = SightingBus()
        phases = build_test_phases(args.test, takeoff_cm=_DEFAULT_TAKEOFF_CM)

        abort_event = threading.Event()
        if not args.no_abort_key:
            from finals.guards import AbortListener
            abort_listener = AbortListener(abort_event)
            try:
                abort_listener.start()
            except RuntimeError as e:
                log.warn(f"abort key not armed ({e}); use the window 'q'")
                abort_listener = None

        holder: dict = {}
        flight_done = threading.Event()
        worker = threading.Thread(
            target=_flight_worker,
            args=(viewer.adapter, phases, drone_id, bus, events, abort_event,
                  holder, flight_done, args.mission_budget_s, log),
            name="flight-worker", daemon=True)
        worker.start()

        seen = _monitor_loop(log, events, viewer, depth_src, bus, drone_id, args,
                             abort_event, flight_done)

        worker.join(timeout=max(10.0, args.mission_budget_s + 10.0))
        if worker.is_alive():
            log.error("flight worker did not finish — abort + safe down")
            abort_event.set()
            worker.join(timeout=15.0)

        summary = holder.get("summary")
        err = holder.get("error")
        if err:
            log.error(f"flight worker error: {err}")
            rc = 1
        elif summary is not None:
            log.line(f"flight summary: completed={summary['completed']} "
                     f"aborted={summary['aborted']} error={summary['error']}")
            for ph in summary["phases"]:
                log.line(f"  phase {ph['phase']}: {ph['result']} — {ph['reason']}")
            if summary["aborted"] or summary["error"]:
                rc = 1
        log.line(f"frames shown: {seen}")
    finally:
        if depth_src is not None:
            try:
                depth_src.stop()
            except (OSError, RuntimeError, ValueError):
                pass
        if abort_listener is not None:
            abort_listener.stop()
        if viewer is not None:
            viewer.teardown()
        events.close()
        log.line(f"PASTE BACK: {os.path.join(outdir, 'flight_monitor.log')}")
        log.close()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
