"""Preflight: the ordered P0-P10 gate before any flight + the standalone bench
tool (`python -m finals.main --profile bench --preflight-only`).

Surface (S10):
- run_preflight(profile, agents, cfg, *, sources, events, run_dir, ...) ->
  list[CheckResult]. Gates run IN ORDER; each prints one line + logs a
  `preflight_gate` event; the whole run logs one `preflight` event and persists
  runs/<ts>/preflight.json. The FIRST CRITICAL failure tears the link down
  (stop started video, disconnect every adapter) and raises PreflightError ->
  main maps it to exit 3. Non-critical gates record a WARN result and continue.
- On SUCCESS in the mission path, adapters are left CONNECTED and video sources
  STARTED for finals.main._amain — this is where the S9-deferred
  discovery->ip->connect()-BEFORE-stream.start() ordering is honoured (P3 sets
  each adapter's IP via PyhulaxAdapter.set_target_ip, P4 connects, P6 starts the
  shared-DroneAPI stream — only possible once P4 connected the api). With
  preflight_only=True the gates instead tear down at the end (the bench tool
  never flies, so it leaves the fleet idle).

Gates, in order (CRITICAL unless noted):
  P0 config sanity (bench/real, plane_ids set+DISTINCT, bands distinct, frame
     backend pyhulax) -> P1 run_dir writable + fsync probe -> P2 perception
     readiness (marker detector builds; YOLO weights present if enabled) ->
     P3 Dola discovery finds EXACTLY the expected plane_ids, IPs applied ->
     P4 per-drone connect -> P5 telemetry sane (battery >= floor, fresh,
     on-ground) -> P6 video fresh (stream starts, first frame, healthy) ->
     P7 marker detect on a live frame + projected tick-load advisory (WARN
     only — a slow laptop is surfaced, not a hard block) -> P8 UWB serial
     (only if use_uwb; finals = skipped) -> P9 safety (identity LED per drone;
     battery failsafe was enabled at P4 connect) -> P10 operator types literal
     GO within go_timeout_s, DEFAULT-DENY (skipped when preflight_only).

P10 fixes mapping_drone.py:318-327, where an invalid answer fell THROUGH to
arming (and sys was never imported): here anything but exactly "GO" — including
a timeout — is a refusal.

This module is PURE: it imports NO SDK at the top level (it is not in
test_conventions.py SDK_ALLOWED). Every SDK touch goes through the injected
adapter/source/api; discovery and the marker detector are imported lazily
inside their gates (or injected by tests). Gate bodies raise only TYPED errors
(PreflightError/FlightError/SensorError/SensorTimeout/ConfigError/OSError/
ImportError); an unexpected exception is a real bug and propagates loud.

Derives from: mapping_drone.py confirm-prompt (audited), docs/quali/
deployment.md pre-run checklist, dola.py discovery. Session: S10.
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
import time
from dataclasses import asdict, dataclass, field
from typing import (Awaitable, Callable, Dict, List, Optional, Sequence, Tuple)

from finals.errors import (ConfigError, FlightError, PreflightError,
                           SensorError, SensorTimeout)
from finals.events import EventLogError

#: Wall-clock window for the video stream's None startup gap (P6); the pyhulax
#: stream takes ~1-2 s to deliver the first frame.
_VIDEO_START_TIMEOUT_S = 10.0
#: On-ground altitude tolerance (P5): a drone that already thinks it is up is a
#: telemetry/zeroing fault to catch on the bench, not in the air.
_ON_GROUND_ALT_TOL_M = 0.5
#: Bounded teardown deadline per adapter on the abort path.
_DISCONNECT_TIMEOUT_S = 6.0

#: Typed errors a gate body may raise to mean "this check failed" (vs a wiring
#: bug, which is NOT in this tuple and so propagates loud — fail-loud).
_GATE_ERRORS = (PreflightError, FlightError, SensorError, SensorTimeout,
                ConfigError, OSError, ImportError)


@dataclass
class CheckResult:
    """One gate's verdict. `critical` gates abort the run on failure; non-critical
    ones record a WARN and continue. `data` carries gate-specific facts for the
    persisted preflight.json (IPs, battery levels, detect timings)."""
    id: str
    name: str
    ok: bool
    critical: bool
    detail: str
    elapsed_s: float = 0.0
    data: dict = field(default_factory=dict)


# ============================================================
# Default seams (lazy — keep this module SDK-free at import time)
# ============================================================
def _default_discover(plane_ids: Sequence[int], timeout_s: float) -> Dict[int, str]:
    from finals.flight.discovery import discover_required
    return discover_required(plane_ids, timeout_s)


def _stdin_go_reader() -> str:
    return input("\n  >>> All P0-P9 passed. Type GO to authorize flight: ")


def _adapter_of(agent) -> object:
    """The agent's FlightAdapter (main's own object; no public accessor)."""
    return agent._adapter


def _pyhulax_leaf(adapter) -> object:
    """The PyhulaxAdapter under a BenchAdapter wrap (set_target_ip lives on the
    leaf), or the adapter itself on the real profile."""
    return getattr(adapter, "inner", adapter)


# ============================================================
# Entry
# ============================================================
async def run_preflight(
        profile: str, agents: Sequence, cfg, *,
        sources: Sequence = (),
        events=None,
        run_dir: Optional[str] = None,
        discover_fn: Optional[Callable[[Sequence[int], float], Dict[int, str]]] = None,
        marker_detector: Optional[Callable] = None,
        confirm_fn: Optional[Callable[[], str]] = None,
        go_timeout_s: float = 60.0,
        preflight_only: bool = False) -> List[CheckResult]:
    """Run the P0-P10 gate. See the module docstring for the contract."""
    if profile not in ("bench", "real"):
        raise PreflightError(
            f"preflight is the bench/real gate; profile {profile!r} has none "
            f"(mock/sitl log a preflight-skipped event in main instead)")

    results: List[CheckResult] = []
    started_sources: List = []
    # Built in P2, reused in P7 (one cv2 detector instance, the aruco.py
    # contract); an injected detector wins (keeps tests cv2-free).
    state = {"marker_detector": marker_detector}

    async def _gate(gid: str, name: str, critical: bool,
                    body: Callable[[], Awaitable[Tuple[bool, str, dict]]]) -> None:
        t0 = time.monotonic()
        try:
            ok, detail, data = await body()
        except _GATE_ERRORS as e:
            ok, detail, data = False, f"{type(e).__name__}: {e}", {}
        cr = CheckResult(gid, name, ok, critical, detail,
                         round(time.monotonic() - t0, 3), data)
        results.append(cr)
        _emit(cr, events)
        if not ok and critical:
            await _teardown(agents, started_sources)
            _persist(run_dir, results)
            if events is not None:
                _log_summary(events, results, status="failed")
            raise PreflightError(
                f"preflight {cr.id} ({cr.name}) FAILED — {cr.detail}. "
                f"Fleet safed down (disconnected, video stopped); refusing to "
                f"fly. Fix and re-run.")

    await _gate("P0", "config sanity", True, lambda: _p0_config(cfg))
    await _gate("P1", "log dir writable", True, lambda: _p1_logdir(run_dir))
    await _gate("P2", "perception readiness", True, lambda: _p2_perception(cfg, state))
    await _gate("P3", "discovery", True,
                lambda: _p3_discovery(cfg, agents, discover_fn))
    await _gate("P4", "connect", True, lambda: _p4_connect(cfg, agents))
    await _gate("P5", "telemetry sane", True, lambda: _p5_telemetry(cfg, agents))
    await _gate("P6", "video fresh", True,
                lambda: _p6_video(sources, started_sources))
    await _gate("P7", "detect + tick load", False,
                lambda: _p7_detect(cfg, sources, state))
    await _gate("P8", "uwb serial", True, lambda: _p8_uwb(cfg))
    await _gate("P9", "safety systems", True, lambda: _p9_safety(cfg, agents))
    await _gate("P10", "operator GO", True,
                lambda: _p10_operator_go(preflight_only, confirm_fn, go_timeout_s))

    _persist(run_dir, results)
    if events is not None:
        _log_summary(events, results, status="passed")
    if preflight_only:
        # The bench tool never flies: connect -> check -> DISCONNECT, leaving
        # the fleet idle. The mission path instead keeps the link + stream up.
        await _teardown(agents, started_sources)
    return results


# ============================================================
# Gates — each returns (ok, detail, data); raises only _GATE_ERRORS on failure
# ============================================================
async def _p0_config(cfg) -> Tuple[bool, str, dict]:
    problems: List[str] = []
    if cfg.profile not in ("bench", "real"):
        problems.append(f"profile {cfg.profile!r} not bench/real")
    plane_ids = [d.plane_id for d in cfg.drones]
    if any(p is None for p in plane_ids):
        problems.append("a drone has no plane_id (Dola key)")
    elif len(set(plane_ids)) != len(plane_ids):
        problems.append(f"duplicate plane_ids {plane_ids} — discovery cannot "
                        f"tell two drones apart")
    bands = [d.altitude_band_m for d in cfg.drones]
    if len(cfg.drones) > 1 and (None in bands or len(set(bands)) != len(bands)):
        problems.append(f"altitude bands not distinct {bands} (the collision "
                        f"guarantee)")
    if cfg.frame_backend != "pyhulax":
        problems.append(f"frame_backend {cfg.frame_backend!r} != 'pyhulax' "
                        f"(live video unwired)")
    data = {"plane_ids": plane_ids, "bands": bands, "n_drones": len(cfg.drones)}
    if problems:
        return False, "; ".join(problems), data
    return (True,
            f"{len(cfg.drones)} drone(s); plane_ids {plane_ids}; bands {bands}",
            data)


async def _p1_logdir(run_dir: Optional[str]) -> Tuple[bool, str, dict]:
    if run_dir is None:
        return True, "no run_dir — persistence skipped (in-memory run)", {}
    probe = os.path.join(run_dir, ".preflight_write_probe")
    with open(probe, "w", encoding="utf-8") as f:
        f.write("ok")
        f.flush()
        os.fsync(f.fileno())
    os.remove(probe)
    return True, f"{run_dir} writable + fsync ok", {"run_dir": run_dir}


async def _p2_perception(cfg, state: dict) -> Tuple[bool, str, dict]:
    if state["marker_detector"] is None:
        # Lazy: importing aruco pulls cv2 (whitelisted there, NOT here). A
        # missing cv2 surfaces as a clean ImportError gate failure.
        from finals.vision.aruco import make_marker_detector
        # PAD-DICT: build the detector over the REAL marker_dict + whitelisted
        # DetectorParameters so P2 proves the configured dictionary resolves on
        # this cv2 (a bad name fails the gate, props off — never mid-flight).
        state["marker_detector"] = make_marker_detector(
            cfg.marker_backend, marker_dict=cfg.marker_dict,
            aruco_detector_params=cfg.aruco_detector_params)
    detail = (f"marker detector '{cfg.marker_backend}' built (primary"
              f"{', dict ' + cfg.marker_dict if cfg.marker_backend == 'aruco' else ''})")
    data = {"marker_backend": cfg.marker_backend,
            "marker_dict": cfg.marker_dict,
            "detector_backend": cfg.detector.backend}
    if cfg.detector.backend == "ultralytics":
        # load_config already proved the weights file exists + is non-COCO; a
        # full smoke inference is deferred to P7's live frame.
        detail += f"; YOLO weights {os.path.basename(cfg.detector.weights)}"
        data["weights"] = cfg.detector.weights
    return True, detail, data


async def _p3_discovery(cfg, agents, discover_fn) -> Tuple[bool, str, dict]:
    df = discover_fn or _default_discover
    plane_ids = [d.plane_id for d in cfg.drones]
    ips = df(plane_ids, cfg.discovery_timeout_s)   # PreflightError names the gap
    # Apply the resolved IP to each adapter BEFORE connect (P4). zip is safe:
    # main builds agents in cfg.drones order (asserted by drone_id below).
    for drone, agent in zip(cfg.drones, agents):
        if agent.drone_id != drone.id:
            raise PreflightError(
                f"preflight wiring: agent {agent.drone_id!r} != config drone "
                f"{drone.id!r} (order mismatch in main)")
        _pyhulax_leaf(_adapter_of(agent)).set_target_ip(ips[drone.plane_id])
    return True, f"found {len(ips)} plane(s): {ips}", {"ips": ips}


async def _p4_connect(cfg, agents) -> Tuple[bool, str, dict]:
    connected: List[str] = []
    for agent in agents:
        await _adapter_of(agent).connect(timeout_s=cfg.command_timeout_s)
        connected.append(agent.drone_id)
    return True, f"connected: {connected}", {"connected": connected}


async def _p5_telemetry(cfg, agents) -> Tuple[bool, str, dict]:
    now = time.monotonic()
    problems: List[str] = []
    data: dict = {}
    stale_s = cfg.guards.telemetry_stale_s
    for agent in agents:
        t = _adapter_of(agent).telemetry()       # FlightError if not connected
        age = t.age_s(now)
        data[agent.drone_id] = {
            "battery_pct": t.battery_pct,
            "altitude_m": t.altitude_m,
            "telemetry_age_s": round(age, 2),
        }
        if t.battery_pct is None or t.battery_pct < cfg.min_battery_pct:
            problems.append(f"{agent.drone_id} battery {t.battery_pct} < "
                            f"floor {cfg.min_battery_pct}")
        if age > stale_s:
            problems.append(f"{agent.drone_id} telemetry STALE "
                            f"{age:.1f} s > {stale_s:.1f} s")
        if t.altitude_m is not None and abs(t.altitude_m) > _ON_GROUND_ALT_TOL_M:
            problems.append(f"{agent.drone_id} not on ground "
                            f"(alt {t.altitude_m:.2f} m)")
    if problems:
        return False, "; ".join(problems), data
    return True, "battery/freshness/on-ground OK for all drones", data


async def _p6_video(sources, started_sources: List) -> Tuple[bool, str, dict]:
    if not sources:
        return True, "no video sources wired (skipped)", {}
    for source in sources:
        source.start(timeout_s=_VIDEO_START_TIMEOUT_S)   # SensorTimeout/Error
        started_sources.append(source)                   # so teardown stops it
        if not source.healthy:
            return False, (f"{source.source_id} started but is not healthy "
                           f"(no frame progress) — check the camera/link"), {}
    return (True, f"video healthy: {[s.source_id for s in sources]}",
            {"sources": [s.source_id for s in sources]})


async def _p7_detect(cfg, sources, state: dict) -> Tuple[bool, str, dict]:
    md = state["marker_detector"]
    if md is None or not sources:
        return True, "no detector/sources to time (skipped)", {}
    timings_ms: List[float] = []
    tested: List[str] = []
    for source in sources:
        frame = source.get_frame()
        if frame is None:
            continue
        t0 = time.perf_counter()
        md(frame, source.source_id)
        timings_ms.append((time.perf_counter() - t0) * 1000.0)
        tested.append(source.source_id)
    if not timings_ms:
        return True, "no frame available to time detection (P6 gated health)", {}
    worst_ms = max(timings_ms)
    # Projected wall-clock fraction: worst per-frame detect time x drones x the
    # per-drone sample rate. > 1.0 means the laptop cannot keep up at tick_hz.
    projected = (worst_ms / 1000.0) * len(sources) * cfg.tick_hz
    data = {"detect_ms_worst": round(worst_ms, 1),
            "projected_load": round(projected, 2),
            "tested": tested, "n_drones": len(sources),
            "tick_hz": cfg.tick_hz}
    detail = (f"detect {worst_ms:.0f} ms/frame worst; projected load "
              f"{projected:.2f}x at {len(sources)}x{cfg.tick_hz:g} Hz")
    if projected > 1.0:
        detail += "  WARN laptop may not keep up — lower tick_hz or shed detection"
    return True, detail, data


async def _p8_uwb(cfg) -> Tuple[bool, str, dict]:
    if not cfg.use_uwb:
        return True, "use_uwb=false — UWB not used (finals swarm)", {"skipped": True}
    if not cfg.uwb_serial_port:
        return False, "use_uwb=true but no uwb_serial_port configured", {}
    return True, f"UWB serial {cfg.uwb_serial_port} configured", \
        {"uwb_serial_port": cfg.uwb_serial_port}


async def _p9_safety(cfg, agents) -> Tuple[bool, str, dict]:
    led_set: List[str] = []
    for drone, agent in zip(cfg.drones, agents):
        if drone.led_rgb is not None:
            await _adapter_of(agent).set_led(*drone.led_rgb)
            led_set.append(f"{drone.id}={drone.led_rgb}")
    return (True,
            f"identity LED set: {led_set or 'none configured'}; battery "
            f"failsafe enabled at connect (P4)",
            {"led_set": led_set})


async def _p10_operator_go(preflight_only: bool, confirm_fn, go_timeout_s
                           ) -> Tuple[bool, str, dict]:
    if preflight_only:
        return True, "skipped (--preflight-only does not fly)", {"skipped": True}
    reader = confirm_fn or _stdin_go_reader
    loop = asyncio.get_running_loop()
    try:
        answer = await asyncio.wait_for(
            loop.run_in_executor(None, reader), go_timeout_s)
    except asyncio.TimeoutError:
        return False, (f"operator did not type GO within {go_timeout_s:.0f} s "
                       f"— DEFAULT-DENY (no arm)"), {}
    answer = (answer or "").strip()
    if answer != "GO":
        return False, (f"operator did not confirm (got {answer!r}, need exactly "
                       f"'GO') — DEFAULT-DENY"), {}
    return True, "operator authorized: GO", {}


# ============================================================
# Reporting + teardown
# ============================================================
def _emit(cr: CheckResult, events) -> None:
    tag = "PASS" if cr.ok else ("FAIL" if cr.critical else "WARN")
    print(f"  [{cr.id:<3}] {cr.name:<22} {tag:<4} ({cr.elapsed_s:.2f}s)  "
          f"{cr.detail}", file=sys.stderr, flush=True)
    if events is not None:
        try:
            events.log("mission", "preflight_gate", id=cr.id, name=cr.name,
                       ok=cr.ok, critical=cr.critical, detail=cr.detail,
                       elapsed_s=cr.elapsed_s)
        except EventLogError as e:
            # A log-write failure must never abort a safety gate (forensics
            # yield to the gate); surfaced on stderr, never swallowed silently.
            print(f"[preflight] WARNING: could not log {cr.id}: {e}",
                  file=sys.stderr, flush=True)


def _log_summary(events, results: List[CheckResult], *, status: str) -> None:
    try:
        events.log("mission", "preflight", status=status,
                   gates=[{"id": r.id, "ok": r.ok, "critical": r.critical}
                          for r in results])
    except EventLogError as e:
        print(f"[preflight] WARNING: could not log summary: {e}",
              file=sys.stderr, flush=True)


def _persist(run_dir: Optional[str], results: List[CheckResult]) -> None:
    if run_dir is None:
        return
    path = os.path.join(run_dir, "preflight.json")
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump([asdict(r) for r in results], f, indent=2, default=str)
            f.flush()
            os.fsync(f.fileno())
    except OSError as e:
        print(f"[preflight] WARNING: could not write {path}: {e}",
              file=sys.stderr, flush=True)


async def _teardown(agents, started_sources) -> None:
    """Safe-down on abort / after a preflight-only run: stop started video,
    disconnect every adapter. Bounded; never raises (both contracts are
    never-raise, the waits are belt-and-suspenders)."""
    for source in started_sources:
        try:
            source.stop()                        # idempotent, never raises
        except (SensorError, OSError) as e:
            print(f"[preflight] teardown: {source.source_id} stop: {e}",
                  file=sys.stderr, flush=True)
    for agent in agents:
        try:
            await asyncio.wait_for(_adapter_of(agent).disconnect(),
                                   _DISCONNECT_TIMEOUT_S)
        except (asyncio.TimeoutError, FlightError) as e:
            print(f"[preflight] teardown: {agent.drone_id} disconnect: {e}",
                  file=sys.stderr, flush=True)
