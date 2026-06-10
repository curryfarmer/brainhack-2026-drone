"""Finals C2 entry point — the composition root.

This is the ONLY file that imports concrete backends, and it imports each one
lazily inside its builder so a missing SDK fails loudly at wiring time but
only for the backend actually selected.

Usage (run from repo root):
  python -m finals.main --profile mock --dry-run
  python -m finals.main --profile sitl --phases takeoff_demo --budget 120
  python -m finals.main --profile replay
  python -m finals.main --profile bench --preflight-only   # onsite bench tool
  python -m finals.main --profile bench
  python -m finals.main --profile real --i-know-this-arms-real-drones

Exit codes: 0 ok | 1 unexpected error / any drone FAILED | 2 config error |
3 preflight failure.

Session: S1 (CLI + config + wiring resolution + --dry-run); S4 (flight path:
run dir + crash hooks + EventLog + per-drone adapter/phase/agent wiring ->
preflight gate -> Orchestrator.run -> exit code); S7 (vision: the no-drone
replay runner below, plus per-drone perception + VideoWatchdog wiring on
flight profiles whose frame backend is in _WIRED_FRAME_BACKENDS).

Wiring notes (binding):
- BENCH SPECIAL CASE: BenchAdapter wraps an INNER backend, so the generic
  flight_cls(drone_id) construction does not fit it — _build_adapter builds
  the inner backend (PyhulaxAdapter, S9) first and wraps it (see the
  BenchAdapter docstring).
- Phase construction soft convention: a phase class MAY define
  from_config(drone_cfg, cfg) (TakeoffDemo does — zone tunables + altitude
  band); phases without it are built with no arguments.
- Preflight gate (S10): bench/real run finals.preflight.run_preflight (the
  ordered P0-P10 gate) — it OWNS adapter.connect (P4) AND video source.start
  (P6), leaving adapters connected + sources started for the orchestrator (the
  S9-deferred connect-before-stream-start ordering). `--preflight-only` runs
  P0-P9 standalone (no flight) as the onsite bench tool. mock/sitl record a
  loud preflight-skipped event instead (nothing to gate in pure software).
- VISION GATE (S7): perception + VideoWatchdog are wired ONLY for frame
  backends in _WIRED_FRAME_BACKENDS (replay + pyhulax, S10) — deliberately
  NARROWER than frame_backend != "none": sitl.json ships frame_backend
  "gazebo" with a drone TODAY (S8 wires it), and gating on != "none" would
  either crash on the GazeboRgbSource stub or hand agents a VideoWatchdog
  with no frame source (the guaranteed-false "no frame EVER" DEGRADE of
  guards.py reconciliation 5) on every SIM run.
- Sighting CSV ownership (S7, binding for S8): sightings.csv rows are
  appended at the PUBLISH site (PerceptionLoop / the detection callback) —
  see finals/vision/perception.py. The bus drains here and in the
  orchestrator are event mirrors only; S8 wires gazebo by adding it to
  _WIRED_FRAME_BACKENDS + a source branch in _build_perception, with NO
  orchestrator changes.

Derives from: qualifier_run.py parse_args/_amain (CLI override conventions:
--weights/--budget/--no-detector/--display kept compatible).
"""
from __future__ import annotations

import argparse
import asyncio
import os
import sys
import threading
import time
import traceback
from typing import List, Optional, Sequence, Tuple, Type

from finals.config import DroneConfig, FinalsConfig, load_config
from finals.errors import ConfigError, PreflightError
from finals.events import (EventLog, EventLogError, create_run_dir,
                           install_crash_hooks)
from finals.guards import (AbortListener, BatteryGuard, GeofenceLite, Guard,
                           LoopOverrunGuard, MissionClockGuard, PhaseTimeout,
                           ProximityGuard, SafetyController, SectorGuard,
                           TelemetryWatchdog, VideoWatchdog)
from typing import TYPE_CHECKING

from finals.mission.agent import DroneAgent
from finals.mission.orchestrator import Orchestrator
from finals.mission.phase import MissionPhase
from finals.mission.phases import resolve_phase

if TYPE_CHECKING:  # type-only: built lazily in _build_convoy_registry/_build_obstacle_map
    from finals.mission.convoy_registry import ConvoyRegistry
    from finals.mission.obstacle_map import ObstacleMap
from finals.sightings import SightingBus, SightingLog

_CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs")

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_CONFIG = 2
EXIT_PREFLIGHT = 3


# ============================================================
# CLI
# ============================================================
def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="finals.main",
        description="BrainHack 2026 finals C2 runner (swarm challenge)",
    )
    p.add_argument("--profile", required=True,
                   choices=("mock", "sitl", "replay", "bench", "real"),
                   help="execution profile; selects finals/configs/<profile>.json")
    p.add_argument("--config", help="explicit config path (overrides the profile default)")
    p.add_argument("--weights", help="detector weights override (forces backend=ultralytics)")
    p.add_argument("--phases", help="comma-separated phase names forced onto ALL drones")
    p.add_argument("--budget", type=float, help="mission wall-clock budget override (s)")
    p.add_argument("--no-detector", action="store_true", help="disable detection entirely")
    p.add_argument("--display", action="store_true", help="show the detection window (debug)")
    p.add_argument("--dry-run", action="store_true",
                   help="load config, resolve backends + phases, print the plan, exit 0")
    p.add_argument("--preflight-only", action="store_true",
                   help="bench/real ONLY: run the preflight P0-P9 gate (no "
                        "operator GO, no orchestrator, no flight) and exit — "
                        "the primary onsite bench tool (exit 0 pass / 3 fail)")
    p.add_argument("--i-know-this-arms-real-drones", action="store_true",
                   help="required confirmation gate for --profile real")
    return p.parse_args(argv)


# ============================================================
# Backend builders — lazy imports, one per backend
# ============================================================
def resolve_flight_adapter_cls(backend: str) -> Optional[Type]:
    """flight_backend name -> adapter class (None for 'none'). ConfigError on unknown."""
    if backend == "none":
        return None
    if backend == "mock":
        from finals.flight.mock_adapter import MockAdapter
        return MockAdapter
    if backend == "mavsdk_sitl":
        from finals.flight.sitl_adapter import MavsdkSitlAdapter
        return MavsdkSitlAdapter
    if backend == "pyhulax":
        from finals.flight.pyhulax_adapter import PyhulaxAdapter
        return PyhulaxAdapter
    if backend == "bench":
        from finals.flight.adapter import BenchAdapter
        return BenchAdapter
    raise ConfigError(
        f"unknown flight_backend {backend!r} — one of: none, mock, mavsdk_sitl, pyhulax, bench"
    )


def resolve_video_source_cls(backend: str) -> Optional[Type]:
    """frame_backend name -> VideoSource class (None for 'none'). ConfigError on unknown."""
    if backend == "none":
        return None
    if backend == "replay":
        from finals.vision.video import ReplaySource
        return ReplaySource
    if backend == "gazebo":
        from finals.vision.gazebo_video import GazeboRgbSource
        return GazeboRgbSource
    if backend == "pyhulax":
        from finals.vision.pyhulax_video import PyhulaxVideoSource
        return PyhulaxVideoSource
    raise ConfigError(
        f"unknown frame_backend {backend!r} — one of: none, replay, gazebo, pyhulax"
    )


# ============================================================
# Dry-run report
# ============================================================
def format_resolved_plan(cfg: FinalsConfig, flight_cls: Optional[Type],
                         video_cls: Optional[Type], config_path: str) -> str:
    lines = [
        "=" * 72,
        f"RESOLVED PLAN  profile={cfg.profile}  config={config_path}",
        "=" * 72,
        f"flight_backend : {cfg.flight_backend:<12} -> {flight_cls.__name__ if flight_cls else '(none)'}",
        f"frame_backend  : {cfg.frame_backend:<12} -> {video_cls.__name__ if video_cls else '(none)'}",
        f"depth_backend  : {cfg.depth_backend:<12} -> "
        + ("(none — monocular)" if cfg.depth_backend == "none"
           else resolve_depth_source_cls(cfg.depth_backend).__name__),
        f"IR proximity   : "
        + (f"ProximityGuard ON (warn {cfg.guards.proximity_warn_cm:g} cm / land "
           f"{cfg.guards.proximity_land_cm:g} cm; LIVE read = onsite gate, "
           f"synthetic feed wired)" if cfg.guards.proximity_enable
           else "(off — guards.proximity_enable false)"),
        "detection      : "
        + ("(no frame source — perception off)" if cfg.frame_backend == "none" else
           f"{cfg.marker_backend} (primary, always on) + yolo: " + cfg.detector.backend
           + (f"  weights={cfg.detector.weights}  conf={cfg.detector.conf}  device={cfg.detector.device}"
              if cfg.detector.backend == "ultralytics" else "")),
        f"budget         : {cfg.mission_budget_s:.0f} s   tick: {cfg.tick_hz:.0f} Hz   "
        f"cmd timeout: {cfg.command_timeout_s:.0f} s   battery floor: {cfg.min_battery_pct:.0f} %",
        f"run_dir        : {cfg.run_dir}",
    ]
    if cfg.frame_backend == "replay":
        lines.append(f"replay_dir     : {cfg.replay_dir}  ({cfg.replay_fps:g} fps)")
    if cfg.use_uwb:
        lines.append(f"uwb            : serial {cfg.uwb_serial_port}")
    lines.append("-" * 72)
    if not cfg.drones:
        lines.append("drones         : (none — laptop-only profile)")
    for d in cfg.drones:
        lines.append(
            f"drone {d.id:<8} plane_id={d.plane_id!s:<5} band={d.altitude_band_m!s:<5} "
            f"led={d.led_rgb!s:<16} phases={d.phases}"
        )
    lines.append("=" * 72)
    return "\n".join(lines)


# ============================================================
# Entry
# ============================================================
def run(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    if args.profile == "real" and not args.i_know_this_arms_real_drones:
        raise ConfigError(
            "refusing --profile real without --i-know-this-arms-real-drones. "
            "This profile arms physical aircraft; the flag is the first of "
            "three gates (flag -> preflight P0-P10 -> operator GO)."
        )

    config_path = args.config or os.path.join(_CONFIG_DIR, f"{args.profile}.json")
    overrides = {}
    if args.weights:
        overrides["weights"] = args.weights
    if args.budget is not None:
        overrides["budget_s"] = args.budget
    if args.phases:
        overrides["phases"] = [p.strip() for p in args.phases.split(",") if p.strip()]
    if args.no_detector:
        overrides["no_detector"] = True
    if args.display:
        overrides["display"] = True

    cfg = load_config(config_path, overrides)
    if cfg.profile != args.profile:
        raise ConfigError(
            f"--profile {args.profile} but {config_path} declares profile "
            f"{cfg.profile!r} — wrong file? Pass --config explicitly or fix the JSON."
        )

    # Resolve EVERYTHING before doing anything: unknown names die here, loudly.
    flight_cls = resolve_flight_adapter_cls(cfg.flight_backend)
    video_cls = resolve_video_source_cls(cfg.frame_backend)
    resolve_depth_source_cls(cfg.depth_backend)   # SENSE-IR: unknown -> loud here
    for d in cfg.drones:
        for phase_name in d.phases:
            resolve_phase(phase_name)  # ConfigError lists available phases

    if args.dry_run:
        print(format_resolved_plan(cfg, flight_cls, video_cls, config_path))
        return EXIT_OK

    if args.preflight_only:
        return _run_preflight_only(cfg)

    return _run_mission(cfg)


# ============================================================
# Mission execution (S4)
# ============================================================
def _make_shared_pyhulax_api(cfg: FinalsConfig):
    """ONE pyhulax DroneAPI per drone (S10): the flight adapter and the video
    source MUST speak to the same link, so main creates the api here and injects
    it into BOTH builders. Method-local SDK import keeps main SDK-free (the
    composition-root convention); tests monkeypatch this seam to hand back a
    FakeDroneAPI(video_stream=FakeVideoStream()) and never import pyhulax."""
    from finals.flight.pyhulax_adapter import _real_drone_api_factory
    return _real_drone_api_factory()


def _build_adapter(cfg: FinalsConfig, drone: DroneConfig, *, api=None):
    """One adapter per drone, from (FinalsConfig, DroneConfig) — NEVER bare
    flight_cls(drone_id): BenchAdapter wraps an inner backend (special case
    below, see its docstring), and MavsdkSitlAdapter will need per-drone
    (sitl_address, mavsdk_grpc_port) from here in S6/SIM-1
    (sim_sessions.md "Notes to roadmap sessions").

    api (S10): the SHARED pyhulax DroneAPI for this drone, created once in
    _run_mission so the PyhulaxAdapter and the PyhulaxVideoSource speak to ONE
    link (None on every non-pyhulax path; the adapter then makes its own at
    connect())."""
    if cfg.flight_backend == "bench":
        from finals.flight.adapter import BenchAdapter
        from finals.flight.pyhulax_adapter import PyhulaxAdapter   # S9
        return BenchAdapter(PyhulaxAdapter(drone.id, api=api))
    if cfg.flight_backend == "mavsdk_sitl":
        # S6/SIM-1: per-drone (sitl_address, grpc_port) — instance i listens
        # on udpin 14540+i with its own mavsdk_server on gRPC 50051+i;
        # single-drone configs fall back to the top-level sitl_address+50051.
        from finals.config import resolve_sitl_endpoint
        from finals.flight.sitl_adapter import MavsdkSitlAdapter
        address, grpc_port = resolve_sitl_endpoint(cfg, drone)
        return MavsdkSitlAdapter(drone.id, sitl_address=address,
                                 grpc_port=grpc_port)
    if cfg.flight_backend == "pyhulax":
        # S10 real profile: ip is None here — preflight P3 resolves plane_id ->
        # ip and applies it via set_target_ip BEFORE P4 connect.
        from finals.flight.pyhulax_adapter import PyhulaxAdapter
        return PyhulaxAdapter(drone.id, api=api)
    flight_cls = resolve_flight_adapter_cls(cfg.flight_backend)
    if flight_cls is None:      # 'none' is guarded before we get here
        raise ConfigError(
            f"profile {cfg.profile!r} has flight_backend 'none' — nothing "
            f"to fly; this path should have been routed to the no-drone "
            f"branch (wiring bug)")
    return flight_cls(drone.id)


def _build_phases(drone_cfg: DroneConfig, cfg: FinalsConfig,
                  registry: "Optional[ConvoyRegistry]" = None,
                  obstacle_map: "Optional[ObstacleMap]" = None,
                  ) -> List[MissionPhase]:
    """Phase names -> instances. Soft convention: classes MAY define
    from_config(drone_cfg, cfg); otherwise no-arg construction. Stub phases
    raise their session pointer loudly here, at wiring time.

    WS-2: a phase that exposes bind_registry (track_convoy) gets the shared C2
    ConvoyRegistry injected here — the same post-construction injection style as
    perception<->agent wiring. Phases without it are untouched, so non-convoy
    missions never see a registry.

    WS-6: a from_config that ACCEPTS an `obstacle_map` parameter (navigate) gets
    the shared collective map passed in (signature-checked, so other phases'
    from_config are untouched). None map -> static-arena-only behaviour."""
    import inspect
    phases: List[MissionPhase] = []
    for name in drone_cfg.phases:
        phase_cls = resolve_phase(name)
        factory = getattr(phase_cls, "from_config", None)
        if factory is not None:
            kw = {}
            if obstacle_map is not None:
                try:
                    accepts = "obstacle_map" in inspect.signature(factory).parameters
                except (TypeError, ValueError):
                    accepts = False
                if accepts:
                    kw["obstacle_map"] = obstacle_map
            phase = factory(drone_cfg, cfg, **kw)
        else:
            phase = phase_cls()
        if registry is not None and hasattr(phase, "bind_registry"):
            phase.bind_registry(registry)
        phases.append(phase)
    return phases


def _uses_track_convoy(cfg: FinalsConfig) -> bool:
    """True iff any drone runs the track_convoy phase — the ONLY consumer of the
    ConvoyRegistry. Gates registry construction so non-convoy missions keep a
    clean heartbeat (no empty convoys block)."""
    return any("track_convoy" in d.phases for d in cfg.drones)


def _build_convoy_registry(cfg: FinalsConfig) -> "Optional[ConvoyRegistry]":
    """The shared C2 convoy-ownership authority, built once per mission — but
    only when a drone actually tracks convoys. lock_ttl_s and the known id set
    (5-of-5 denominator) come from config; both were validated at load."""
    if not _uses_track_convoy(cfg):
        return None
    from finals.mission.convoy_registry import ConvoyRegistry
    return ConvoyRegistry(lock_ttl_s=cfg.convoy_lock_ttl_s,
                          known_ids=cfg.convoy_ids)


def _uses_navigate(cfg: FinalsConfig) -> bool:
    """True iff any drone runs navigate — the ONLY consumer of the shared
    ObstacleMap (it merges the map into its transit plan)."""
    return any("navigate" in d.phases for d in cfg.drones)


def _build_obstacle_map(cfg: FinalsConfig) -> "Optional[ObstacleMap]":
    """The shared collective map of FIXED obstacles (WS-6 extension), built once
    per mission and threaded into EVERY navigating drone so a keep-out one drone
    (or the operator pre-flight tap) contributed routes ALL of them. Built only
    when a drone navigates; pre-seeded from cfg.observed_keep_out (validated at
    load) with provenance 'operator'. Returns None when nothing navigates OR no
    observations exist (so the static-arena path is byte-for-byte unchanged)."""
    if not _uses_navigate(cfg) or not cfg.observed_keep_out:
        return None
    from finals.mission.obstacle_map import ObstacleMap
    from finals.mission.planning.types import KeepOut
    omap = ObstacleMap()
    for i, raw in enumerate(cfg.observed_keep_out):
        ko = KeepOut.from_dict(raw, index=f"observed_keep_out[{i}]")
        omap.add_keep_out("operator", ko, now=0.0)     # pre-flight survey ts
    return omap


#: The orchestrator's supervision beat — shared with LoopOverrunGuard so the
#: overrun limit is judged against the period the loop actually runs at.
_HEARTBEAT_PERIOD_S = 1.0

#: Frame backends with END-TO-END perception wiring (video source +
#: PerceptionLoop + agent frame-ts). S8 (SIM-4) added "gazebo"; S9/S10 adds
#: "pyhulax". DELIBERATELY narrower than `frame_backend != "none"` — see the
#: VISION GATE wiring note in the module docstring (a backend declared in a
#: config but not yet wired would crash on its stub or false-DEGRADE).
#: One backend per line so parallel sessions add theirs as an isolated hunk.
_WIRED_FRAME_BACKENDS = (
    "replay",
    "gazebo",    # S8 (SIM-4): GazeboRgbSource <- sim/gz_camera_bridge (TCP)
    "pyhulax",   # S10: PyhulaxVideoSource over the shared DroneAPI (preflight
                 # connects the link before _amain starts the stream)
)


def _frames_wired(cfg: FinalsConfig) -> bool:
    """ONE source of truth for the vision gate: _build_guards (the
    VideoWatchdog) and _run_mission (perception) must never diverge — a
    watchdog without a frame source is a guaranteed-false DEGRADE."""
    return cfg.frame_backend in _WIRED_FRAME_BACKENDS


def _build_guards(cfg: FinalsConfig, drone: DroneConfig) -> List[Guard]:
    """Per-drone guard list from config. FRESH instances per drone — guards
    hold per-drone latch/counter state (see finals/guards.py)."""
    g = cfg.guards
    guards: List[Guard] = [
        TelemetryWatchdog(stale_s=g.telemetry_stale_s),
        BatteryGuard(floor_pct=cfg.min_battery_pct,
                     warn_pct=g.battery_warn_pct),
    ]
    if g.phase_timeout_s is not None:
        guards.append(PhaseTimeout(timeout_s=g.phase_timeout_s))
    if g.geofence_radius_m is not None:
        guards.append(GeofenceLite(radius_m=g.geofence_radius_m,
                                   alt_max_m=g.geofence_alt_m))
    if drone.sector_deg is not None:
        # NAV-8 advisory per-drone sector keep-in (SPACE half of the
        # deconfliction). Needs the C2 origin from the arena; a sector_deg
        # without an arena is a wiring error (ConfigError, fail loud — the
        # operator must set arena_name). ADVISORY ONLY (never a control input).
        if cfg.arena is None:
            raise ConfigError(
                f"drone {drone.id!r}: sector_deg is set but cfg.arena is None "
                f"— the advisory SectorGuard needs the C2 origin "
                f"(arena.c2_origin_m); set arena_name in the profile config")
        center_deg, half_width_deg = drone.sector_deg
        guards.append(SectorGuard(
            c2_origin_m=cfg.arena.c2_origin_m,
            sector_center_deg=center_deg,
            sector_half_width_deg=half_width_deg))
    if g.proximity_enable:
        # SENSE-IR: the HULA 4-directional IR obstacle guard (advisory->LAND
        # ladder). Built only when enabled; the reading reaches it via the
        # agent's proximity_fn (the synthetic feed in _build_agents — the LIVE
        # pyhulax IR read is an ONSITE gate, pyhulax exposes no IR getter today).
        guards.append(ProximityGuard(warn_cm=g.proximity_warn_cm,
                                     land_cm=g.proximity_land_cm))
    if _frames_wired(cfg):
        # S7: built ONLY when the perception wiring also feeds this drone's
        # agent a frame timestamp (frame_ts_fn -> GuardContext.last_frame_ts
        # in _run_mission). The stub-era note said `frame_backend != "none"`;
        # that gate is deliberately NARROWED to the wired set — sitl.json
        # declares "gazebo" before S8 wires it, and a watchdog with no frame
        # source logs a guaranteed-false "no frame EVER" DEGRADE every run
        # (guards.py reconciliation 5).
        guards.append(VideoWatchdog(stale_s=g.video_stale_s))
    return guards


def _build_swarm_guards(cfg: FinalsConfig) -> List[Guard]:
    """Mission-level guard list for the orchestrator tick."""
    g = cfg.guards
    swarm: List[Guard] = []
    if g.landing_reserve_s > 0:
        # reserve 0 = OFF (the default): a non-zero default would
        # instant-trip short --budget smoke runs (guards.py reconciliation 7).
        swarm.append(MissionClockGuard(budget_s=cfg.mission_budget_s,
                                       landing_reserve_s=g.landing_reserve_s))
    swarm.append(LoopOverrunGuard(period_s=_HEARTBEAT_PERIOD_S,
                                  factor=g.loop_overrun_factor,
                                  n_ticks=g.loop_overrun_ticks))
    return swarm


# ============================================================
# Vision wiring (S7)
# ============================================================
def _build_detector(cfg: FinalsConfig, bus: SightingBus,
                    slog: Optional[SightingLog], run_dir: str,
                    csv_health=None):
    """The OPTIONAL shared YOLO pool (ONE instance serves all drones — the
    detector.py contract) or None for backend "none". Lazy imports per the
    composition-root convention. csv_health: the RUN-WIDE
    CsvRecordingHealth so YOLO-path CSV death is observable too."""
    det = cfg.detector
    if det.backend == "none":
        return None
    from finals.vision.perception import make_detection_callback
    callback = make_detection_callback(
        bus, slog, class_map=det.class_map,
        camera_hfov_deg=cfg.camera_hfov_deg, csv_health=csv_health)
    if det.backend == "canned":
        from finals.vision.detector import CannedDetector
        return CannedDetector(det.canned_script, callback,
                              num_workers=det.workers)
    if det.backend == "ultralytics":
        from finals.vision.detector import make_ultralytics_detector
        return make_ultralytics_detector(
            det, callback,
            save_dir=os.path.join(run_dir, "detections"),
            enable_display=det.display)
    raise ConfigError(   # unreachable after load_config validation
        f"unknown detector.backend {det.backend!r} — wiring/validation drift")


def _build_perception(cfg: FinalsConfig, drone_id: str, bus: SightingBus,
                      slog: Optional[SightingLog], events: EventLog,
                      detector, csv_health=None, *, run_dir: str,
                      api=None) -> Tuple[object, object]:
    """One (VideoSource, PerceptionLoop) pair per drone. Called only when
    _frames_wired(cfg). The VideoSource is chosen by cfg.frame_backend:
    ReplaySource (replay), GazeboRgbSource over the sim TCP bridge (gazebo,
    SIM-4), or PyhulaxVideoSource over the per-drone shared DroneAPI (pyhulax,
    S10 — the SAME link the flight adapter uses; NOT started here, preflight P6
    owns start() AFTER P4 connects the api = the connect-before-stream-start
    ordering). The PerceptionLoop wiring below is identical for every backend."""
    from finals.vision.aruco import make_marker_detector
    from finals.vision.perception import PerceptionLoop
    if cfg.frame_backend == "gazebo":
        # S8 (SIM-4): frames arrive over a localhost TCP socket from
        # sim/gz_camera_bridge (a system-py3.10 gz subscriber) — finals/
        # imports NO gz binding (the 3.11-venv constraint). The bridge must
        # already be streaming before this source's start() (run-script gates
        # on the first frame); a missing bridge -> SensorTimeout -> loud abort.
        from finals.config import resolve_gazebo_video_port
        from finals.vision.gazebo_video import GazeboRgbSource
        source = GazeboRgbSource(
            drone_id,
            host=cfg.gazebo_video_host,
            port=resolve_gazebo_video_port(cfg, drone_id),  # SIM-5: per-drone bridge
            video_channel_order=cfg.video_channel_order,
            stale_s=cfg.guards.video_stale_s)
    elif cfg.frame_backend == "pyhulax":
        # S10 live path: the shared DroneAPI (api) is connected by preflight
        # P4 BEFORE preflight P6 calls source.start() — connect-before-stream.
        from finals.vision.pyhulax_video import PyhulaxVideoSource
        if api is None:      # wiring drift: _build_agents must pass the shared api
            raise ConfigError(
                f"{drone_id}: frame_backend 'pyhulax' needs the shared "
                f"DroneAPI — _build_agents must pass api= (wiring bug)")
        source = PyhulaxVideoSource(
            drone_id, api,
            video_channel_order=cfg.video_channel_order,
            stale_s=cfg.guards.video_stale_s)
    else:
        from finals.vision.video import ReplaySource
        source = ReplaySource(
            drone_id, cfg.replay_dir, fps=cfg.replay_fps,
            # The replay PROFILE ends when the frames do; a flight profile
            # replaying frames (dev rig) loops the clip for the whole mission.
            loop=(cfg.profile != "replay"))
    # S11: per-drone annotated-frame dir, wired only when save_marker_frames
    # is on (default off -> save_dir None -> minimal Sightings, suite unchanged).
    marker_save_dir = (os.path.join(run_dir, "marker_frames", drone_id)
                       if cfg.save_marker_frames else None)
    perception = PerceptionLoop(
        drone_id, source, bus, events,
        detect_marker=make_marker_detector(
            cfg.marker_backend,
            marker_dict=cfg.marker_dict,                 # PAD-DICT: real field 7x7
            aruco_detector_params=cfg.aruco_detector_params,
            save_dir=marker_save_dir),
        slog=slog, detector=detector,
        camera_hfov_deg=cfg.camera_hfov_deg,
        csv_health=csv_health,
        sample_hz=cfg.tick_hz,
        # A legal sub-1 Hz tick_hz must not die at PerceptionLoop's
        # degraded_hz <= sample_hz gate — shedding just becomes a no-op.
        degraded_hz=min(1.0, float(cfg.tick_hz)))
    return source, perception


def _perception_screamer(events: EventLog):
    """Done-callback for perception tasks: a crashed perception task must
    NEVER be a silent zero-sighting mission (install_crash_hooks does not
    see swallowed task exceptions). Flight is unaffected by design — the
    drone's VideoWatchdog DEGRADEs on the now-stale frame ts."""

    def _scream(task: "asyncio.Task") -> None:
        if task.cancelled():
            return
        exc = task.exception()       # also marks the exception as retrieved
        if exc is None:
            return
        tb = "".join(traceback.format_exception(type(exc), exc,
                                                exc.__traceback__))
        print(f"[main] ERROR: {task.get_name()} CRASHED — detection is DEAD "
              f"for that drone (flight continues; its VideoWatchdog will "
              f"DEGRADE on the stale frame ts):\n{tb}",
              file=sys.stderr, flush=True)
        try:
            events.log("mission", "perception_crashed",
                       task=task.get_name(), error=str(exc),
                       error_type=type(exc).__name__)
        except EventLogError as e:
            print(f"[main] WARNING: could not log perception_crashed: {e}",
                  file=sys.stderr, flush=True)

    return _scream


async def _amain(cfg: FinalsConfig, agents: List[DroneAgent],
                 events: EventLog, run_dir: str, bus: SightingBus,
                 perceptions: Sequence[Tuple[object, object]] = (),
                 registry: "Optional[ConvoyRegistry]" = None) -> int:
    """Preflight gate -> perception tasks (S7, when frames are wired) ->
    Orchestrator.run (with the S5 abort listener armed around it), on ONE
    event loop. Perception teardown always runs (finally), in dependency
    order: stop sampling -> stop sources (the detector pool is stopped by
    _run_mission, which owns it).

    SENSE-IR: the OPTIONAL per-drone DepthSource (depth_backend != "none") is
    lifecycle-managed here alongside the RGB sources — started before the
    mission, stopped in finally — but is NEVER a hard dependency: with
    depth_backend "none" (the default) `depth_sources` is empty and this whole
    path is a no-op (the monocular mission is byte-for-byte unchanged). Depth is
    not yet a perception CONSUMER (the swarm path is monocular); the seam exists
    so a future drift-correct/obstacle consumer plugs in without re-touching the
    lifecycle. A depth source that fails to start raises SensorTimeout, like the
    RGB source — loud, before anything arms."""
    sources = [s for s, _p in perceptions]
    # Optional depth sources — one per drone with perception wired (depth is a
    # per-drone sensor, parallel to the RGB source). Empty unless depth_backend
    # is set; absence = clean no-op.
    depth_sources = []
    if cfg.depth_backend != "none" and perceptions:
        for source, _perception in perceptions:
            ds = _build_depth(cfg, source.source_id)
            if ds is not None:
                depth_sources.append(ds)
    # bench/real: preflight (S10) owns connect (P4) AND video start (P6) — it
    # leaves adapters CONNECTED and sources STARTED for the orchestrator, so the
    # generic source.start loop below MUST skip them (PyhulaxVideoSource.start
    # refuses a second call). Other profiles have no hardware gate and start
    # their own sources here.
    preflight_owns_sources = cfg.profile in ("bench", "real")
    if preflight_owns_sources:
        from finals.preflight import run_preflight
        await run_preflight(cfg.profile, agents, cfg, sources=sources,
                            events=events, run_dir=run_dir)   # PreflightError -> 3
    else:
        # "mission" = the orchestrator's mission-level pseudo drone id.
        events.log("mission", "preflight", status="skipped",
                   reason=f"profile {cfg.profile!r} has no preflight gate "
                          f"(P0-P10 gates bench/real hardware; mock/sitl are "
                          f"pure software)")
    abort_event = threading.Event()
    orchestrator = Orchestrator(agents, events, run_dir,
                                budget_s=cfg.mission_budget_s, bus=bus,
                                heartbeat_period_s=_HEARTBEAT_PERIOD_S,
                                swarm_guards=_build_swarm_guards(cfg),
                                abort_event=abort_event,
                                convoy_registry=registry)
    listener = AbortListener(abort_event,
                             on_abort=orchestrator.request_stop_threadsafe)
    p_stop = asyncio.Event()
    p_tasks: List[asyncio.Task] = []
    listener.start()
    try:
        if perceptions:
            screamer = _perception_screamer(events)
            # The mission's hard ceiling (budget + settle grace) bounds the
            # perception tasks too; p_stop in the finally is the normal end.
            p_deadline = time.monotonic() + cfg.mission_budget_s + 60.0
            if not preflight_owns_sources:
                for source, perception in perceptions:
                    source.start(timeout_s=10.0)   # SensorTimeout -> loud abort
                                                   # BEFORE anything arms
                for ds in depth_sources:           # SENSE-IR optional depth
                    ds.start(timeout_s=10.0)       # SensorTimeout -> loud abort
                    events.log("mission", "depth_source_started",
                               drone=ds.source_id, backend=cfg.depth_backend)
            for source, perception in perceptions:
                task = asyncio.get_running_loop().create_task(
                    perception.run(deadline=p_deadline, stop_event=p_stop),
                    name=f"perception:{source.source_id}")
                task.add_done_callback(screamer)
                p_tasks.append(task)
        return await orchestrator.run()
    finally:
        listener.stop()
        p_stop.set()
        if p_tasks:
            await asyncio.wait(p_tasks, timeout=10.0)
            for task in p_tasks:
                if not task.done():
                    print(f"[main] WARNING: {task.get_name()} ignored the "
                          f"stop event for 10 s — cancelling",
                          file=sys.stderr, flush=True)
                    task.cancel()
        for source, _perception in perceptions:
            source.stop()                      # idempotent, never raises
        for ds in depth_sources:
            ds.stop()                          # SENSE-IR: idempotent, never raises


def resolve_depth_source_cls(backend: str) -> Optional[Type]:
    """depth_backend name -> DepthSource class (None for 'none'). ConfigError on
    unknown. The OPTIONAL SENSE-IR seam: 'none' (the default monocular swarm
    path) wires NO depth at all; 'fake' wires the dependency-free
    FakeDepthSource. A real 'realsense' backend is out of scope (reference only
    — see finals/vision/depth.py)."""
    if backend == "none":
        return None
    if backend == "fake":
        from finals.vision.depth import FakeDepthSource
        return FakeDepthSource
    raise ConfigError(
        f"unknown depth_backend {backend!r} — one of: none, fake "
        f"(realsense is reference-only, not wired — the swarm path is "
        f"monocular; see finals/vision/depth.py)")


def _build_depth(cfg: FinalsConfig, drone_id: str):
    """The OPTIONAL per-drone DepthSource (SENSE-IR), or None when
    depth_backend is 'none' (the degrade-absent default) — so the monocular
    mission is byte-for-byte unchanged. Built behind the perception wiring; the
    source is lifecycle-managed (start/stop) alongside the RGB sources but is
    NEVER a hard dependency — perception works identically with it None. The
    real RealSense backend is out of scope (reference only)."""
    depth_cls = resolve_depth_source_cls(cfg.depth_backend)
    if depth_cls is None:
        return None
    # Only "fake" reaches here today (resolve_depth_source_cls guards the rest).
    return depth_cls(drone_id)


def _build_proximity_fn(cfg: FinalsConfig, drone: DroneConfig, *, api=None):
    """The per-drone IR proximity_fn for the agent (SENSE-IR), or None when
    the ProximityGuard is OFF (proximity_enable False) — so a non-IR mission is
    byte-for-byte unchanged.

    LIVE-WIRE = ONSITE GATE: pyhulax exposes no IR getter today (see
    finals/flight/proximity.py / pyhulax_adapter.py), so the live
    PyhulaxProximitySensor stays a stub. We wire SyntheticProximitySensor with
    its DEFAULT (reading=None): the guard gets an honest 'no live IR reading'
    every tick and SKIPS — never a fabricated clear lane. At the hardware
    window, swap to PyhulaxProximitySensor(drone.id, api) and the guard goes
    live with NO other change. Returns the sensor's bound read method (the
    agent's proximity_fn injectable)."""
    if not cfg.guards.proximity_enable:
        return None
    from finals.flight.proximity import SyntheticProximitySensor
    sensor = SyntheticProximitySensor(drone.id)   # reading=None -> guard SKIPS
    return sensor.read


def _build_agents(cfg: FinalsConfig, events: EventLog, bus: SightingBus,
                  slog: Optional[SightingLog], detector, csv_health, safety,
                  run_dir: str,
                  registry: "Optional[ConvoyRegistry]" = None,
                  obstacle_map: "Optional[ObstacleMap]" = None,
                  ) -> Tuple[List[DroneAgent], List[Tuple[object, object]]]:
    """One DroneAgent + its (source, perception) pair per cfg.drone. Shared by
    the mission path (_run_mission) and --preflight-only (_run_preflight_only)
    so they build the IDENTICAL fleet — the bench tool must exercise exactly
    what flies. For the live pyhulax path one shared DroneAPI per drone feeds
    BOTH the adapter and the video source (the same-link invariant).

    Returns agents in cfg.drones order (preflight P3 zips on that order)."""
    agents: List[DroneAgent] = []
    perceptions: List[Tuple[object, object]] = []
    for d in cfg.drones:
        frame_ts_fn = None
        on_degrade = None
        perception = None
        # S10: ONE shared pyhulax DroneAPI per drone (flight + video, same
        # link). None on every other backend — each builder makes its own.
        api = (_make_shared_pyhulax_api(cfg)
               if cfg.frame_backend == "pyhulax" else None)
        if _frames_wired(cfg):
            source, perception = _build_perception(
                cfg, d.id, bus, slog, events, detector,
                csv_health=csv_health, run_dir=run_dir, api=api)
            perceptions.append((source, perception))
            frame_ts_fn = perception.last_frame_ts
            on_degrade = (lambda trip, p=perception:
                          p.shed(trip.reason))
        agent = DroneAgent(d.id, _build_adapter(cfg, d, api=api),
                           _build_phases(d, cfg, registry, obstacle_map),
                           events, bus=bus,
                           command_timeout_s=cfg.command_timeout_s,
                           guards=_build_guards(cfg, d),
                           safety=safety,
                           frame_ts_fn=frame_ts_fn,
                           on_degrade=on_degrade,
                           proximity_fn=_build_proximity_fn(cfg, d, api=api))
        if perception is not None:
            # Wire-once AFTER the agent exists (the perception<->agent
            # reference cycle): enrichment reads the agent's cached per-tick
            # telemetry, never the adapter directly.
            perception.set_telemetry_source(
                lambda a=agent: a.last_telemetry)
        agents.append(agent)
    return agents, perceptions


def _build_fleet_support(cfg: FinalsConfig, events: EventLog, bus: SightingBus,
                         run_dir: str):
    """The optional vision support trio (SightingLog, shared YOLO detector,
    run-wide CsvRecordingHealth) — built ONLY when frames are wired, None
    otherwise. The caller owns teardown (detector.stop / slog.close in its
    finally). Shared by the mission + preflight-only paths."""
    if not _frames_wired(cfg):
        return None, None, None
    from finals.vision.perception import CsvRecordingHealth
    slog = SightingLog(os.path.join(run_dir, "sightings.csv"))
    csv_health = CsvRecordingHealth()              # run-wide, all paths
    detector = _build_detector(cfg, bus, slog, run_dir, csv_health=csv_health)
    return slog, detector, csv_health


def _build_safety(cfg: FinalsConfig, events: EventLog) -> SafetyController:
    return SafetyController(
        events,
        land_retry_period_s=cfg.guards.land_retry_period_s,
        land_retry_window_s=cfg.guards.land_retry_window_s,
        command_timeout_s=cfg.command_timeout_s,
        slot_wait_s=cfg.guards.slot_wait_s,
        launch_slot_wait_s=cfg.guards.launch_slot_wait_s)


def _run_mission(cfg: FinalsConfig) -> int:
    if cfg.flight_backend == "none":
        return _run_replay(cfg)

    run_dir = create_run_dir(cfg.run_dir)
    install_crash_hooks(run_dir)
    with EventLog(run_dir) as events:
        bus = SightingBus()
        safety = _build_safety(cfg, events)
        registry = _build_convoy_registry(cfg)
        obstacle_map = _build_obstacle_map(cfg)
        slog, detector, csv_health = _build_fleet_support(
            cfg, events, bus, run_dir)
        try:
            agents, perceptions = _build_agents(
                cfg, events, bus, slog, detector, csv_health, safety, run_dir,
                registry=registry, obstacle_map=obstacle_map)
            return asyncio.run(_amain(cfg, agents, events, run_dir, bus,
                                      perceptions=perceptions,
                                      registry=registry))
        finally:
            if detector is not None:
                detector.stop()                # joins the worker threads
            if slog is not None:
                slog.close()


def _run_preflight_only(cfg: FinalsConfig) -> int:
    """`--preflight-only`: build the EXACT mission fleet and run the P0-P9 gate
    (P10 operator-GO is skipped — preflight-only never flies), then tear the
    link down. The primary onsite bench tool. Requires bench/real (preflight is
    their gate); any other profile is a ConfigError. Exit 0 = all critical
    gates passed (WARNs allowed); a critical failure raises PreflightError ->
    main() maps it to exit 3."""
    if cfg.profile not in ("bench", "real"):
        raise ConfigError(
            f"--preflight-only needs --profile bench or real (preflight is "
            f"their P0-P10 hardware gate); profile {cfg.profile!r} has none — "
            f"mock/sitl are pure software")
    run_dir = create_run_dir(cfg.run_dir)
    install_crash_hooks(run_dir)
    with EventLog(run_dir) as events:
        from finals.preflight import run_preflight
        bus = SightingBus()
        safety = _build_safety(cfg, events)
        registry = _build_convoy_registry(cfg)
        obstacle_map = _build_obstacle_map(cfg)
        slog, detector, csv_health = _build_fleet_support(
            cfg, events, bus, run_dir)
        try:
            agents, perceptions = _build_agents(
                cfg, events, bus, slog, detector, csv_health, safety, run_dir,
                registry=registry, obstacle_map=obstacle_map)
            sources = [s for s, _p in perceptions]
            results = asyncio.run(run_preflight(
                cfg.profile, agents, cfg, sources=sources, events=events,
                run_dir=run_dir, preflight_only=True))   # PreflightError -> 3
            # We only reach here when no critical gate failed (a critical fail
            # raises). WARNs (non-critical, ok=False) do not fail the bench.
            ok = all(r.ok or not r.critical for r in results)
            return EXIT_OK if ok else EXIT_PREFLIGHT
        finally:
            if detector is not None:
                detector.stop()
            if slog is not None:
                slog.close()


# ============================================================
# The no-drone replay runner (S7)
# ============================================================
def _run_replay(cfg: FinalsConfig) -> int:
    """profile=replay: frames from disk -> marker detection (+ optional
    YOLO) -> sightings.csv (the score-relevant artifact) + mission.jsonl.
    0 drones, no flight — the Orchestrator (which refuses an empty agent
    list) is replaced by the small bounded beat in _areplay. Exit 0 on a
    clean run (frames exhausted or budget reached); 1 when perception
    crashed or the source died mid-stream."""
    run_dir = create_run_dir(cfg.run_dir)
    install_crash_hooks(run_dir)
    t0 = time.monotonic()
    with EventLog(run_dir) as events:
        from finals.vision.perception import CsvRecordingHealth
        bus = SightingBus()
        slog = SightingLog(os.path.join(run_dir, "sightings.csv"))
        csv_health = CsvRecordingHealth()      # run-wide: marker + YOLO paths
        detector = None
        try:
            detector = _build_detector(cfg, bus, slog, run_dir,
                                       csv_health=csv_health)
            source, perception = _build_perception(
                cfg, "replay", bus, slog, events, detector,
                csv_health=csv_health, run_dir=run_dir)
            events.log("mission", "run_start", profile=cfg.profile,
                       replay_dir=cfg.replay_dir, replay_fps=cfg.replay_fps,
                       marker_backend=cfg.marker_backend,
                       detector=cfg.detector.backend,
                       budget_s=cfg.mission_budget_s, run_dir=run_dir)
            source.start(timeout_s=10.0)       # SensorTimeout -> loud abort
            try:
                # _areplay stops the detector BEFORE its final drain, so the
                # snapshot/summary below see every row a worker produced.
                exit_code = asyncio.run(
                    _areplay(cfg, events, bus, source, perception,
                             detector=detector, csv_health=csv_health))
            finally:
                source.stop()
            rows = slog.snapshot()
            events.log("mission", "run_end", exit_code=exit_code,
                       sightings=len(rows),
                       csv_dead=csv_health.dead,
                       frames_delivered=source.delivered_count,
                       frames_sampled=perception.stats()["frames_sampled"])
            _print_replay_summary(cfg, run_dir, rows, source, perception,
                                  csv_health,
                                  elapsed_s=time.monotonic() - t0)
            return exit_code
        finally:
            if detector is not None:
                detector.stop()                # idempotent (also in _areplay)
            slog.close()


async def _areplay(cfg: FinalsConfig, events: EventLog, bus: SightingBus,
                   source, perception, detector=None, csv_health=None) -> int:
    """The replay supervision beat: run perception as a task, mirror every
    bus sighting into the event log (the orchestrator's _drain_bus shape),
    end on frames-exhausted / source-error / budget / task-death — all
    bounded (convention 3). A DEAD sighting CSV is exit 1: the CSV is the
    score artifact, and 'exit 0 + silently truncated CSV' is exactly the
    failure shape this runner exists to prevent."""
    stop = asyncio.Event()
    deadline = time.monotonic() + cfg.mission_budget_s
    task = asyncio.get_running_loop().create_task(
        perception.run(deadline=deadline, stop_event=stop),
        name="perception:replay")
    task.add_done_callback(_perception_screamer(events))
    cursor = 0
    beat_s = min(0.25, perception.current_period_s)

    def _drain() -> None:
        nonlocal cursor
        cursor, items = bus.drain_after(cursor)
        for s in items:
            events.log(s.drone_id, "sighting", source=s.source,
                       class_name=s.class_name, marker_id=s.marker_id,
                       confidence=s.confidence, ts=s.ts,
                       frame_number=s.frame_number,
                       bearing_deg=s.bearing_deg)

    exit_code = EXIT_OK
    try:
        # Bounded (convention 3): budget deadline + exhaustion + source
        # error + perception-task death, re-checked every beat.
        while True:
            _drain()
            if task.done():
                break                          # screamer already reported it
            if source.errored:
                exit_code = EXIT_ERROR         # source screamed on stderr
                events.log("mission", "replay_source_died",
                           delivered=source.delivered_count)
                break
            if source.exhausted:
                # Let perception drain the final frame before stopping.
                await asyncio.sleep(3 * perception.current_period_s)
                events.log("mission", "replay_exhausted",
                           delivered=source.delivered_count)
                break
            if time.monotonic() >= deadline:
                events.log("mission", "budget_expired",
                           budget_s=cfg.mission_budget_s,
                           note="replay budget reached before exhaustion")
                break
            await asyncio.sleep(beat_s)
    finally:
        stop.set()
        await asyncio.wait({task}, timeout=10.0)
        if not task.done():
            print("[main] WARNING: perception:replay ignored the stop "
                  "event for 10 s — cancelling", file=sys.stderr, flush=True)
            task.cancel()
            exit_code = EXIT_ERROR
        elif not task.cancelled() and task.exception() is not None:
            exit_code = EXIT_ERROR             # screamer printed the traceback
        if detector is not None:
            # BEFORE the final drain: joins the workers (pending frames are
            # abandoned by contract), so every published sighting is on the
            # bus by the time we drain — the event mirror and the CSV agree.
            # Synchronous on the loop; the run is over, nothing to starve.
            detector.stop()
        _drain()                               # nothing left behind

    if csv_health is not None and csv_health.dead:
        events.log("mission", "csv_recording_dead",
                   failures=csv_health.failures_total,
                   recover="mission.jsonl 'sighting' events mirror the lost rows")
        print("[main] ERROR: the sighting CSV died mid-run "
              f"({csv_health.failures_total} append failure(s)) — exit 1; "
              f"RECOVER score rows from this run's mission.jsonl 'sighting' "
              f"events", file=sys.stderr, flush=True)
        exit_code = EXIT_ERROR

    return exit_code


def _print_replay_summary(cfg: FinalsConfig, run_dir: str, rows: list,
                          source, perception, csv_health,
                          elapsed_s: float) -> None:
    by_class: dict = {}
    for s in rows:
        by_class[s.class_name] = by_class.get(s.class_name, 0) + 1
    stats = perception.stats()
    lines = [
        "=" * 72,
        f"REPLAY SUMMARY  elapsed={elapsed_s:.1f}s  "
        f"frames: {source.delivered_count} delivered / "
        f"{stats['frames_sampled']} sampled  sightings={len(rows)}"
        + ("  [DEGRADED]" if stats["degraded"] else "")
        + ("  [CSV-DEAD — rows LOST; recover from mission.jsonl]"
           if csv_health.dead else ""),
        "=" * 72,
    ]
    for name in sorted(by_class):
        lines.append(f"  {name:<24} x{by_class[name]}")
    if not rows:
        lines.append("  (no sightings — check the frames / marker_backend)")
    lines.append(f"sightings.csv : {os.path.join(run_dir, 'sightings.csv')}")
    lines.append(f"run dir       : {run_dir}")
    lines.append("=" * 72)
    print("\n".join(lines), flush=True)


def main(argv: Optional[List[str]] = None) -> int:
    try:
        return run(argv)
    except ConfigError as e:
        print(f"\nCONFIG ERROR: {e}\n", file=sys.stderr)
        return EXIT_CONFIG
    except PreflightError as e:
        print(f"\nPREFLIGHT FAILED: {e}\n", file=sys.stderr)
        return EXIT_PREFLIGHT
    # Anything else (incl. NotImplementedError stub session pointers — the
    # finals/docs/module_map.md convention — and FinalsError subtypes not
    # handled above) propagates with a full traceback: fail loud.


if __name__ == "__main__":
    sys.exit(main())
