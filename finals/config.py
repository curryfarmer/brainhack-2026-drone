"""Config schema + loud JSON loader for the finals package.

Format is JSON to match the repo's existing convention (model_config.json,
qualifier_run.py MissionConfig.from_json) — no new deps, diff-able, editable
at 7 a.m. on competition day.

Derives from: qualifier_run.py:72-132 MissionConfig (dataclass + from_json),
hardened with the lessons of docs/quali known issue #9 (best.pt trained but the
config silently kept pointing at the COCO placeholder — a scored run almost
flew with the wrong weights). Hence the two weights guards below:
  1. weights file must EXIST on disk (error lists *.pt candidates it found);
  2. known COCO placeholder names are REJECTED unless "allow_coco_weights":
     true is set explicitly (the interim car-class proxy is an opt-in, never
     an accident).

Loader conventions (binding):
- Unknown keys -> ConfigError naming them and listing valid keys (catches typos).
- Missing required keys -> ConfigError with the exact key and an example value.
- Keys starting with "_" are ignored everywhere (JSON has no comments; use
  "_comment": "..." freely).

Session: S1 (implemented).
"""
from __future__ import annotations

import glob
import json
import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from finals.errors import ConfigError
from finals.mission.planning.types import ArenaMap

VALID_PROFILES = ("mock", "sitl", "replay", "bench", "real")
VALID_FRAME_BACKENDS = ("none", "gazebo", "pyhulax", "replay")
VALID_DETECTOR_BACKENDS = ("none", "ultralytics", "canned")
# The PRIMARY (marker) detector seam — S7. "aruco" is the default per the
# 2026-06-06 intel (convoy robots carry markers to detect + READ); "qr" is
# the alternate path for the still-open "QR 20x20 cm" confirmation. Both
# feed the SAME Sighting stream (finals/vision/aruco.py).
VALID_MARKER_BACKENDS = ("aruco", "qr")

# Each profile pins its flight backend — both appear in the JSON so a human
# reading the file sees the whole story, and the loader cross-checks them so a
# copy-paste between profiles fails loudly instead of arming the wrong backend.
PROFILE_FLIGHT_BACKEND = {
    "mock": "mock",
    "sitl": "mavsdk_sitl",
    "replay": "none",
    "bench": "bench",
    "real": "pyhulax",
}

# Stock ultralytics COCO checkpoints — selecting one of these is almost always
# the placeholder accident from the qualifiers, so it requires an explicit flag.
KNOWN_COCO_PLACEHOLDERS = (
    "yolov8n.pt", "yolov8s.pt", "yolov10n.pt", "yolov10s.pt",
    "yolo11n.pt", "yolo11s.pt", "yolov5n.pt", "yolov5s.pt",
)


# ============================================================
# Schema
# ============================================================
@dataclass
class DetectorConfig:
    backend: str = "none"                       # "ultralytics" | "canned" | "none"
    weights: str = ""                           # REQUIRED for ultralytics
    conf: float = 0.4
    device: str = "cpu"                         # "cuda" if the C2 laptop has it
    workers: int = 1
    class_map: Dict[str, str] = field(default_factory=dict)   # model name -> canonical name
    canned_script: Optional[str] = None         # REQUIRED for canned
    allow_coco_weights: bool = False            # explicit opt-in for the COCO car-proxy interim
    display: bool = False                       # detection window (debug only)


@dataclass
class GuardsConfig:
    """S5 guard thresholds (finals/guards.py). Everything here is the onsite
    "tune config, not code" surface; main.py builds the guard objects from
    these. BatteryGuard's FLOOR deliberately stays the existing top-level
    min_battery_pct (one battery floor, not two)."""

    telemetry_stale_s: float = 2.0          # TelemetryWatchdog (policy layer;
                                            # the agent's 5 s backstop stays)
    battery_warn_pct: float = 30.0          # BatteryGuard warn -> advisory event
    video_stale_s: float = 2.0              # VideoWatchdog (built only when
                                            # frame_backend != "none")
    landing_reserve_s: float = 0.0          # MissionClockGuard: land-all at
                                            # budget - reserve; 0 = guard OFF
    phase_timeout_s: Optional[float] = None # PhaseTimeout; None = guard OFF
    geofence_radius_m: Optional[float] = None   # GeofenceLite (advisory only);
    geofence_alt_m: Optional[float] = None      # None = guard OFF
    loop_overrun_factor: float = 2.0        # LoopOverrunGuard: latency limit =
    loop_overrun_ticks: int = 5             # factor x period, for n ticks
    land_retry_period_s: float = 1.0        # SafetyController ladder cadence
    land_retry_window_s: float = 30.0       # ladder total -> operator alarm
    slot_wait_s: float = 120.0              # max wait for the landing slot
    launch_slot_wait_s: float = 120.0       # max wait for the C2 launch
                                            # corridor slot (NAV-8 staggered
                                            # launch); bounded, never infinite


@dataclass
class DroneConfig:
    id: str                                     # "alpha" — used in logs, sightings, prefixes
    plane_id: Optional[int] = None              # Dola discovery key (REQUIRED for bench/real)
    phases: List[str] = field(default_factory=list)   # PHASE_REGISTRY names, run in order
    led_rgb: Optional[Tuple[int, int, int]] = None    # identity colour (bench/real)
    altitude_band_m: Optional[float] = None     # swarm vertical separation (1.2/1.7/2.2)
    zone: Dict[str, Any] = field(default_factory=dict)  # per-drone search params (briefing-shaped)
    # NAV-8 per-drone ADVISORY sector keep-in wedge (SPACE half of the
    # deconfliction): [center_deg, half_width_deg], a heading range from C2
    # (deg, CCW+, 0 = +north). ADVISORY ONLY (SectorGuard, never a control
    # input). Optional; omit -> no sector guard for this drone.
    sector_deg: Optional[Tuple[float, float]] = None
    # SITL multi-instance endpoints (S6/SIM-1): instance i listens on UDP
    # 14540+i and its mavsdk_server takes gRPC 50051+i. REQUIRED (and
    # distinct) on every drone when a sitl profile has >1 drone; a single
    # drone may omit both and falls back to the top-level sitl_address +
    # 50051 (resolve_sitl_endpoint below).
    sitl_address: Optional[str] = None          # e.g. "udpin://0.0.0.0:14541"
    mavsdk_grpc_port: Optional[int] = None      # e.g. 50052
    # Per-drone gz_camera_bridge TCP port (SIM-5): each camera-drone reads its
    # OWN onboard camera via its OWN bridge, so a multi-drone gazebo config
    # needs a DISTINCT port per drone. A single-drone gazebo config may omit it
    # and falls back to the top-level gazebo_video_port (resolve_gazebo_video_port).
    gazebo_video_port: Optional[int] = None     # e.g. 5601


@dataclass
class FinalsConfig:
    profile: str                                # "mock" | "sitl" | "replay" | "bench" | "real"
    flight_backend: str                         # pinned per profile, cross-checked
    frame_backend: str                          # "none" | "gazebo" | "pyhulax" | "replay"
    detector: DetectorConfig
    drones: List[DroneConfig]
    run_dir: str = "./runs_finals"
    tick_hz: float = 10.0
    mission_budget_s: float = 600.0
    command_timeout_s: float = 15.0
    discovery_timeout_s: float = 10.0           # preflight P3 Dola listen window (bench/real)
    min_battery_pct: float = 20.0
    video_channel_order: str = "rgb"            # what .to_rgb() ACTUALLY returns — bench-verified
    camera_hfov_deg: Optional[float] = None     # needed for Sighting.bearing_deg; bench-measured
    sitl_address: str = "udpin://0.0.0.0:14540"
    marker_backend: str = "aruco"               # "aruco" | "qr" — the primary detector seam (S7)
    replay_dir: Optional[str] = None            # REQUIRED whenever frame_backend=replay
    replay_fps: float = 10.0                    # ReplaySource pacing (frames/s from disk)
    gazebo_video_host: str = "127.0.0.1"        # GazeboRgbSource <- sim/gz_camera_bridge endpoint
    gazebo_video_port: int = 5600               # localhost TCP port the bridge serves frames on
    use_uwb: bool = False
    uwb_serial_port: Optional[str] = None
    # Challenge-2A landing navigation (S11/NAV-0): the optional arena_name names a
    # finals/configs/arenas/<name>.json map (obstacles + pads + C2 frame). It is
    # resolved into `arena` at load time; `arena` is DERIVED, never set in JSON.
    # NAV-2 hardens the arena validation + ships configs/arenas/sample.json.
    arena_name: Optional[str] = None
    arena: Optional[ArenaMap] = None
    guards: GuardsConfig = field(default_factory=GuardsConfig)


def resolve_sitl_endpoint(cfg: "FinalsConfig",
                          drone: "DroneConfig") -> Tuple[str, int]:
    """(sitl_address, mavsdk_grpc_port) for one drone: the per-drone fields
    when set, else the single-drone fallback (top-level sitl_address +
    gRPC 50051). Multi-drone sitl configs are validated to carry BOTH fields
    on every drone, so the fallback can only serve a single-drone config.
    Pure — unit-tested without mavsdk; main._build_adapter is the consumer."""
    address = drone.sitl_address if drone.sitl_address is not None \
        else cfg.sitl_address
    port = drone.mavsdk_grpc_port if drone.mavsdk_grpc_port is not None \
        else 50051
    return address, port


def resolve_gazebo_video_port(cfg: "FinalsConfig", drone_id: str) -> int:
    """The gz_camera_bridge TCP port for one drone (SIM-5): the drone's own
    gazebo_video_port when set, else the top-level cfg.gazebo_video_port
    fallback (single-drone gazebo). Multi-drone gazebo configs are validated to
    carry a DISTINCT gazebo_video_port on every drone, so the fallback can only
    serve a single-drone config. Pure — unit-tested; main._build_perception is
    the consumer (the gazebo branch). A drone_id not in cfg.drones (the no-drone
    replay runner never reaches the gazebo branch) yields the top-level port."""
    drone = next((d for d in cfg.drones if d.id == drone_id), None)
    if drone is not None and drone.gazebo_video_port is not None:
        return drone.gazebo_video_port
    return cfg.gazebo_video_port


# ============================================================
# Loud loader
# ============================================================
def _check_keys(raw: Dict[str, Any], required: Tuple[str, ...],
                optional: Tuple[str, ...], where: str) -> Dict[str, Any]:
    """Return raw minus comment keys; ConfigError on unknown or missing keys."""
    if not isinstance(raw, dict):
        raise ConfigError(f"{where}: expected a JSON object, got {type(raw).__name__}")
    data = {k: v for k, v in raw.items() if not k.startswith("_")}
    unknown = sorted(set(data) - set(required) - set(optional))
    if unknown:
        raise ConfigError(
            f"{where}: unknown key(s) {unknown} — valid keys: "
            f"{sorted(set(required) | set(optional))} (typo?)"
        )
    missing = sorted(set(required) - set(data))
    if missing:
        raise ConfigError(f"{where}: missing required key(s) {missing}")
    return data


def _validate_detector(det: DetectorConfig, config_dir: str) -> None:
    if det.backend not in VALID_DETECTOR_BACKENDS:
        raise ConfigError(
            f"detector.backend {det.backend!r} invalid — one of {VALID_DETECTOR_BACKENDS}"
        )
    if not 0.0 < det.conf <= 1.0:
        raise ConfigError(f"detector.conf {det.conf} out of range (0, 1]")
    if (not isinstance(det.workers, int) or isinstance(det.workers, bool)
            or det.workers < 1):
        raise ConfigError(
            f"detector.workers must be an int >= 1, got {det.workers!r} — "
            f"the worker pool needs at least one thread")
    if det.backend == "ultralytics":
        if not det.weights:
            raise ConfigError(
                'detector.weights is required for backend "ultralytics" '
                '(e.g. "best.pt") — never relies on a silent default'
            )
        # Resolve relative to the config file's dir, then the CWD (repo root).
        candidates = [det.weights, os.path.join(config_dir, det.weights)]
        resolved = next((p for p in candidates if os.path.isfile(p)), None)
        if resolved is None:
            found = sorted(glob.glob("*.pt")) or ["<none in CWD>"]
            raise ConfigError(
                f"detector.weights {det.weights!r} not found on disk "
                f"(tried {candidates}). .pt files visible from CWD: {found}"
            )
        det.weights = resolved
        base = os.path.basename(resolved).lower()
        if base in KNOWN_COCO_PLACEHOLDERS and not det.allow_coco_weights:
            raise ConfigError(
                f"detector.weights {base!r} is a stock COCO checkpoint — the "
                f"qualifier placeholder trap. If the COCO car-class proxy is "
                f'intended (interim, pre-retrain), set "allow_coco_weights": true.'
            )
    if det.backend == "canned":
        if not det.canned_script:
            raise ConfigError('detector.canned_script is required for backend "canned"')


def _build_drone(raw: Dict[str, Any], index: int) -> DroneConfig:
    where = f"drones[{index}]"
    data = _check_keys(
        raw,
        required=("id", "phases"),
        optional=("plane_id", "led_rgb", "altitude_band_m", "zone",
                  "sitl_address", "mavsdk_grpc_port", "gazebo_video_port",
                  "sector_deg"),
        where=where,
    )
    phases = data["phases"]
    if (not isinstance(phases, list) or not phases
            or not all(isinstance(p, str) and p for p in phases)):
        raise ConfigError(
            f"{where}.phases must be a non-empty list of phase names "
            f'(e.g. ["takeoff_demo"]) — got {phases!r}'
        )
    led = data.get("led_rgb")
    if led is not None:
        if (not isinstance(led, list) or len(led) != 3
                or not all(isinstance(c, int) and 0 <= c <= 255 for c in led)):
            raise ConfigError(f"{where}.led_rgb must be [r, g, b] ints 0-255 — got {led!r}")
        led = tuple(led)
    sitl_address = data.get("sitl_address")
    if sitl_address is not None and (
            not isinstance(sitl_address, str) or not sitl_address):
        raise ConfigError(
            f"{where}.sitl_address must be a non-empty string like "
            f'"udpin://0.0.0.0:14541" — got {sitl_address!r}')
    grpc_port = data.get("mavsdk_grpc_port")
    if grpc_port is not None and (
            not isinstance(grpc_port, int) or isinstance(grpc_port, bool)
            or not 1024 <= grpc_port <= 65535):
        raise ConfigError(
            f"{where}.mavsdk_grpc_port must be an int in [1024, 65535] "
            f"(instance i uses 50051+i) — got {grpc_port!r}")
    gz_port = data.get("gazebo_video_port")
    if gz_port is not None and (
            not isinstance(gz_port, int) or isinstance(gz_port, bool)
            or not 1024 <= gz_port <= 65535):
        raise ConfigError(
            f"{where}.gazebo_video_port must be an int in [1024, 65535] "
            f"(the per-drone gz_camera_bridge TCP port) — got {gz_port!r}")
    sector = data.get("sector_deg")
    if sector is not None:
        if (not isinstance(sector, (list, tuple)) or len(sector) != 2
                or any(not isinstance(c, (int, float)) or isinstance(c, bool)
                       or not math.isfinite(c) for c in sector)):
            raise ConfigError(
                f"{where}.sector_deg must be [center_deg, half_width_deg] "
                f"finite numbers (the ADVISORY keep-in wedge from C2, deg, "
                f"CCW+) — got {sector!r}")
        if sector[1] < 0:
            raise ConfigError(
                f"{where}.sector_deg half_width_deg must be >= 0 (a negative "
                f"wedge half-angle would strand the drone outside every "
                f"sector) — got {sector!r}")
        sector = (float(sector[0]), float(sector[1]))
    return DroneConfig(
        id=str(data["id"]),
        plane_id=data.get("plane_id"),
        phases=list(phases),
        led_rgb=led,
        altitude_band_m=data.get("altitude_band_m"),
        zone=dict(data.get("zone", {})),
        sitl_address=sitl_address,
        mavsdk_grpc_port=grpc_port,
        gazebo_video_port=gz_port,
        sector_deg=sector,
    )


def load_config(path: str, overrides: Optional[Dict[str, Any]] = None) -> FinalsConfig:
    """Load + validate a profile JSON. Raises ConfigError with an actionable
    message on ANY problem — this function never half-loads.

    overrides: CLI-level tweaks applied before validation. Recognized keys:
      weights (str), budget_s (float), phases (list[str], replaces every
      drone's list), no_detector (bool), display (bool).
    """
    if not os.path.isfile(path):
        raise ConfigError(f"config file not found: {path!r} (CWD: {os.getcwd()})")
    with open(path, "r", encoding="utf-8") as f:
        try:
            raw = json.load(f)
        except json.JSONDecodeError as e:
            raise ConfigError(f"{path}: invalid JSON — {e}") from e

    top = _check_keys(
        raw,
        required=("profile", "flight_backend", "frame_backend", "detector", "drones"),
        optional=(
            "run_dir", "tick_hz", "mission_budget_s", "command_timeout_s",
            "discovery_timeout_s",
            "min_battery_pct", "video_channel_order", "camera_hfov_deg",
            "sitl_address", "marker_backend", "replay_dir", "replay_fps",
            "gazebo_video_host", "gazebo_video_port",
            "use_uwb", "uwb_serial_port", "arena_name", "guards",
        ),
        where=path,
    )

    det_data = _check_keys(
        top["detector"],
        required=("backend",),
        optional=("weights", "conf", "device", "workers", "class_map",
                  "canned_script", "allow_coco_weights", "display"),
        where=f"{path}: detector",
    )
    detector = DetectorConfig(**det_data)

    guards_data = _check_keys(
        top.get("guards", {}),
        required=(),
        optional=("telemetry_stale_s", "battery_warn_pct", "video_stale_s",
                  "landing_reserve_s", "phase_timeout_s", "geofence_radius_m",
                  "geofence_alt_m", "loop_overrun_factor", "loop_overrun_ticks",
                  "land_retry_period_s", "land_retry_window_s", "slot_wait_s",
                  "launch_slot_wait_s"),
        where=f"{path}: guards",
    )
    guards = GuardsConfig(**guards_data)

    if not isinstance(top["drones"], list):
        raise ConfigError(f"{path}: drones must be a list (may be empty only for replay)")
    drones = [_build_drone(d, i) for i, d in enumerate(top["drones"])]

    cfg = FinalsConfig(
        profile=top["profile"],
        flight_backend=top["flight_backend"],
        frame_backend=top["frame_backend"],
        detector=detector,
        drones=drones,
        guards=guards,
        **{k: top[k] for k in (
            "run_dir", "tick_hz", "mission_budget_s", "command_timeout_s",
            "discovery_timeout_s",
            "min_battery_pct", "video_channel_order", "camera_hfov_deg",
            "sitl_address", "marker_backend", "replay_dir", "replay_fps",
            "gazebo_video_host", "gazebo_video_port",
            "use_uwb", "uwb_serial_port", "arena_name",
        ) if k in top},
    )

    # --- CLI overrides (before validation so they are validated too) ---
    ov = dict(overrides or {})
    if ov.pop("no_detector", False):
        cfg.detector.backend = "none"
    if "weights" in ov:
        cfg.detector.weights = ov.pop("weights")
        cfg.detector.backend = "ultralytics"
    if "budget_s" in ov:
        cfg.mission_budget_s = float(ov.pop("budget_s"))
    if "phases" in ov:
        forced = list(ov.pop("phases"))
        for d in cfg.drones:
            d.phases = list(forced)
    if ov.pop("display", False):
        cfg.detector.display = True
    if ov:
        raise ConfigError(f"unrecognized override key(s): {sorted(ov)}")

    _validate(cfg, config_dir=os.path.dirname(os.path.abspath(path)))
    return cfg


def _validate(cfg: FinalsConfig, config_dir: str) -> None:
    if cfg.profile not in VALID_PROFILES:
        raise ConfigError(f"profile {cfg.profile!r} invalid — one of {VALID_PROFILES}")

    expected_backend = PROFILE_FLIGHT_BACKEND[cfg.profile]
    if cfg.flight_backend != expected_backend:
        raise ConfigError(
            f"profile {cfg.profile!r} pins flight_backend {expected_backend!r} "
            f"but config says {cfg.flight_backend!r} — copy-paste between "
            f"profiles? Fix the config; this guard exists so the wrong backend "
            f"can never arm."
        )

    if cfg.frame_backend not in VALID_FRAME_BACKENDS:
        raise ConfigError(
            f"frame_backend {cfg.frame_backend!r} invalid — one of {VALID_FRAME_BACKENDS}"
        )

    if cfg.marker_backend not in VALID_MARKER_BACKENDS:
        raise ConfigError(
            f"marker_backend {cfg.marker_backend!r} invalid — one of "
            f"{VALID_MARKER_BACKENDS}"
        )

    if cfg.profile == "replay":
        if cfg.drones:
            raise ConfigError(
                "profile 'replay' is laptop-only (frames from disk) — drones "
                f"must be [] but config lists {[d.id for d in cfg.drones]}"
            )
        if cfg.frame_backend != "replay":
            raise ConfigError("profile 'replay' requires frame_backend 'replay'")
        if not cfg.replay_dir:
            raise ConfigError(
                'profile \'replay\' requires "replay_dir" (directory of frames '
                "or a video file)"
            )
    else:
        if not cfg.drones:
            raise ConfigError(f"profile {cfg.profile!r} requires at least one drone")

    if cfg.frame_backend == "replay":
        # Any profile may replay frames from disk (e.g. a mock flight with
        # replay frames is the S7 vision-wiring smoke). The dir/file must
        # EXIST at load time — same philosophy as the weights guard: a
        # missing frame source dies here, loudly, not minutes later in a
        # perception thread.
        if not isinstance(cfg.replay_dir, str) or not cfg.replay_dir:
            # The isinstance check matters: a non-str (123, true) would
            # escape as a raw TypeError from os.path.join below instead of
            # the loader's ConfigError contract.
            raise ConfigError(
                f'frame_backend "replay" requires "replay_dir" (a string '
                f"path to a directory of jpg/png frames or a video file) — "
                f"got {cfg.replay_dir!r}"
            )
        candidates = [cfg.replay_dir, os.path.join(config_dir, cfg.replay_dir)]
        resolved = next((p for p in candidates
                         if os.path.isdir(p) or os.path.isfile(p)), None)
        if resolved is None:
            raise ConfigError(
                f"replay_dir {cfg.replay_dir!r} not found on disk (tried "
                f"{[os.path.abspath(c) for c in candidates]}) — check the "
                f"path (dev fixtures live at finals/tests/fixtures/frames; "
                f"run from the repo root)"
            )
        cfg.replay_dir = os.path.abspath(resolved)

    ids = [d.id for d in cfg.drones]
    if len(ids) != len(set(ids)):
        raise ConfigError(f"duplicate drone ids: {ids}")

    if cfg.profile == "sitl" and any(d.sitl_address is None
                                     for d in cfg.drones):
        # At least one drone will fall back to the top-level address — it
        # must be usable, and dying HERE beats a ValueError at adapter
        # construction minutes later (same philosophy as the weights guard).
        if not isinstance(cfg.sitl_address, str) or not cfg.sitl_address:
            raise ConfigError(
                f"profile 'sitl': top-level sitl_address must be a non-empty "
                f"string (drones without a per-drone sitl_address fall back "
                f"to it) — got {cfg.sitl_address!r}")

    if cfg.profile == "sitl" and len(cfg.drones) > 1:
        # S6/SIM-1: every concurrent SITL drone needs its OWN MAVLink udpin
        # port + mavsdk_server gRPC port (instance i -> 14540+i / 50051+i;
        # no auto-selection exists), and the altitude bands are the swarm's
        # primary collision guarantee — same rule as bench/real below.
        missing = [d.id for d in cfg.drones
                   if d.sitl_address is None or d.mavsdk_grpc_port is None]
        if missing:
            raise ConfigError(
                f"profile 'sitl' with {len(cfg.drones)} drones requires "
                f"sitl_address AND mavsdk_grpc_port on EVERY drone "
                f"(instance i: udpin://0.0.0.0:1454<i> + 5005<i+1>) — "
                f"missing on: {missing}")
        addresses = [d.sitl_address for d in cfg.drones]
        if len(set(addresses)) != len(addresses):
            raise ConfigError(
                f"profile 'sitl' multi-drone sitl_address values must be "
                f"DISTINCT (each PX4 instance sends to its own 14540+i) — "
                f"got {addresses}")
        ports = [d.mavsdk_grpc_port for d in cfg.drones]
        if len(set(ports)) != len(ports):
            raise ConfigError(
                f"profile 'sitl' multi-drone mavsdk_grpc_port values must be "
                f"DISTINCT (one mavsdk_server per drone) — got {ports}")
        bands = [d.altitude_band_m for d in cfg.drones]
        if None in bands or len(set(bands)) != len(bands):
            raise ConfigError(
                f"profile 'sitl' with {len(cfg.drones)} drones requires a "
                f"DISTINCT altitude_band_m per drone (vertical separation is "
                f"the primary collision guarantee) — got {bands}")

    if cfg.profile in ("bench", "real"):
        missing = [d.id for d in cfg.drones if d.plane_id is None]
        if missing:
            raise ConfigError(
                f"profile {cfg.profile!r} needs plane_id (Dola discovery key) "
                f"for every drone — missing on: {missing}"
            )
        # Multi-drone separation. The DEFAULT collision guarantee is the swarm
        # altitude band (distinct per drone). BUT Challenge-2A flies under a
        # ~1.1 m ceiling with a no-overfly rule, which KILLS altitude bands —
        # so the LANDING mission separates by TIME (the SafetyController launch
        # + landing corridor slots, NAV-8) + SPACE (per-drone advisory
        # sectors). A config opts into that model by declaring sector_deg on
        # EVERY drone; then bands are NOT required (and need not be distinct).
        # Either mechanism is accepted; a config that declares NEITHER on a
        # multi-drone flight is refused (silent no-separation is the bug class
        # this guard exists to prevent).
        if len(cfg.drones) > 1:
            bands = [d.altitude_band_m for d in cfg.drones]
            sectors_all = all(d.sector_deg is not None for d in cfg.drones)
            bands_distinct = None not in bands and len(set(bands)) == len(bands)
            if not bands_distinct and not sectors_all:
                missing_sectors = [d.id for d in cfg.drones
                                   if d.sector_deg is None]
                raise ConfigError(
                    f"profile {cfg.profile!r} with {len(cfg.drones)} drones "
                    f"needs a multi-drone SEPARATION mechanism: EITHER a "
                    f"DISTINCT altitude_band_m per drone (the swarm vertical "
                    f"separation; got bands {bands}) OR a sector_deg on EVERY "
                    f"drone (the NAV-8 TIME+SPACE model for the ~1.1 m-ceiling "
                    f"landing mission, where altitude bands are illegal) — "
                    f"missing sector_deg on: {missing_sectors}")

    for name, value in (("tick_hz", cfg.tick_hz),
                        ("mission_budget_s", cfg.mission_budget_s),
                        ("command_timeout_s", cfg.command_timeout_s)):
        if not value > 0:
            raise ConfigError(f"{name} must be > 0, got {value}")
    if (not isinstance(cfg.discovery_timeout_s, (int, float))
            or isinstance(cfg.discovery_timeout_s, bool)
            or not math.isfinite(cfg.discovery_timeout_s)
            or cfg.discovery_timeout_s <= 0):
        # inf would make preflight P3's Dola listen window never close.
        raise ConfigError(
            f"discovery_timeout_s must be finite and > 0 (the preflight P3 "
            f"Dola listen window), got {cfg.discovery_timeout_s!r}")
    if (not isinstance(cfg.replay_fps, (int, float))
            or isinstance(cfg.replay_fps, bool)
            or not math.isfinite(cfg.replay_fps) or cfg.replay_fps <= 0):
        # inf would make the pacing period 0 (busy spin); NaN poisons it.
        raise ConfigError(
            f"replay_fps must be finite and > 0, got {cfg.replay_fps!r}")
    # gz_camera_bridge endpoint (frame_backend "gazebo"): same shape as a
    # mavsdk_grpc_port — die HERE on a bad port, not minutes later when the
    # GazeboRgbSource fails to connect (the weights-guard philosophy).
    if (not isinstance(cfg.gazebo_video_port, int)
            or isinstance(cfg.gazebo_video_port, bool)
            or not 1024 <= cfg.gazebo_video_port <= 65535):
        raise ConfigError(
            f"gazebo_video_port must be an int in [1024, 65535] (the "
            f"sim/gz_camera_bridge TCP endpoint), got {cfg.gazebo_video_port!r}")
    if not isinstance(cfg.gazebo_video_host, str) or not cfg.gazebo_video_host:
        raise ConfigError(
            f'gazebo_video_host must be a non-empty string (e.g. "127.0.0.1"), '
            f"got {cfg.gazebo_video_host!r}")
    if cfg.frame_backend == "gazebo" and len(cfg.drones) > 1:
        # SIM-5: each camera-drone reads its OWN onboard camera through its OWN
        # gz_camera_bridge, so every drone needs a DISTINCT gazebo_video_port
        # (the top-level fallback can only serve a single-drone config). Same
        # rule shape as the multi-drone mavsdk_grpc_port guard above.
        missing = [d.id for d in cfg.drones if d.gazebo_video_port is None]
        if missing:
            raise ConfigError(
                f"frame_backend 'gazebo' with {len(cfg.drones)} drones requires "
                f"gazebo_video_port on EVERY drone (each reads its own onboard "
                f"camera via its own bridge; the top-level fallback serves only a "
                f"single drone) — missing on: {missing}")
        gz_ports = [d.gazebo_video_port for d in cfg.drones]
        if len(set(gz_ports)) != len(gz_ports):
            raise ConfigError(
                f"frame_backend 'gazebo' multi-drone gazebo_video_port values "
                f"must be DISTINCT (one gz_camera_bridge per drone) — got "
                f"{gz_ports}")
    if not 0 <= cfg.min_battery_pct <= 100:
        raise ConfigError(f"min_battery_pct {cfg.min_battery_pct} out of range [0, 100]")
    if cfg.video_channel_order not in ("rgb", "bgr"):
        raise ConfigError(f'video_channel_order must be "rgb" or "bgr", got {cfg.video_channel_order!r}')
    if cfg.use_uwb and not cfg.uwb_serial_port:
        # Auto-detect exists in UWBParserThread but preflight needs a deterministic target.
        raise ConfigError('use_uwb=true requires "uwb_serial_port" (e.g. "COM7" / "/dev/ttyUSB0")')

    _validate_detector(cfg.detector, config_dir)
    _validate_guards(cfg)
    _resolve_arena(cfg, config_dir)


def _resolve_arena(cfg: FinalsConfig, config_dir: str) -> None:
    """Load cfg.arena_name -> cfg.arena from a JSON map file (S11/NAV-0). No
    arena_name -> arena stays None (the convoy configs don't navigate). Resolves
    <name>.json under the config file's dir (so a profile + its arena travel
    together), then the repo-root finals/configs/arenas/. Dies HERE on a missing
    or malformed file — the weights-guard philosophy. NAV-2 hardens the SEMANTIC
    arena validation (bounds ordering, pads within bounds, unique ids)."""
    if cfg.arena_name is None:
        return
    name = cfg.arena_name
    if not isinstance(name, str) or not name:
        raise ConfigError(
            f"arena_name must be a non-empty string (a map basename under "
            f"finals/configs/arenas/), got {name!r}")
    filename = name if name.endswith(".json") else f"{name}.json"
    candidates = [
        os.path.join(config_dir, "arenas", filename),
        os.path.join(config_dir, filename),
        os.path.join("finals", "configs", "arenas", filename),
    ]
    resolved = next((p for p in candidates if os.path.isfile(p)), None)
    if resolved is None:
        raise ConfigError(
            f"arena_name {name!r}: map file not found (tried "
            f"{[os.path.abspath(c) for c in candidates]}) — add "
            f"finals/configs/arenas/{filename} (NAV-2 ships a sample)")
    try:
        with open(resolved, "r", encoding="utf-8") as f:
            raw = json.load(f)
    except json.JSONDecodeError as e:
        raise ConfigError(f"{resolved}: invalid JSON — {e}") from e
    except OSError as e:
        raise ConfigError(
            f"arena_name {name!r}: cannot read map file {resolved} — {e}; "
            f"check the file exists, is readable, and is not locked") from e
    cfg.arena = ArenaMap.from_dict(raw, name=name)


def _validate_guards(cfg: FinalsConfig) -> None:
    """Guard thresholds (S5). Runs AFTER CLI overrides, so e.g. a --budget
    override is checked against landing_reserve_s too."""
    g = cfg.guards

    def _num(name: str, value, *, zero_ok: bool = False) -> None:
        if (not isinstance(value, (int, float)) or isinstance(value, bool)
                or not math.isfinite(value)
                or value < 0 or (value == 0 and not zero_ok)):
            raise ConfigError(
                f"guards.{name} must be finite and "
                f"{'>= 0' if zero_ok else '> 0'}, got {value!r}")

    _num("telemetry_stale_s", g.telemetry_stale_s)
    # Lazy import: only the one constant, and only at validation time.
    from finals.mission.agent import DEFAULT_TELEMETRY_STALE_S
    if g.telemetry_stale_s >= DEFAULT_TELEMETRY_STALE_S:
        raise ConfigError(
            f"guards.telemetry_stale_s ({g.telemetry_stale_s}) must stay "
            f"UNDER the agent's {DEFAULT_TELEMETRY_STALE_S:.0f} s "
            f"SensorTimeout backstop — at/above it the layering inverts and "
            f"every stale-telemetry event becomes an emergency FAILED "
            f"instead of the orderly guard landing")
    _num("video_stale_s", g.video_stale_s)
    _num("battery_warn_pct", g.battery_warn_pct, zero_ok=True)
    if not g.battery_warn_pct <= 100:
        raise ConfigError(
            f"guards.battery_warn_pct {g.battery_warn_pct} out of range [0, 100]")
    if g.battery_warn_pct < cfg.min_battery_pct:
        raise ConfigError(
            f"guards.battery_warn_pct ({g.battery_warn_pct}) < min_battery_pct "
            f"({cfg.min_battery_pct}) — the warn must come BEFORE the floor "
            f"on the way down")
    _num("landing_reserve_s", g.landing_reserve_s, zero_ok=True)
    if g.landing_reserve_s >= cfg.mission_budget_s:
        raise ConfigError(
            f"guards.landing_reserve_s ({g.landing_reserve_s}) >= "
            f"mission_budget_s ({cfg.mission_budget_s}) — the mission clock "
            f"guard would land everything at t=0 (check a --budget override "
            f"shrinking the budget under the reserve)")
    if g.phase_timeout_s is not None:
        _num("phase_timeout_s", g.phase_timeout_s)
    if g.geofence_radius_m is not None:
        _num("geofence_radius_m", g.geofence_radius_m)
    if g.geofence_alt_m is not None:
        _num("geofence_alt_m", g.geofence_alt_m)
        if g.geofence_radius_m is None:
            raise ConfigError(
                "guards.geofence_alt_m is set but guards.geofence_radius_m is "
                "not — GeofenceLite needs the radius; without it the altitude "
                "limit would be silently ignored")
    _num("loop_overrun_factor", g.loop_overrun_factor)
    if not g.loop_overrun_factor > 1:
        raise ConfigError(
            f"guards.loop_overrun_factor must be > 1 (a factor <= 1 trips on "
            f"a healthy loop), got {g.loop_overrun_factor!r}")
    if (not isinstance(g.loop_overrun_ticks, int)
            or isinstance(g.loop_overrun_ticks, bool)
            or g.loop_overrun_ticks < 1):
        raise ConfigError(
            f"guards.loop_overrun_ticks must be an int >= 1, got "
            f"{g.loop_overrun_ticks!r}")
    _num("land_retry_period_s", g.land_retry_period_s)
    _num("land_retry_window_s", g.land_retry_window_s)
    if g.land_retry_window_s < g.land_retry_period_s:
        raise ConfigError(
            f"guards.land_retry_window_s ({g.land_retry_window_s}) < "
            f"land_retry_period_s ({g.land_retry_period_s}) — the landing "
            f"ladder would never retry")
    _num("slot_wait_s", g.slot_wait_s)
    _num("launch_slot_wait_s", g.launch_slot_wait_s)
