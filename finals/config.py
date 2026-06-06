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

VALID_PROFILES = ("mock", "sitl", "replay", "bench", "real")
VALID_FRAME_BACKENDS = ("none", "gazebo", "pyhulax", "replay")
VALID_DETECTOR_BACKENDS = ("none", "ultralytics", "canned")

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


@dataclass
class DroneConfig:
    id: str                                     # "alpha" — used in logs, sightings, prefixes
    plane_id: Optional[int] = None              # Dola discovery key (REQUIRED for bench/real)
    phases: List[str] = field(default_factory=list)   # PHASE_REGISTRY names, run in order
    led_rgb: Optional[Tuple[int, int, int]] = None    # identity colour (bench/real)
    altitude_band_m: Optional[float] = None     # swarm vertical separation (1.2/1.7/2.2)
    zone: Dict[str, Any] = field(default_factory=dict)  # per-drone search params (briefing-shaped)


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
    min_battery_pct: float = 20.0
    video_channel_order: str = "rgb"            # what .to_rgb() ACTUALLY returns — bench-verified
    camera_hfov_deg: Optional[float] = None     # needed for Sighting.bearing_deg; bench-measured
    sitl_address: str = "udpin://0.0.0.0:14540"
    replay_dir: Optional[str] = None            # REQUIRED for profile=replay
    use_uwb: bool = False
    uwb_serial_port: Optional[str] = None
    guards: GuardsConfig = field(default_factory=GuardsConfig)


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
        optional=("plane_id", "led_rgb", "altitude_band_m", "zone"),
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
    return DroneConfig(
        id=str(data["id"]),
        plane_id=data.get("plane_id"),
        phases=list(phases),
        led_rgb=led,
        altitude_band_m=data.get("altitude_band_m"),
        zone=dict(data.get("zone", {})),
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
            "min_battery_pct", "video_channel_order", "camera_hfov_deg",
            "sitl_address", "replay_dir", "use_uwb", "uwb_serial_port",
            "guards",
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
                  "land_retry_period_s", "land_retry_window_s", "slot_wait_s"),
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
            "min_battery_pct", "video_channel_order", "camera_hfov_deg",
            "sitl_address", "replay_dir", "use_uwb", "uwb_serial_port",
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

    ids = [d.id for d in cfg.drones]
    if len(ids) != len(set(ids)):
        raise ConfigError(f"duplicate drone ids: {ids}")

    if cfg.profile in ("bench", "real"):
        missing = [d.id for d in cfg.drones if d.plane_id is None]
        if missing:
            raise ConfigError(
                f"profile {cfg.profile!r} needs plane_id (Dola discovery key) "
                f"for every drone — missing on: {missing}"
            )
        bands = [d.altitude_band_m for d in cfg.drones]
        if len(cfg.drones) > 1 and (None in bands or len(set(bands)) != len(bands)):
            raise ConfigError(
                f"profile {cfg.profile!r} with {len(cfg.drones)} drones requires "
                f"a DISTINCT altitude_band_m per drone (vertical separation is "
                f"the primary collision guarantee) — got {bands}"
            )

    for name, value in (("tick_hz", cfg.tick_hz),
                        ("mission_budget_s", cfg.mission_budget_s),
                        ("command_timeout_s", cfg.command_timeout_s)):
        if not value > 0:
            raise ConfigError(f"{name} must be > 0, got {value}")
    if not 0 <= cfg.min_battery_pct <= 100:
        raise ConfigError(f"min_battery_pct {cfg.min_battery_pct} out of range [0, 100]")
    if cfg.video_channel_order not in ("rgb", "bgr"):
        raise ConfigError(f'video_channel_order must be "rgb" or "bgr", got {cfg.video_channel_order!r}')
    if cfg.use_uwb and not cfg.uwb_serial_port:
        # Auto-detect exists in UWBParserThread but preflight needs a deterministic target.
        raise ConfigError('use_uwb=true requires "uwb_serial_port" (e.g. "COM7" / "/dev/ttyUSB0")')

    _validate_detector(cfg.detector, config_dir)
    _validate_guards(cfg)


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
