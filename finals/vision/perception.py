"""PerceptionLoop — per-drone sampler: video -> marker detect (+ optional
YOLO) -> SightingLog + SightingBus. Plus the bearing math and the shared
worker-thread detection callback.

Surface (S7, implemented):
- One async loop per drone over a VideoSource. Marker detection (the
  injected `detect_marker` callable — finals/vision/aruco.py builds it from
  config) is cheap (~2-5 ms) and runs SYNCHRONOUSLY on EVERY new frame at
  sample_hz (~10 Hz; the qualifier ran 5 Hz, qualifier_run.py:236-252) —
  high rates matter for reading markers on MOVING robots. YOLO frames go to
  the shared DetectorPool worker pool, config-gated.
- Frames are deduped on FrameStamped.frame_number (a monotonic delivery
  counter): a latest-frame source sampled faster than it delivers must not
  produce duplicate sightings of the same pixels.
- Detectors emit MINIMAL Sightings; this loop enriches them
  (dataclasses.replace — Sighting is frozen) with telemetry yaw/alt,
  bearing_deg, and the dead-reckoned est_north/east + pos_quality.
- CSV + bus at the PUBLISH SITE (binding, decided S7): SightingBus is
  bounded (maxlen eviction is silent by design), so a drain-side CSV writer
  could lose score-relevant rows under burst. Every sighting is
  slog.append()ed (fsync) the moment it exists, THEN bus.publish()ed. The
  orchestrator's drain stays an event mirror only. A SightingLogError is
  typed-caught: scream + flag, keep publishing to the bus — intel flow
  survives forensics death.
- Thread -> asyncio handoff ONLY via SightingBus: the detection callback
  (fires on a WORKER thread) touches nothing but the bus and the log
  (both thread-safe); agents see sightings when THEY drain the bus.
- last_frame_ts() is the VideoWatchdog feed (main.py wires it into the
  agent's GuardContext): it stamps the last frame THIS LOOP SAMPLED, not
  the source's latest — so a dead perception task stales out through the
  same guard as a dead source.
- shed(reason) is the DEGRADE_DETECTION consumer (guards.py reconciliation
  8: the LoopOverrunGuard ladder and the VideoWatchdog finally have a
  shedding target): one-way latch to degraded_hz (default 1 Hz), loudly
  logged, never raises. Also self-trips when the detector pool reports
  drops (the queue-backing-up signal).
- Health: source.healthy False for > unhealthy_log_period_s -> loud stderr
  + event, repeated at most once per period — never silent, never spam.
- A raising detect_marker/source is NOT blanket-caught here (the whitelist
  stays guards.py / orchestrator.py / vision/detector.py): a systematic
  detector bug would raise on every frame, so retrying buys nothing — the
  task dies, the wiring's done-callback screams, and last_frame_ts going
  stale trips the VideoWatchdog DEGRADE. Failure surfaces through the guard
  chain by construction.

Derives from: detection_loop + make_detection_callback
(qualifier_run.py:192-252, proven in sim). Bugs fixed in adaptation:
- detection_loop silently skipped frames whenever pose/depth context was
  missing (qualifier_run.py:247) — here a missing telemetry source just
  means un-enriched sightings (bearing/yaw None), never a silent drop.
- the callback's barrel dedup/scoring is gone (the convoy MOVES; the log is
  append-only — finals/sightings.py).
- the bearing sign: types.py's original comment said `yaw + offset`, which
  is wrong under the binding CCW+ yaw convention (a target right of frame
  centre lies CLOCKWISE of the nose, i.e. at DECREASING yaw) — see
  finals/flight/dead_reckon.py "KNOWN UPSTREAM CONFLICT". bearing_from_bbox
  implements yaw MINUS offset and the sign is pinned by tests.

Pure stdlib — NO cv2/numpy imports (enforced by tests/test_conventions.py:
this module is deliberately NOT in SDK_ALLOWED); detectors arrive as
injected callables.

Session: S7 (implemented).
"""
from __future__ import annotations

import asyncio
import dataclasses
import math
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from finals.events import EventLog, EventLogError
from finals.sightings import SightingBus, SightingLog, SightingLogError
from finals.types import (FrameStamped, PositionQuality, Sighting,
                          Telemetry)
from finals.vision.video import VideoSource

#: (frame, drone_id) -> minimal Sightings (finals/vision/aruco.py builds these).
MarkerDetector = Callable[[FrameStamped, str], List[Sighting]]
#: The DetectorPool callback contract (finals/vision/detector.py).
DetectionCallback = Callable[[List[Dict[str, Any]], Optional[Any],
                              Dict[str, Any]], None]


# ============================================================
# Bearing math (pure)
# ============================================================
def bearing_from_bbox(yaw_deg: float,
                      bbox_xyxy: Tuple[float, float, float, float],
                      frame_w: int, hfov_deg: float) -> float:
    """World bearing to a bbox centre, in the BINDING yaw frame
    (finals/flight/dead_reckon.py): CCW-positive viewed from above, 0 = the
    boot heading, normalized to (-180, 180].

        bearing = yaw_deg - (cx - w/2) / w * hfov_deg

    MINUS, not plus: pixels grow rightward, and a target right of frame
    centre lies CLOCKWISE of the nose — at DECREASING yaw under CCW+. (The
    original types.py comment had the sign wrong; resolved here, S7.)
    """
    for name, value in (("yaw_deg", yaw_deg), ("hfov_deg", hfov_deg),
                        ("bbox_xyxy[0]", bbox_xyxy[0]),
                        ("bbox_xyxy[2]", bbox_xyxy[2])):
        if not isinstance(value, (int, float)) or isinstance(value, bool) \
                or not math.isfinite(value):
            raise ValueError(
                f"bearing_from_bbox: {name} must be a finite number, got "
                f"{value!r} — a NaN here poisons every bearing silently "
                f"(the dead_reckon.py bug class); check telemetry/"
                f"camera_hfov_deg/the detector's bbox")
    if not isinstance(frame_w, int) or isinstance(frame_w, bool) \
            or frame_w <= 0:
        raise ValueError(
            f"bearing_from_bbox: frame_w must be an int > 0, got "
            f"{frame_w!r} — check Sighting.frame_shape")
    cx = (bbox_xyxy[0] + bbox_xyxy[2]) / 2.0
    bearing = yaw_deg - (cx - frame_w / 2.0) / frame_w * hfov_deg
    bearing %= 360.0
    if bearing > 180.0:
        bearing -= 360.0
    return bearing


def _enrich(s: Sighting, telemetry: Optional[Telemetry],
            camera_hfov_deg: Optional[float]) -> Sighting:
    """Minimal detector Sighting + telemetry context -> enriched Sighting
    (frozen dataclass: replace, never mutate). camera_hfov_deg None ->
    bearing_deg None (the config open question: bench-measure the HULA HFOV)."""
    if telemetry is None:
        return s
    yaw = telemetry.yaw_deg
    bearing = None
    if yaw is not None and camera_hfov_deg is not None:
        bearing = bearing_from_bbox(yaw, s.bbox_xyxy, s.frame_shape[1],
                                    camera_hfov_deg)
    north = east = None
    if telemetry.position_m is not None:
        north, east = telemetry.position_m[0], telemetry.position_m[1]
    return dataclasses.replace(
        s, drone_yaw_deg=yaw, drone_alt_m=telemetry.altitude_m,
        bearing_deg=bearing, pos_quality=telemetry.position_quality,
        est_north_m=north, est_east_m=east)


# ============================================================
# CSV recording health (shared by the marker path + the YOLO callback)
# ============================================================
class CsvRecordingHealth:
    """Tracks sighting-CSV append health across EVERY publish site (each
    drone's marker path + the shared worker-thread callback), thread-safe.

    Design (reviewed, S7): finals/sightings.py raises SightingLogError for
    BOTH (a) codec refusals of one malformed row — which leave the log
    perfectly healthy for the next valid row — and (b) real append/fsync
    failures — which poison the log instance. Latching "dead" on the first
    error would convert one bad sighting into ZERO CSV score rows for the
    rest of the mission (against the publish-site charter: never lose score
    rows). So: every failure screams with full context and drops THAT row;
    only `latch_after` CONSECUTIVE failures (a truly dead/poisoned log fails
    every append) latch `dead` and stop the append attempts. The mission
    event log's `sighting` mirror remains the recovery source either way.
    """

    def __init__(self, latch_after: int = 3):
        if not isinstance(latch_after, int) or isinstance(latch_after, bool) \
                or latch_after < 1:
            raise ValueError(
                f"CsvRecordingHealth: latch_after must be an int >= 1, got "
                f"{latch_after!r}")
        self._latch_after = latch_after
        self._lock = threading.Lock()
        self._consecutive = 0
        self._failures_total = 0
        self._dead = False

    @property
    def dead(self) -> bool:
        return self._dead

    @property
    def failures_total(self) -> int:
        with self._lock:
            return self._failures_total

    def note_success(self) -> None:
        with self._lock:
            self._consecutive = 0

    def note_failure(self, drone_id: str, source: str, exc: Exception) -> None:
        """Scream (always) + latch after `latch_after` consecutive failures.
        Never raises — the bus publish after it must always happen."""
        with self._lock:
            self._consecutive += 1
            self._failures_total += 1
            n, total = self._consecutive, self._failures_total
            latched_now = (not self._dead and n >= self._latch_after)
            if latched_now:
                self._dead = True
        print(f"[perception] ERROR: sighting CSV append FAILED for "
              f"{drone_id}/{source} (consecutive #{n}, total {total}) — the "
              f"row is LOST from sightings.csv but still on the bus and in "
              f"the mission.jsonl 'sighting' mirror: {exc} — check disk "
              f"space / the file locked by antivirus or Excel",
              file=sys.stderr, flush=True)
        if latched_now:
            print(f"[perception] CRITICAL: {n} CONSECUTIVE CSV append "
                  f"failures — the sighting log is judged DEAD; no further "
                  f"appends will be attempted. RECOVER the score rows from "
                  f"the mission.jsonl 'sighting' events of this run",
                  file=sys.stderr, flush=True)


# ============================================================
# The shared worker-thread callback (one per pool, serves all drones)
# ============================================================
def make_detection_callback(bus: SightingBus,
                            slog: Optional[SightingLog], *,
                            class_map: Dict[str, str],
                            camera_hfov_deg: Optional[float],
                            csv_health: Optional[CsvRecordingHealth] = None
                            ) -> DetectionCallback:
    """Build the DetectorPool callback. Runs ON WORKER THREADS — it may
    touch ONLY the bus and the log (both thread-safe); never agents, events,
    or asyncio objects (the binding handoff rule).

    class_map semantics: empty map = identity passthrough (canned/dev runs);
    non-empty map = filter-and-rename — an unmapped class is skipped with a
    once-per-name stderr warning (the qualifier silently ignored them).

    csv_health: pass the RUN-WIDE CsvRecordingHealth (main.py does) so CSV
    death is observable in one place across the marker + YOLO paths.
    """
    warned: set = set()
    health = csv_health if csv_health is not None else CsvRecordingHealth()
    lock = threading.Lock()

    def callback(detections: List[Dict[str, Any]], annotated: Any,
                 context: Dict[str, Any]) -> None:
        drone_id = context.get("drone_id")
        ts = context.get("ts")
        shape = context.get("frame_shape")
        if drone_id is None or ts is None or shape is None:
            with lock:
                first = "context" not in warned
                warned.add("context")
            if first:
                print("[perception] ERROR: detection context is missing "
                      f"drone_id/ts/frame_shape (got keys "
                      f"{sorted(context)}) — wiring bug; these detections "
                      f"are DROPPED (warned once)",
                      file=sys.stderr, flush=True)
            return
        yaw = context.get("yaw")
        position_m = context.get("position_m")
        telemetry_like = Telemetry(
            ts=float(ts), yaw_deg=yaw, altitude_m=context.get("alt"),
            position_m=position_m,
            # Same-frame parity with the marker path: a YOLO sighting must
            # not systematically lose the position estimate its frame had.
            position_quality=context.get("position_quality",
                                         PositionQuality.NONE))
        for det in detections:
            name = det["class_name"]
            if class_map:
                mapped = class_map.get(name)
                if mapped is None:
                    with lock:
                        first = name not in warned
                        warned.add(name)
                    if first:
                        print(f"[perception] WARNING: detector class "
                              f"{name!r} has no detector.class_map entry — "
                              f"its detections are skipped (warned once per "
                              f"class)", file=sys.stderr, flush=True)
                    continue
            else:
                mapped = name
            s = Sighting(
                drone_id=str(drone_id),
                ts=float(ts),
                source="yolo",
                class_name=mapped,
                marker_id=None,
                bbox_xyxy=tuple(float(v) for v in det["bbox"]),
                confidence=float(det["confidence"]),
                frame_shape=(int(shape[0]), int(shape[1])),
                frame_number=context.get("frame_number"),
                frame_path=context.get("saved_path"),
            )
            s = _enrich(s, telemetry_like, camera_hfov_deg)
            if slog is not None and not health.dead:
                try:
                    slog.append(s)
                    health.note_success()
                except SightingLogError as e:
                    health.note_failure(s.drone_id, s.source, e)
            bus.publish(s)

    return callback


# ============================================================
# PerceptionLoop
# ============================================================
class PerceptionLoop:
    """One drone's sampler. Built by main.py per drone (replay runner: one
    per replay source); runs as its own asyncio task next to the agents."""

    def __init__(self, drone_id: str, source: VideoSource, bus: SightingBus,
                 events: EventLog, *,
                 detect_marker: MarkerDetector,
                 slog: Optional[SightingLog] = None,
                 detector: Optional[Any] = None,
                 camera_hfov_deg: Optional[float] = None,
                 csv_health: Optional[CsvRecordingHealth] = None,
                 sample_hz: float = 10.0,
                 degraded_hz: float = 1.0,
                 unhealthy_log_period_s: float = 5.0,
                 clock: Callable[[], float] = time.monotonic):
        if not isinstance(drone_id, str) or not drone_id:
            raise ValueError(
                f"PerceptionLoop: drone_id must be a non-empty str, got "
                f"{drone_id!r} — check the wiring")
        if not isinstance(source, VideoSource):
            raise ValueError(
                f"PerceptionLoop({drone_id!r}): source must be a "
                f"VideoSource, got {type(source).__name__!r} — check the "
                f"wiring")
        if not callable(detect_marker):
            raise ValueError(
                f"PerceptionLoop({drone_id!r}): detect_marker must be "
                f"callable (finals.vision.aruco.make_marker_detector builds "
                f"it), got {detect_marker!r}")
        for name, value in (("sample_hz", sample_hz),
                            ("degraded_hz", degraded_hz),
                            ("unhealthy_log_period_s", unhealthy_log_period_s)):
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(
                    f"PerceptionLoop({drone_id!r}): {name} must be finite "
                    f"and > 0, got {value!r}")
        if degraded_hz > sample_hz:
            raise ValueError(
                f"PerceptionLoop({drone_id!r}): degraded_hz ({degraded_hz}) "
                f"> sample_hz ({sample_hz}) — shedding must LOWER the rate")

        self._drone_id = drone_id
        self._source = source
        self._bus = bus
        self._events = events
        self._detect_marker = detect_marker
        self._slog = slog
        self._detector = detector
        self._camera_hfov_deg = camera_hfov_deg
        self._sample_hz = float(sample_hz)
        self._degraded_hz = float(degraded_hz)
        self._unhealthy_period_s = float(unhealthy_log_period_s)
        self._clock = clock

        self._telemetry_fn: Optional[Callable[[], Optional[Telemetry]]] = None
        self._last_frame_ts: Optional[float] = None
        self._last_frame_key: Optional[Any] = None
        self._degraded = False
        self._csv_health = (csv_health if csv_health is not None
                            else CsvRecordingHealth())
        self._unhealthy_since: Optional[float] = None
        self._last_unhealthy_log: Optional[float] = None
        self._last_dropped_total = 0
        self._frames_sampled = 0
        self._marker_sightings = 0
        self._submits = 0

    # ---------------- wiring hooks ----------------
    def set_telemetry_source(self, fn: Callable[[], Optional[Telemetry]]) -> None:
        """Wire-once enrichment source (the agent's cached telemetry) —
        post-construction because agents and perception reference each other
        (agent gets last_frame_ts; perception gets last_telemetry)."""
        if not callable(fn):
            raise ValueError(
                f"PerceptionLoop({self._drone_id!r}): telemetry source must "
                f"be callable, got {fn!r}")
        self._telemetry_fn = fn

    def last_frame_ts(self) -> Optional[float]:
        """ts of the last frame THIS LOOP sampled (None = none yet) — the
        agent's VideoWatchdog feed. Same monotonic domain as the source's
        stamps (inject the same fake clock in tests)."""
        return self._last_frame_ts

    def shed(self, reason: str) -> None:
        """DEGRADE_DETECTION consumer: one-way latch to degraded_hz. Loud,
        idempotent, NEVER raises (it is called from the agent's guard-trip
        path — see the contract in mission/agent.py)."""
        if self._degraded:
            return
        self._degraded = True
        print(f"[perception:{self._drone_id}] DEGRADE: sampling shed "
              f"{self._sample_hz:g} Hz -> {self._degraded_hz:g} Hz — "
              f"{reason}", file=sys.stderr, flush=True)
        try:
            self._events.log(self._drone_id, "perception_degraded",
                             reason=reason, from_hz=self._sample_hz,
                             to_hz=self._degraded_hz)
        except EventLogError as e:
            print(f"[perception:{self._drone_id}] WARNING: could not log "
                  f"perception_degraded: {e}", file=sys.stderr, flush=True)

    @property
    def degraded(self) -> bool:
        return self._degraded

    @property
    def current_period_s(self) -> float:
        return 1.0 / (self._degraded_hz if self._degraded else self._sample_hz)

    def stats(self) -> dict:
        return {
            "drone_id": self._drone_id,
            "frames_sampled": self._frames_sampled,
            "marker_sightings": self._marker_sightings,
            "detector_submits": self._submits,
            "degraded": self._degraded,
            "csv_dead": self._csv_health.dead,
            "csv_append_failures": self._csv_health.failures_total,
            "last_frame_ts": self._last_frame_ts,
        }

    # ---------------- the loop ----------------
    async def run(self, *, deadline: float,
                  stop_event: asyncio.Event) -> None:
        """Sample until the mission deadline or the stop event (bounded —
        convention 3). detect_marker/source exceptions propagate: the
        wiring's done-callback screams and the VideoWatchdog stales out
        (see the module docstring)."""
        while not stop_event.is_set() and self._clock() < deadline:
            self.sample_once()
            try:
                await asyncio.wait_for(stop_event.wait(),
                                       timeout=self.current_period_s)
            except asyncio.TimeoutError:
                pass    # period elapsed — the normal beat

    def sample_once(self) -> None:
        """One synchronous sampling pass (run() calls this each beat; tests
        drive it LOCKSTEP for deterministic every-frame-exactly-once pins)."""
        self._check_health()
        frame = self._source.get_frame()
        if frame is None:
            return                      # nothing yet; health path logs
        key = (frame.frame_number if frame.frame_number is not None
               else frame.ts)
        if key == self._last_frame_key:
            return                      # latest-frame source resampled
        self._last_frame_key = key
        self._last_frame_ts = frame.ts
        self._frames_sampled += 1

        telemetry = self._telemetry_fn() if self._telemetry_fn else None

        sightings = self._detect_marker(frame, self._drone_id)
        for s in sightings:
            s = _enrich(s, telemetry, self._camera_hfov_deg)
            self._marker_sightings += 1
            if self._slog is not None and not self._csv_health.dead:
                try:
                    self._slog.append(s)
                    self._csv_health.note_success()
                except SightingLogError as e:
                    # One malformed row must not kill a mission's CSV (the
                    # log itself stays healthy after a codec refusal); a
                    # truly dead log latches after consecutive failures —
                    # see CsvRecordingHealth.
                    self._csv_health.note_failure(self._drone_id, s.source, e)
            self._bus.publish(s)

        if self._detector is not None:
            dropped = self._detector.dropped_total
            if dropped > self._last_dropped_total:
                self._last_dropped_total = dropped
                self.shed(f"detector queue backing up ({dropped} frame(s) "
                          f"dropped so far) — inference slower than "
                          f"submission")
            self._submits += 1
            self._detector.submit_image(frame.image, context={
                # FRESH dict every submit — the pool mutates it (saved_path).
                "drone_id": self._drone_id,
                "ts": frame.ts,
                "yaw": telemetry.yaw_deg if telemetry else None,
                "alt": telemetry.altitude_m if telemetry else None,
                # Same-frame parity: the YOLO sighting gets the SAME position
                # estimate the marker sighting on this frame gets.
                "position_m": telemetry.position_m if telemetry else None,
                "position_quality": (telemetry.position_quality if telemetry
                                     else PositionQuality.NONE),
                "frame_shape": (frame.image.shape[0], frame.image.shape[1]),
                "frame_number": frame.frame_number,
            })

    def _check_health(self) -> None:
        now = self._clock()
        if self._source.healthy:
            self._unhealthy_since = None
            return
        if self._unhealthy_since is None:
            self._unhealthy_since = now
            return
        if now - self._unhealthy_since < self._unhealthy_period_s:
            return                      # not unhealthy for long enough yet
        if (self._last_unhealthy_log is not None
                and now - self._last_unhealthy_log < self._unhealthy_period_s):
            return                      # already screamed this period
        self._last_unhealthy_log = now
        unhealthy_s = now - self._unhealthy_since
        print(f"[perception:{self._drone_id}] WARNING: video source "
              f"unhealthy for {unhealthy_s:.1f} s (stale/errored/exhausted) "
              f"— detection is blind; flight unaffected — check the "
              f"stream", file=sys.stderr, flush=True)
        try:
            self._events.log(self._drone_id, "perception_unhealthy",
                             unhealthy_s=round(unhealthy_s, 1))
        except EventLogError as e:
            print(f"[perception:{self._drone_id}] WARNING: could not log "
                  f"perception_unhealthy: {e}", file=sys.stderr, flush=True)
