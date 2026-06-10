r"""live_view.py — REAL-TIME CV feed visualiser for ONE HULA drone (NO FLIGHT).

WHY this exists: the operator needs to SEE what the drone's camera sees, with the
ArUco + YOLO overlays the mission stack would compute, live, while aiming markers
and the landing pad at the lens — to confirm the field-marker dict-lock decodes
the right ids (and which "ghost" mis-decodes leak in), and to eyeball the
landing-pad detector. It is READ-ONLY: it connects, opens the camera, overlays
detections in a `cv2.imshow` window, and NEVER issues a flight command.

It REUSES the production seams (so the window validates the real code path) and
the hula_smoke.py helpers (the ArUco detector build + the field-id allowlist +
the synthetic-frame self-test double), so this file adds only the drawing/HUD/
vote-aggregation layer.

ARCHITECTURE (testability): the pure drawing + aggregation logic lives in
standalone functions that take a frame ndarray + detection data and RETURN an
annotated ndarray (annotate_aruco / annotate_yolo / draw_hud) plus pure-Python
vote helpers (update_votes / dominant_id / classify_ghost). The cv2.imshow window
loop (`_run_window`) is SEPARATE and the tests NEVER call it — CI must never open
a window. The headless `--no-window`/`--headless` path (and `--fake`) iterates
frames and exits WITHOUT imshow, so it runs over SSH / in CI.

cv2 / numpy / ultralytics / pyhulax are ALL imported lazily INSIDE functions (the
package seam discipline — test_conventions.py FORBIDDEN_SDK_ROOTS). This file is a
TOOL, but the top-level-import scan still applies, so nothing SDK lands at module
top level.

Self-test (run ONLINE first — no drone, no window):
    python finals\tools\live_view.py --fake --headless --frames 30
    python finals\tools\live_view.py --fake                 # opens a window locally

Live run (OFFLINE, on the drone Wi-Fi) — opens the window:
  1. Join the drone Wi-Fi: SSID `Hula-2502180050`, pw `12345678`.
  2. Windows firewall OFF for inbound UDP (the heartbeat/telemetry); a stale
     `bind_client` is cleared by power-cycling the drone.
  3. pyhulax ports: control/telemetry TCP 8888, discovery UDP 8668.
    python finals\tools\live_view.py --ip 192.168.100.1
    python finals\tools\live_view.py --plane-id 7        # discover, pick plane_id 7

Hotkeys (in the window):  a = toggle ArUco   y = toggle YOLO
    d = toggle dedup/vote view   s = snapshot (raw + annotated)   q = quit
"""
from __future__ import annotations

import argparse
import sys
import time
import traceback
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

# Make the repo importable whether launched as a path or `-m`.
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# Stable, SDK-free helpers from the bring-up smoke (the field-id allowlist + the
# ArUco detector build + synthetic frames + the fake video/drone doubles). These
# imports are pure-Python at import time (hula_smoke keeps cv2/numpy/pyhulax lazy
# too), so importing it here does NOT pull an SDK to module top level.
from finals.errors import FlightError
from finals.tools import hula_smoke as hs

# cv2 / numpy / ultralytics / pyhulax are imported LAZILY inside functions only.


# ============================================================
# Tiny tee logger (mirrors hula_smoke._Log style; own copy so this tool does not
# depend on hula_smoke internals that other agents may be editing).
# ============================================================
class _Log:
    """Timestamped tee: prints to the console and (optionally) appends to a log
    file. Fail-loud: warn/error are counted; exc() logs a full traceback."""

    def __init__(self, path: Optional[Path] = None):
        self._fh = open(path, "w", encoding="utf-8") if path is not None else None
        self.path = path
        self.warnings = 0
        self.errors = 0

    def line(self, msg: str = "") -> None:
        stamp = time.strftime("%H:%M:%S")
        text = f"[{stamp}] {msg}" if msg else ""
        print(text, flush=True)
        if self._fh is not None:
            self._fh.write(text + "\n")
            self._fh.flush()

    def warn(self, msg: str) -> None:
        self.warnings += 1
        self.line(f"WARN: {msg}")

    def error(self, msg: str) -> None:
        self.errors += 1
        self.line(f"ERROR: {msg}")

    def exc(self, where: str) -> None:
        """Log the active exception with a full traceback (never swallowed)."""
        self.errors += 1
        self.line(f"ERROR in {where}:")
        for ln in traceback.format_exc().rstrip().splitlines():
            self.line(f"    {ln}")

    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.close()
            except OSError:
                pass


# ============================================================
# Colors (BGR — cv2 order) + sizing. Module-level constants, not mutable globals.
# ============================================================
#: field marker (in the {11,45,51,67,101} allowlist) — bright green.
_COLOR_FIELD = (0, 255, 0)
#: GHOST id (decoded OUTSIDE the allowlist = a mis-decode) — warning orange.
_COLOR_GHOST = (0, 140, 255)
#: YOLO box base color — green (every box drawn; NO edge rejection — that concept
#: was retired from this repo, see the module docstring / hula_smoke YOLO notes).
_COLOR_YOLO = (0, 220, 0)
#: lower-confidence YOLO box tint (cosmetic only; still drawn, never rejected).
_COLOR_YOLO_LOW = (0, 200, 200)
#: HUD text + panel.
_COLOR_HUD = (255, 255, 255)
_COLOR_HUD_BG = (0, 0, 0)
_COLOR_DOMINANT = (0, 255, 255)   # the currently-dominant ArUco id — cyan

#: confidence at/above which a YOLO box uses the full-green color (cosmetic).
_YOLO_CONF_BRIGHT = 0.50


# ============================================================
# PURE aggregation logic (NO cv2 / numpy — runs on the bare venv)
# ============================================================
def classify_ghost(marker_id: int,
                   field_ids: Sequence[int] = None) -> bool:
    """True if `marker_id` is a GHOST (a suspected cross-dict / error-correction
    mis-decode) — i.e. it is NOT one of the fixed field markers. False for a real
    field id. Pure: the live HUD colors ghosts with this exact boundary.

    field_ids defaults to the five fixed field markers {11,45,51,67,101}."""
    allow = hs._FIELD_ARUCO_IDS if field_ids is None else frozenset(field_ids)
    return int(marker_id) not in allow


def update_votes(counter: Counter, ids: Optional[Sequence[int]]) -> Counter:
    """Add one per-frame vote for each decoded id into `counter` (mutated AND
    returned). `ids` may be None / empty (a frame with no decode) — a no-op then.
    This is the running per-id Counter the dedup/vote HUD shows; the dominant id
    is the most-voted (see dominant_id)."""
    if ids:
        for i in ids:
            counter[int(i)] += 1
    return counter


def dominant_id(counter: Counter) -> Optional[int]:
    """The id with the MOST frame-votes (the deduped 'real' marker), or None if
    no votes yet. Tie-break is DETERMINISTIC: highest vote count wins, and on an
    EXACT tie the SMALLEST id wins (so the operator sees a stable choice, not a
    flicker between equally-voted ids)."""
    if not counter:
        return None
    # max vote count, then smallest id among those tied at that count.
    best = max(counter.values())
    return min(i for i, n in counter.items() if n == best)


def vote_summary(counter: Counter,
                 field_ids: Sequence[int] = None) -> Dict[str, object]:
    """Pure summary of the running votes for the HUD / a report: dominant id,
    sorted (id, votes) pairs, and the field-vs-ghost split. No cv2 / numpy."""
    allow = hs._FIELD_ARUCO_IDS if field_ids is None else frozenset(field_ids)
    pairs = sorted(counter.items(), key=lambda kv: (-kv[1], kv[0]))
    field = sorted(i for i in counter if int(i) in allow)
    ghosts = sorted(i for i in counter if int(i) not in allow)
    return {
        "dominant": dominant_id(counter),
        "pairs": pairs,
        "field_ids": field,
        "ghost_ids": ghosts,
        "total_votes": sum(counter.values()),
    }


# ============================================================
# PURE drawing logic (cv2 + numpy, but headless — RETURN an annotated ndarray,
# never opens a window). Tests call these against synthetic frames.
# ============================================================
def annotate_aruco(img, corners, ids, field_ids: Sequence[int] = None):
    """Draw ArUco detections onto a COPY of `img` and return it. Field-valid ids
    use cv2.aruco.drawDetectedMarkers + a green id label; ghost ids (outside the
    allowlist) get a WARNING-orange label + outline so the operator sees the
    mis-decode. `corners`/`ids` are exactly what ArucoDetector.detectMarkers
    returns (ids is an Nx1 ndarray or None). Returns the input copy unchanged
    when nothing decoded."""
    import cv2
    import numpy as np

    out = img.copy()
    if ids is None or len(ids) == 0 or corners is None or len(corners) == 0:
        return out
    allow = hs._FIELD_ARUCO_IDS if field_ids is None else frozenset(field_ids)
    flat = [int(x) for x in np.asarray(ids).flatten()]

    # Let cv2 draw the marker borders (green) for the whole set first.
    try:
        cv2.aruco.drawDetectedMarkers(out, corners, np.asarray(ids))
    except cv2.error:
        # Never let a draw quirk crash the live loop; the labels below still run.
        pass

    for marker_id, corner in zip(flat, corners):
        ghost = marker_id not in allow
        color = _COLOR_GHOST if ghost else _COLOR_FIELD
        pts = np.asarray(corner).reshape(-1, 2)
        cx = int(pts[:, 0].mean())
        cy = int(pts[:, 1].mean())
        if ghost:
            # Re-outline ghosts in the warning color so they stand out from the
            # green field markers cv2 just drew.
            cv2.polylines(out, [pts.astype(np.int32)], True, color, 2)
        label = f"id {marker_id}{'  GHOST' if ghost else ''}"
        _put_label(out, label, (cx - 20, cy), color)
    return out


def annotate_yolo(img, boxes, conf_bright: float = _YOLO_CONF_BRIGHT):
    """Draw EVERY YOLO box onto a COPY of `img` and return it. NO edge rejection
    / box post-processing (that concept was retired from this repo) — every box
    the model returned is drawn GREEN with its class + confidence. Boxes below
    `conf_bright` get a cosmetic teal tint (still drawn, never rejected).

    `boxes` is a list of (class_name, confidence, (x1,y1,x2,y2)) tuples — the pure
    contract the window loop adapts the ultralytics Results into, so this draw fn
    needs no ultralytics."""
    import cv2

    out = img.copy()
    if not boxes:
        return out
    for class_name, conf, xyxy in boxes:
        x1, y1, x2, y2 = (int(round(v)) for v in xyxy)
        color = _COLOR_YOLO if conf >= conf_bright else _COLOR_YOLO_LOW
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        _put_label(out, f"{class_name} {conf:.2f}", (x1, max(0, y1 - 6)), color)
    return out


def draw_hud(img, stats: dict, show_votes: bool = True):
    """Draw the HUD overlay onto a COPY of `img` and return it. `stats` keys
    (all optional, missing -> shown as 'n/a'):
      fps, battery_pct, yaw_deg, frames, aruco_on, yolo_on, channel_order,
      votes (a Counter or dict id->votes), field_ids.
    When show_votes, the per-id ArUco vote counts + the DOMINANT id (the dedup
    indicator) are drawn; ghost ids are tinted warning-orange so a mis-decode is
    visible at a glance."""
    import cv2

    out = img.copy()
    h, w = out.shape[:2]
    lines = _hud_top_lines(stats)
    _draw_text_block(out, lines, origin=(8, 8))

    if show_votes:
        votes = stats.get("votes")
        counter = votes if isinstance(votes, Counter) else Counter(votes or {})
        field_ids = stats.get("field_ids")
        summary = vote_summary(counter, field_ids)
        allow = (hs._FIELD_ARUCO_IDS if field_ids is None
                 else frozenset(field_ids))
        vote_lines: List[Tuple[str, Tuple[int, int, int]]] = []
        dom = summary["dominant"]
        vote_lines.append((f"DEDUP dominant id: {dom if dom is not None else '-'}",
                           _COLOR_DOMINANT))
        if summary["pairs"]:
            for marker_id, n in summary["pairs"][:8]:
                ghost = int(marker_id) not in allow
                col = _COLOR_GHOST if ghost else _COLOR_FIELD
                tag = " GHOST" if ghost else ""
                star = " *" if marker_id == dom else ""
                vote_lines.append((f"  id {marker_id}: {n}{tag}{star}", col))
        else:
            vote_lines.append(("  (no ArUco votes yet)", _COLOR_HUD))
        # bottom-left block
        _draw_colored_block(out, vote_lines,
                            origin=(8, h - 18 * (len(vote_lines)) - 8))
    return out


def _hud_top_lines(stats: dict) -> List[str]:
    """The top-left HUD text (pure string formatting — unit-tested directly)."""
    def f(key: str, unit: str) -> str:
        v = stats.get(key)
        return "n/a" if v is None else f"{v:.1f}{unit}"

    fps = f("fps", "")
    batt = f("battery_pct", "%")
    yaw = f("yaw_deg", "deg")
    frames = stats.get("frames")
    a_on = "on" if stats.get("aruco_on", True) else "OFF"
    y_on = "on" if stats.get("yolo_on", True) else "OFF"
    ch = stats.get("channel_order", "?")
    lines = [
        f"fps {fps}   frames {frames if frames is not None else 'n/a'}",
        f"batt {batt}   yaw {yaw}   ch {ch}",
        f"ArUco [{a_on}]  YOLO [{y_on}]   (a/y/d/s/q)",
    ]
    return lines


# ============================================================
# Small drawing primitives (cv2; kept tiny + typed-catch-free — they only touch
# numpy/cv2 array ops which raise cv2.error, handled by callers' loops).
# ============================================================
def _put_label(img, text: str, org: Tuple[int, int],
               color: Tuple[int, int, int]) -> None:
    import cv2
    x, y = int(org[0]), int(org[1])
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(text, font, 0.5, 1)
    cv2.rectangle(img, (x - 1, y - th - 4), (x + tw + 2, y + 3),
                  _COLOR_HUD_BG, -1)
    cv2.putText(img, text, (x, y), font, 0.5, color, 1, cv2.LINE_AA)


def _draw_text_block(img, lines: Sequence[str],
                     origin: Tuple[int, int]) -> None:
    import cv2
    x, y0 = origin
    font = cv2.FONT_HERSHEY_SIMPLEX
    line_h = 18
    # one translucent-ish backing rectangle for the block
    width = max((cv2.getTextSize(s, font, 0.5, 1)[0][0] for s in lines),
                default=0)
    cv2.rectangle(img, (x - 4, y0 - 2),
                  (x + width + 6, y0 + line_h * len(lines) + 2),
                  _COLOR_HUD_BG, -1)
    for i, s in enumerate(lines):
        y = y0 + line_h * (i + 1) - 4
        cv2.putText(img, s, (x, y), font, 0.5, _COLOR_HUD, 1, cv2.LINE_AA)


def _draw_colored_block(img, lines: Sequence[Tuple[str, Tuple[int, int, int]]],
                        origin: Tuple[int, int]) -> None:
    import cv2
    x, y0 = origin
    y0 = max(0, int(y0))
    font = cv2.FONT_HERSHEY_SIMPLEX
    line_h = 18
    width = max((cv2.getTextSize(s, font, 0.5, 1)[0][0] for s, _ in lines),
                default=0)
    cv2.rectangle(img, (x - 4, y0 - 2),
                  (x + width + 6, y0 + line_h * len(lines) + 2),
                  _COLOR_HUD_BG, -1)
    for i, (s, col) in enumerate(lines):
        y = y0 + line_h * (i + 1) - 4
        cv2.putText(img, s, (x, y), font, 0.5, col, 1, cv2.LINE_AA)


# ============================================================
# YOLO adapter — turn an ultralytics Results into the pure box contract
# ============================================================
def yolo_boxes_from_result(result, names) -> List[Tuple[str, float,
                                                         Tuple[float, float,
                                                               float, float]]]:
    """Adapt ONE ultralytics Results object into the pure (class, conf, xyxy)
    contract annotate_yolo consumes. EVERY box is kept — no edge rejection / box
    post-processing (retired from this repo). Returns [] for an empty result."""
    out: List[Tuple[str, float, Tuple[float, float, float, float]]] = []
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return out
    for b in boxes:
        cls_name = names[int(b.cls[0])]
        conf = float(b.conf[0])
        x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
        out.append((cls_name, conf, (x1, y1, x2, y2)))
    return out


# ============================================================
# Frame source wiring (mirrors hula_smoke --fake exactly; real path documented)
# ============================================================
class _Viewer:
    """Owns one drone's adapter + video source + the running ArUco vote Counter.
    NO flight: connect / read / disconnect only. Teardown NEVER raises (logs)."""

    def __init__(self, log: _Log, drone_id: str):
        self.log = log
        self.drone_id = drone_id
        self.adapter = None
        self.source = None
        self.detector = None          # the single locked ArUco detector
        self.aruco_dict = "DICT_7X7_1000"
        self.votes: Counter = Counter()
        self.model = None             # ultralytics YOLO (or None)
        self.yolo_names = {}
        self.channel_order = "rgb"

    # ---- build the detector (reusing hula_smoke's tuned build) ----
    def build_aruco(self, dict_name: str, all_dicts: bool) -> None:
        only = None if all_dicts else dict_name
        detectors = hs._build_aruco_detectors(self.log, only=only)
        if not detectors:
            self.log.warn("no ArUco detector built — ArUco overlay disabled")
            self.detector = None
            return
        # Lock to ONE detector for the live overlay (the field path). With
        # --all-dicts we still pick the requested dict if present, else the first.
        self.aruco_dict = dict_name if dict_name in detectors \
            else next(iter(detectors))
        self.detector = detectors[self.aruco_dict]
        if all_dicts:
            self.log.line(f"ArUco: built {len(detectors)} dicts; overlay uses "
                          f"{self.aruco_dict} (the live window draws one dict)")
        else:
            self.log.line(f"ArUco: LOCKED to {self.aruco_dict}")

    def build_yolo(self, weights: Optional[str], no_yolo: bool) -> None:
        if no_yolo:
            self.log.line("YOLO disabled (--no-yolo)")
            return
        if not weights:
            self.log.warn("no local YOLO weights found — YOLO overlay disabled "
                          "(pass --weights or drop a .pt in models\\ / repo root)")
            return
        try:
            from ultralytics import YOLO
            self.model = YOLO(weights)
            self.yolo_names = self.model.names
            self.log.line(f"YOLO loaded ({len(self.yolo_names)} classes) from "
                          f"{weights}")
        except (OSError, RuntimeError, ValueError, ImportError) as e:
            self.log.error(f"could not load YOLO weights {weights!r} "
                           f"({type(e).__name__}: {e}) — YOLO overlay disabled; "
                           f"check the .pt path / that ultralytics is installed")
            self.model = None

    # ---- detect on a frame -> (corners, ids, yolo_boxes) ----
    def detect(self, image, run_aruco: bool, run_yolo: bool,
               yolo_conf: float):
        import cv2
        corners, ids = None, None
        if run_aruco and self.detector is not None:
            try:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = self.detector.detectMarkers(gray)
            except cv2.error as e:
                self.log.warn(f"ArUco detect failed ({type(e).__name__}: {e})")
                corners, ids = None, None
            else:
                if ids is not None and len(ids):
                    import numpy as np
                    update_votes(self.votes,
                                 [int(x) for x in np.asarray(ids).flatten()])
        boxes: List[Tuple[str, float, Tuple[float, float, float, float]]] = []
        if run_yolo and self.model is not None:
            try:
                results = self.model(image, verbose=False, conf=yolo_conf)
            except (OSError, RuntimeError, ValueError) as e:
                self.log.warn(f"YOLO infer failed ({type(e).__name__}: {e})")
                results = []
            for r in results:
                boxes.extend(yolo_boxes_from_result(r, self.yolo_names))
        return corners, ids, boxes

    def teardown(self) -> None:
        """Never raises — stop the stream + disconnect, logging any failure."""
        src = self.source
        if src is not None:
            try:
                src.stop()
            except (OSError, RuntimeError, ValueError) as e:
                self.log.warn(f"video stop failed ({type(e).__name__}: {e})")
            except Exception:  # never-raise teardown — see whitelist note
                self.log.exc("video stop")
        adapter = self.adapter
        if adapter is not None:
            try:
                import asyncio
                asyncio.run(adapter.disconnect())
            except Exception:  # never-raise teardown — see whitelist note
                self.log.exc("adapter disconnect")


def _annotate_frame(viewer: "_Viewer", image, corners, ids, boxes, stats,
                    show_aruco: bool, show_yolo: bool, show_votes: bool,
                    yolo_conf_bright: float):
    """Compose the full annotated frame (ArUco -> YOLO -> HUD) — pure draw
    pipeline, returns a new ndarray. Used by BOTH the window loop and the
    headless path so the e2e path exercises the exact draw stack."""
    annotated = image
    if show_aruco:
        annotated = annotate_aruco(annotated, corners, ids)
    if show_yolo:
        annotated = annotate_yolo(annotated, boxes, conf_bright=yolo_conf_bright)
    annotated = draw_hud(annotated, stats, show_votes=show_votes)
    return annotated


def _save_snapshot(log: _Log, outdir: Path, drone_id: str, idx: int,
                   raw_image, annotated_image) -> None:
    """Save BOTH the raw and the annotated frame (the `s` hotkey). Never raises
    — a failed write logs and the loop continues."""
    import cv2
    try:
        outdir.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%H%M%S")
        raw_p = outdir / f"{drone_id}_snap_{idx:03d}_{ts}_raw.jpg"
        ann_p = outdir / f"{drone_id}_snap_{idx:03d}_{ts}_annot.jpg"
        cv2.imwrite(str(raw_p), raw_image)
        cv2.imwrite(str(ann_p), annotated_image)
        log.line(f"snapshot saved: {raw_p.name} + {ann_p.name}")
    except (OSError, cv2.error) as e:
        log.warn(f"snapshot save failed ({type(e).__name__}: {e})")


# ============================================================
# Source construction (fake mirrors hula_smoke; real uses the same seams)
# ============================================================
def _build_viewer(log: _Log, args, weights: Optional[str]) -> Optional[_Viewer]:
    """Connect (no flight) and start the video stream, returning a ready _Viewer
    or None on failure (logged). --fake injects a FakeDroneAPI + synthetic-marker
    NumpyFakeStream (no SDK, no hardware); the real path uses PyhulaxAdapter +
    PyhulaxVideoSource over the same seams the mission flies."""
    import asyncio
    from finals.flight.pyhulax_adapter import PyhulaxAdapter, FakeDroneAPI
    from finals.vision.pyhulax_video import PyhulaxVideoSource

    drone_id = str(args.plane_id) if args.plane_id is not None else (
        args.ip if args.ip else "fakeA")
    viewer = _Viewer(log, drone_id)

    if args.fake:
        log.line("--fake: synthetic FakeDroneAPI + DICT_7X7_1000 marker frame "
                 "(no SDK, no hardware)")
        marker_id = args.fake_marker_id
        fake = FakeDroneAPI(
            battery_pct=87.0, altitude_cm=0.0, yaw_deg=10.0, is_flying=False,
            video_stream=hs._NumpyFakeStream(
                hs._synthetic_marker_frame(marker_id)))
        viewer.channel_order = "bgr"   # the synthetic frame is already BGR
        adapter = PyhulaxAdapter(drone_id, ip="127.0.0.1", api=fake)
        fake_api = fake
    else:
        viewer.channel_order = "rgb"   # real cam: .to_rgb() reverses to BGR
        ip = _resolve_ip(log, args)
        if ip is None:
            return None
        viewer.drone_id = str(args.plane_id) if args.plane_id is not None \
            else "hula"
        adapter = PyhulaxAdapter(viewer.drone_id, ip=ip)
        fake_api = None

    # ---- connect (no flight) ----
    try:
        if sys.platform == "win32":
            asyncio.set_event_loop_policy(
                asyncio.WindowsProactorEventLoopPolicy())
        asyncio.run(adapter.connect(timeout_s=args.connect_timeout))
    except Exception:  # connect surfaces an open SDK error set — log + abort
        log.exc("connect")
        return None
    viewer.adapter = adapter
    log.line(f"[{viewer.drone_id}] connected (no flight will be commanded)")

    # ---- start the video stream over the shared DroneAPI ----
    api = fake_api if fake_api is not None else getattr(adapter, "_api", None)
    if api is None:
        log.error("adapter has no _api after connect — cannot open video")
        viewer.teardown()
        return None
    try:
        src = PyhulaxVideoSource(viewer.drone_id, api,
                                 video_channel_order=viewer.channel_order)
        src.start(timeout_s=args.video_timeout)
        viewer.source = src
        log.line(f"[{viewer.drone_id}] video stream started "
                 f"(channel_order={viewer.channel_order!r})")
    except Exception:  # video bring-up open error set (P6 class) — log + abort
        log.exc("video.start")
        viewer.teardown()
        return None

    viewer.build_aruco(args.aruco_dict, args.all_dicts)
    viewer.build_yolo(weights, args.no_yolo)
    return viewer


def _resolve_ip(log: _Log, args) -> Optional[str]:
    """--ip wins; else discover and pick --plane-id (or the lowest). Bounded by
    the Dola scan window. Reuses hula_smoke.discover_all (the audited path)."""
    if args.ip:
        log.line(f"--ip given: using {args.ip} (skipping discovery)")
        return args.ip
    summary: dict = {}
    found = hs.discover_all(log, summary, args.discover_secs)
    if not found:
        log.error("no drone discovered and no --ip given — check Wi-Fi / SSID / "
                  "power / UDP 8668 firewall (Windows firewall OFF for inbound "
                  "UDP). Cannot continue.")
        return None
    if args.plane_id is not None:
        if args.plane_id not in found:
            log.error(f"plane_id {args.plane_id} not among discovered "
                      f"{sorted(found)} — cannot continue")
            return None
        return found[args.plane_id]
    pid = sorted(found)[0]
    log.line(f"no --plane-id: using lowest discovered plane_id {pid}")
    return found[pid]


# ============================================================
# Headless frame loop (CI / SSH / --fake e2e — NEVER opens a window)
# ============================================================
def run_headless(log: _Log, viewer: _Viewer, args, outdir: Path) -> int:
    """Iterate up to --frames frames (or --duration-s), running detect + the full
    annotate pipeline each frame, WITHOUT cv2.imshow. Proves the whole path
    headless-safe (SSH/CI). Returns 0 on a clean run, 1 if no frame ever arrived.
    A bounded while-loop (frame cap AND wall-clock deadline)."""
    max_frames = max(1, args.frames)
    start = time.monotonic()
    # The loop is FRAME-bounded by default; --duration-s (if > 0) caps wall
    # clock. A SAFETY deadline ALWAYS bounds the loop so a never-arriving frame
    # cannot spin forever (convention: every while-loop bounded) — generous so
    # a slow real link still fills --frames.
    safety_s = max(5.0, args.frames * 1.0)
    cap_s = args.duration_s if args.duration_s > 0 else safety_s
    deadline = start + cap_s
    seen = 0
    last_annotated = None
    last_raw = None
    while seen < max_frames and time.monotonic() < deadline:
        fs = _next_frame(log, viewer)
        if fs is None:
            time.sleep(0.01)
            continue
        seen += 1
        stats = _frame_stats(viewer, fps=_fps(seen, start),
                             frames=seen, channel_order=viewer.channel_order,
                             aruco_on=not args.no_aruco, yolo_on=not args.no_yolo)
        corners, ids, boxes = viewer.detect(
            fs.image, run_aruco=not args.no_aruco, run_yolo=not args.no_yolo,
            yolo_conf=args.yolo_conf)
        last_raw = fs.image
        last_annotated = _annotate_frame(
            viewer, fs.image, corners, ids, boxes, stats,
            show_aruco=not args.no_aruco, show_yolo=not args.no_yolo,
            show_votes=True, yolo_conf_bright=_YOLO_CONF_BRIGHT)
    summary = vote_summary(viewer.votes)
    log.line(f"[{viewer.drone_id}] headless: {seen} frames; dominant ArUco id "
             f"{summary['dominant']}; field {summary['field_ids']}; ghosts "
             f"{summary['ghost_ids']}")
    if seen and args.snapshot_on_exit and last_annotated is not None:
        _save_snapshot(log, outdir, viewer.drone_id, 0, last_raw, last_annotated)
    if seen == 0:
        log.error(f"[{viewer.drone_id}] NO frames received — camera / link / "
                  f"decode (the P6 video-bringup failure class)")
        return 1
    return 0


def _next_frame(log: _Log, viewer: _Viewer):
    """One get_frame() with a typed catch; None on no-frame / decode miss."""
    src = viewer.source
    if src is None:
        return None
    try:
        return src.get_frame()
    except (OSError, RuntimeError, ValueError) as e:
        log.warn(f"get_frame failed ({type(e).__name__}: {e})")
        return None


def _fps(seen: int, start: float) -> Optional[float]:
    elapsed = time.monotonic() - start
    return round(seen / elapsed, 1) if elapsed > 1e-6 else None


def _frame_stats(viewer: _Viewer, *, fps, frames, channel_order,
                 aruco_on, yolo_on) -> dict:
    """Assemble the HUD stats dict from the latest telemetry + vote Counter.
    Telemetry read is typed-caught — a stale link must not crash the viewer."""
    battery_pct = None
    yaw_deg = None
    adapter = viewer.adapter
    if adapter is not None:
        # telemetry() raises a typed FlightError when the link is stale/dead/
        # never-connected; the read-only HUD just shows n/a then, never crashes.
        try:
            t = adapter.telemetry()
            battery_pct = t.battery_pct
            yaw_deg = t.yaw_deg
        except FlightError as e:
            viewer.log.warn(f"telemetry read failed ({type(e).__name__}: {e})")
    return {
        "fps": fps, "frames": frames, "channel_order": channel_order,
        "battery_pct": battery_pct, "yaw_deg": yaw_deg,
        "aruco_on": aruco_on, "yolo_on": yolo_on,
        "votes": viewer.votes,
    }


# ============================================================
# The cv2.imshow window loop — SEPARATE; tests NEVER call this.
# ============================================================
def _run_window(log: _Log, viewer: _Viewer, args, outdir: Path) -> int:
    """The real-time window: grab -> detect -> annotate -> imshow, with the
    a/y/d/s/q hotkeys. NOT exercised by CI (it opens a window). Bounded by the
    --duration-s wall clock and waitKey; q quits early."""
    import cv2

    win = f"live_view [{viewer.drone_id}]  (a/y/d/s/q)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    show_aruco = not args.no_aruco
    show_yolo = not args.no_yolo
    show_votes = True
    snaps = 0
    seen = 0
    start = time.monotonic()
    deadline = (start + args.duration_s) if args.duration_s > 0 else None
    log.line("window open — a=ArUco y=YOLO d=dedup s=snapshot q=quit")
    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                log.line("duration reached — closing window")
                break
            fs = _next_frame(log, viewer)
            if fs is not None:
                seen += 1
                stats = _frame_stats(
                    viewer, fps=_fps(seen, start), frames=seen,
                    channel_order=viewer.channel_order,
                    aruco_on=show_aruco, yolo_on=show_yolo)
                corners, ids, boxes = viewer.detect(
                    fs.image, run_aruco=show_aruco, run_yolo=show_yolo,
                    yolo_conf=args.yolo_conf)
                annotated = _annotate_frame(
                    viewer, fs.image, corners, ids, boxes, stats,
                    show_aruco=show_aruco, show_yolo=show_yolo,
                    show_votes=show_votes, yolo_conf_bright=_YOLO_CONF_BRIGHT)
                try:
                    cv2.imshow(win, annotated)
                except cv2.error as e:
                    log.error(f"cv2.imshow failed ({type(e).__name__}: {e}) — no "
                              f"display? run --headless over SSH/CI")
                    break
                viewer._last_raw = fs.image
                viewer._last_annotated = annotated
            key = (cv2.waitKey(1) & 0xFF)
            if key == ord("q"):
                log.line("q pressed — quitting")
                break
            elif key == ord("a"):
                show_aruco = not show_aruco
                log.line(f"ArUco overlay {'on' if show_aruco else 'OFF'}")
            elif key == ord("y"):
                show_yolo = not show_yolo
                log.line(f"YOLO overlay {'on' if show_yolo else 'OFF'}")
            elif key == ord("d"):
                show_votes = not show_votes
                log.line(f"dedup/vote view {'on' if show_votes else 'OFF'}")
            elif key == ord("s"):
                raw = getattr(viewer, "_last_raw", None)
                ann = getattr(viewer, "_last_annotated", None)
                if raw is not None and ann is not None:
                    _save_snapshot(log, outdir, viewer.drone_id, snaps, raw, ann)
                    snaps += 1
                else:
                    log.warn("snapshot: no frame yet")
    finally:
        try:
            cv2.destroyAllWindows()
        except cv2.error as e:
            log.warn(f"destroyAllWindows failed ({type(e).__name__}: {e})")
    if seen == 0:
        log.error(f"[{viewer.drone_id}] NO frames received in the window — "
                  f"camera / link / decode (the P6 video-bringup failure class)")
        return 1
    return 0


# ============================================================
# CLI
# ============================================================
def _parse_args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="REAL-TIME CV feed visualiser for ONE HULA drone (NO FLIGHT, "
                    "read-only): connect -> camera -> ArUco/YOLO overlays in a "
                    "cv2.imshow window. --fake / --headless run with no hardware "
                    "and no window (SSH/CI).")
    # ---- target selection ----
    p.add_argument("--ip", default=None,
                   help="drone IP; skips discovery (e.g. 192.168.100.1)")
    p.add_argument("--plane-id", type=int, default=None,
                   help="view this plane_id from discovery (else lowest)")
    p.add_argument("--fake", action="store_true",
                   help="self-test with a FakeDroneAPI + synthetic DICT_7X7_1000 "
                        "marker frame (no SDK, no hardware) — run ONLINE first")
    p.add_argument("--fake-marker-id", type=int, default=11,
                   help="field-marker id baked into the --fake frame (default 11; "
                        "use a non-field id to demo the GHOST color)")
    # ---- window vs headless ----
    p.add_argument("--headless", "--no-window", dest="headless",
                   action="store_true",
                   help="iterate frames + run the full draw pipeline WITHOUT "
                        "cv2.imshow (SSH / CI / e2e) — exits after --frames or "
                        "--duration-s")
    p.add_argument("--frames", type=int, default=60,
                   help="headless: max frames to process before exit (default 60)")
    p.add_argument("--duration-s", type=float, default=0.0,
                   help="wall-clock cap in seconds (0 = unbounded window / "
                        "frame-bounded headless; default 0)")
    p.add_argument("--snapshot-on-exit", action="store_true",
                   help="headless: save the last raw+annotated frame on exit")
    # ---- ArUco ----
    p.add_argument("--aruco-dict", default="DICT_7X7_1000",
                   choices=list(hs._ARUCO_DICTS),
                   help="LOCK ArUco decode to ONE dict (default DICT_7X7_1000, "
                        "the field dict)")
    p.add_argument("--all-dicts", action="store_true",
                   help="build all candidate dicts (the overlay still draws ONE) "
                        "— for the cross-dict double-decode demo")
    p.add_argument("--no-aruco", action="store_true",
                   help="start with the ArUco overlay off (toggle with 'a')")
    # ---- YOLO (NO edge-margin / box post-processing — retired from this repo) --
    p.add_argument("--yolo-conf", type=float, default=0.25,
                   help="YOLO confidence threshold (default 0.25)")
    p.add_argument("--no-yolo", action="store_true", help="disable the YOLO overlay")
    p.add_argument("--weights", default=None,
                   help="YOLO .pt path (default: auto-detect a local one)")
    # ---- timeouts / output ----
    p.add_argument("--discover-secs", type=float, default=15.0,
                   help="Dola discovery listen window (default 15)")
    p.add_argument("--connect-timeout", type=float, default=15.0,
                   help="connect timeout seconds (default 15)")
    p.add_argument("--video-timeout", type=float, default=15.0,
                   help="first-frame timeout seconds (default 15)")
    p.add_argument("--out", default=None,
                   help="snapshot/log dir (default runs\\live_view_<timestamp>)")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)

    ts = time.strftime("%Y%m%dT%H%M%S")
    outdir = (Path(args.out) if args.out
              else _REPO_ROOT / "runs" / f"live_view_{ts}")
    try:
        outdir.mkdir(parents=True, exist_ok=True)
    except OSError as e:
        print(f"ERROR: cannot create out dir {outdir} ({e})", file=sys.stderr)
        return 1
    log = _Log(outdir / "live_view.log")
    log.line(f"live_view  ({'FAKE' if args.fake else 'LIVE'})  out={outdir}")
    log.line("NO flight commands are issued (connect / read / disconnect only).")

    weights = args.weights or hs._find_weights()
    log.line(f"yolo weights: {weights or '(none — YOLO overlay disabled)'}")

    viewer = None
    rc = 1
    try:
        viewer = _build_viewer(log, args, weights)
        if viewer is None:
            log.error("could not bring up the drone/video — see the errors above")
            return _finish(log, 1)
        # --headless is the SSH/CI path. --fake ALSO runs headless so the e2e
        # self-test is non-interactive (never blocks on a window in CI); use a
        # real --ip (no --headless) for the live cv2.imshow window.
        if args.headless or args.fake:
            if args.fake and not args.headless:
                log.line("--fake: running headless (no window) so the self-test "
                         "is non-interactive; use a real --ip for the live window")
            rc = run_headless(log, viewer, args, outdir)
        else:
            rc = _run_window(log, viewer, args, outdir)
    except KeyboardInterrupt:
        log.warn("interrupted by operator (Ctrl-C)")
        rc = 1
    except Exception:  # top-level loop must log loudly, never crash silent
        log.exc("main")
        rc = 1
    finally:
        if viewer is not None:
            viewer.teardown()
    return _finish(log, rc)


def _finish(log: _Log, rc: int) -> int:
    log.line(f"done (warnings={log.warnings} errors={log.errors}) rc={rc}")
    log.close()
    # Non-zero only on a hard failure (rc) OR a logged error — fail-loud.
    return rc if rc != 0 else (1 if log.errors else 0)


if __name__ == "__main__":
    raise SystemExit(main())
