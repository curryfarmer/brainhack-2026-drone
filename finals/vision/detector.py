"""Laptop YOLO detection — vendored FIXED Detector worker pool + CannedDetector.

OPTIONAL FALLBACK (user-confirmed 2026-06-06): the convoy robots carry
markers, so finals/vision/aruco.py is the primary detector and YOLO is OFF by
default in every shipped config (detector.backend "none"). Enable it (one
config edit: backend "ultralytics" + weights) only if the briefing scores
spotting robots whose marker isn't readable.

Derives from: root Detector.py (audited line-by-line, vendored — the root
file stays untouched for the qualifier stack). Bugs fixed in adaptation:
1. THE WORKER-KILLER (root Detector.py:143-150): the worker's
   `finally: del results ...` raises NameError when self.model() raises
   BEFORE `results` binds — the exception propagates out of the loop and the
   worker thread dies SILENTLY: detection stops forever with no crash. Fixed
   by removing the del-block entirely (CPython frees locals on scope exit)
   and guarding inference per item: full traceback + errors counter, then
   the NEXT item is still processed — loud survival, not silent death.
2. THE SILENT COCO FALLBACK (root Detector.py:28-29): model_path=None fell
   back to "yolov8n.pt" without a word — the qualifier known-issue #9 trap.
   Here there is NO fallback path at all: the inference callable is injected
   (make_ultralytics_detector requires validated weights from finals.config).
3. THE UNBOUNDED QUEUE (root Detector.py:33/65): queue.Queue() grows without
   bound when inference is slower than submission — stale frames pile up and
   latency runs away silently. Fixed: bounded deque(maxlen), DROP-OLDEST
   (freshest frames win), with a LOUD rate-limited drop counter exposed as
   `dropped_total` (perception's adaptive shed watches it).

Kept IDENTICAL to the root contract (perception relies on both):
- callback(detections, annotated_image, context); each detection dict has
  exactly bbox / confidence / class_id / class_name (root :112-117);
- the callback fires ONLY when there ARE detections (root :120/:135) —
  callback silence does NOT mean "frame processed, nothing found";
- context dicts may be mutated by the pool (`saved_path`, root :125) —
  callers pass a FRESH dict per submit.

`except Exception` NOTE (whitelisted in tests/test_conventions.py — a
DELIBERATE, reviewed widening): the model may raise anything (ultralytics /
torch / cv2 / CUDA errors are an open set), and the per-item + per-callback
guards exist precisely so a worker thread can never die silently again
(threading.excepthook is NOT covered by events.install_crash_hooks). Every
catch here logs the full traceback to stderr and increments a counter —
none is silent.

No SDK imports at module level despite the SDK_ALLOWED entry: the pool takes
an injected `infer` callable, so CannedDetector and the worker-survival tests
run on a machine with neither ultralytics nor cv2 installed. cv2/ultralytics
are imported lazily inside the code paths that need them.

One shared pool instance serves all drones; frames carry
context={"drone_id", "ts", "yaw", "alt", ...}.

Session: S7 (implemented).
"""
from __future__ import annotations

import json
import math
import os
import queue
import sys
import threading
import time
import traceback
from collections import deque
from typing import (TYPE_CHECKING, Any, Callable, Deque, Dict, List, Optional,
                    Tuple, Union)

from finals.errors import ConfigError

if TYPE_CHECKING:  # heavy types for annotations only; module stays stdlib
    import numpy as np
    from finals.config import DetectorConfig

#: image -> (detections, annotated_image_or_None). The detection dicts carry
#: the root Detector.py contract keys: bbox, confidence, class_id, class_name.
Infer = Callable[[Any], Tuple[List[Dict[str, Any]], Optional[Any]]]
DetectionCallback = Callable[[List[Dict[str, Any]], Optional[Any],
                              Dict[str, Any]], None]

#: Seconds between repeated drop warnings (the first drop always warns).
_DROP_WARN_PERIOD_S = 5.0


class DetectorPool:
    """Threaded inference worker pool over a BOUNDED drop-oldest queue.

    Threading model: submit_image() may be called from any thread (the
    perception loops); workers are dedicated non-daemon threads (joined by
    stop() — a leaked worker hangs pytest on Windows); the callback fires ON
    A WORKER THREAD and must only touch thread-safe sinks (SightingBus /
    SightingLog — never agents, events, or asyncio objects).
    """

    def __init__(self, infer: Infer,
                 callback: Optional[DetectionCallback], *,
                 num_workers: int = 1,
                 queue_maxlen: int = 4,
                 save_dir: Optional[str] = None,
                 enable_display: bool = False,
                 name: str = "detector",
                 clock: Callable[[], float] = time.monotonic):
        if not callable(infer):
            raise ValueError(
                f"DetectorPool({name!r}): infer must be callable, got "
                f"{infer!r} — check the wiring (make_ultralytics_detector "
                f"builds it)")
        if callback is not None and not callable(callback):
            raise ValueError(
                f"DetectorPool({name!r}): callback must be callable or None, "
                f"got {callback!r} — check the perception wiring")
        if not isinstance(num_workers, int) or isinstance(num_workers, bool) \
                or num_workers < 1:
            raise ValueError(
                f"DetectorPool({name!r}): num_workers must be an int >= 1, "
                f"got {num_workers!r} — check detector.workers in the config")
        if not isinstance(queue_maxlen, int) or isinstance(queue_maxlen, bool) \
                or queue_maxlen < 1:
            raise ValueError(
                f"DetectorPool({name!r}): queue_maxlen must be an int >= 1, "
                f"got {queue_maxlen!r}")

        self._infer = infer
        self._callback = callback
        self._name = name
        self._clock = clock

        self._dq: "Deque[Tuple[Any, Dict[str, Any]]]" = deque(maxlen=queue_maxlen)
        self._cond = threading.Condition()
        self._stop_event = threading.Event()
        self._stopped = False

        # Counters share one lock (+= is not atomic across threads).
        self._count_lock = threading.Lock()
        self._dropped_total = 0
        self._errors_total = 0
        self._callback_errors_total = 0
        self._processed_total = 0
        self._last_drop_warn: Optional[float] = None
        self._save_counter = 0

        self._save_dir: Optional[str] = None
        if save_dir is not None:
            self._save_dir = os.path.abspath(save_dir)
            try:
                os.makedirs(self._save_dir, exist_ok=True)
            except OSError as e:
                raise ValueError(
                    f"DetectorPool({name!r}): cannot create save_dir "
                    f"{self._save_dir!r} — errno {e.errno} ({e.strerror}) — "
                    f"check the path/permissions") from e

        self._workers: List[threading.Thread] = []
        for i in range(num_workers):
            t = threading.Thread(target=self._worker,
                                 name=f"{name}-worker-{i}", daemon=False)
            t.start()
            self._workers.append(t)

        # Display path vendored from root Detector.py:73-87 (debug only):
        # maxsize=1 queue so the UI always shows the LATEST frame.
        self._display_queue: "queue.Queue[Any]" = queue.Queue(maxsize=1)
        self._display_thread: Optional[threading.Thread] = None
        if enable_display:
            self._display_thread = threading.Thread(
                target=self._display_worker, name=f"{name}-display",
                daemon=True)
            self._display_thread.start()

    # ---------------- submission ----------------
    def submit_image(self, image: Any,
                     context: Optional[Dict[str, Any]] = None) -> None:
        """Queue one frame from any thread. A full queue DROPS THE OLDEST
        frame (freshest wins) and counts it loudly — never blocks, never
        grows without bound (root bug 3)."""
        if context is None:
            context = {}
        with self._cond:
            if self._stopped:
                raise RuntimeError(
                    f"DetectorPool({self._name!r}): submit_image after "
                    f"stop() — check shutdown ordering")
            if len(self._dq) == self._dq.maxlen:
                self._note_drop_locked()       # deque(maxlen) evicts on append
            self._dq.append((image, context))
            self._cond.notify()

    def _note_drop_locked(self) -> None:
        """Called under self._cond. First drop warns immediately; afterwards
        at most one warning per _DROP_WARN_PERIOD_S with the running total —
        loud without flooding stderr at sustained overload."""
        victim_drone = (self._dq[0][1].get("drone_id", "?")
                        if self._dq else "?")
        with self._count_lock:
            self._dropped_total += 1
            total = self._dropped_total
        now = self._clock()
        if (self._last_drop_warn is not None
                and now - self._last_drop_warn < _DROP_WARN_PERIOD_S):
            return
        self._last_drop_warn = now
        print(f"[DetectorPool:{self._name}] WARNING: submit queue full "
              f"(maxlen {self._dq.maxlen}) — dropped the OLDEST frame "
              f"(drone {victim_drone}; {total} dropped so far) — inference "
              f"is slower than submission; perception sheds to a lower rate "
              f"on this signal — check model size/device",
              file=sys.stderr, flush=True)

    # ---------------- worker ----------------
    def _next_item(self) -> Optional[Tuple[Any, Dict[str, Any]]]:
        with self._cond:
            # Bounded (convention 3): exits on the stop event; the wait is
            # timeout-bounded so a missed notify can never wedge a worker.
            # The stop check comes BEFORE the queue check: pending frames
            # are ABANDONED on stop (the stop() contract) — a slow model
            # must not stretch shutdown by maxlen x inference time, and a
            # late callback must never hit an already-closed SightingLog.
            while True:
                if self._stop_event.is_set():
                    return None
                if self._dq:
                    return self._dq.popleft()
                self._cond.wait(0.5)

    def _worker(self) -> None:
        # Bounded (convention 3): _next_item returns None once the stop
        # event is set (pending frames abandoned — see its comment).
        while True:
            item = self._next_item()
            if item is None:
                return
            self._process(*item)

    def _process(self, image: Any, context: Dict[str, Any]) -> None:
        try:
            detections, annotated = self._infer(image)
        except Exception:
            # WHITELISTED blanket catch (tests/test_conventions.py): the
            # model may raise ANYTHING; root Detector.py died silently here
            # (bug 1 — the finally/del NameError). Loud survival instead:
            # full traceback, counter, and the NEXT frame still detects.
            with self._count_lock:
                self._errors_total += 1
                n = self._errors_total
            print(f"[DetectorPool:{self._name}] ERROR: inference raised on "
                  f"a frame from drone {context.get('drone_id', '?')} "
                  f"(error #{n}; worker continues — the next frame is still "
                  f"processed):\n{traceback.format_exc()}",
                  file=sys.stderr, flush=True)
            return
        with self._count_lock:
            self._processed_total += 1
        if not detections:
            # Root parity (:120): no detections -> no save, no display, NO
            # CALLBACK. Perception must not read silence as "nothing found
            # was confirmed" — only as "no detections were reported".
            return

        if self._save_dir is not None and annotated is not None:
            self._save_annotated(annotated, context)
        if self._display_thread is not None and annotated is not None:
            try:
                self._display_queue.put_nowait(annotated)
            except queue.Full:
                pass    # root parity (:131-132): skip to keep the UI live

        if self._callback is None:
            return
        try:
            self._callback(detections, annotated, context)
        except Exception:
            # WHITELISTED blanket catch: the callback is OUR code (the
            # perception sink) but a poisoned SightingLog or a bug there
            # must not kill detection for the rest of the mission. Always
            # the full traceback + counter — never silent (root :136-139
            # printed only str(e)).
            with self._count_lock:
                self._callback_errors_total += 1
                n = self._callback_errors_total
            print(f"[DetectorPool:{self._name}] ERROR: detection callback "
                  f"raised for drone {context.get('drone_id', '?')} "
                  f"(error #{n}; worker continues):\n"
                  f"{traceback.format_exc()}", file=sys.stderr, flush=True)

    def _save_annotated(self, annotated: Any,
                        context: Dict[str, Any]) -> None:
        """Root parity (:120-125): save the annotated frame, record the path
        in the (caller-owned, mutated) context. Lazy cv2 — only this branch
        needs it, and only a real model ever produces an annotated image."""
        try:
            import cv2
        except ImportError:
            print(f"[DetectorPool:{self._name}] WARNING: save_dir set but "
                  f"cv2 is not installed — annotated frames are NOT saved",
                  file=sys.stderr, flush=True)
            return
        with self._count_lock:
            self._save_counter += 1
            n = self._save_counter
        path = os.path.join(self._save_dir,
                            f"det_{n}_{int(time.time() * 1000)}.jpg")
        try:
            ok = cv2.imwrite(path, annotated)
        except cv2.error as e:
            print(f"[DetectorPool:{self._name}] WARNING: imwrite({path!r}) "
                  f"raised: {e} — annotated frame not saved",
                  file=sys.stderr, flush=True)
            return
        if ok:
            context["saved_path"] = path
        else:
            print(f"[DetectorPool:{self._name}] WARNING: imwrite({path!r}) "
                  f"returned False — annotated frame not saved; check disk "
                  f"space/extension", file=sys.stderr, flush=True)

    def _display_worker(self) -> None:
        """Vendored root display thread (:73-87) — debug only."""
        try:
            import cv2
        except ImportError:
            print(f"[DetectorPool:{self._name}] WARNING: display requested "
                  f"but cv2 is not installed — display disabled",
                  file=sys.stderr, flush=True)
            return
        window = f"finals detections [{self._name}]"
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        # Bounded (convention 3): stop event + the timeout-bounded get.
        while not self._stop_event.is_set():
            try:
                img = self._display_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                cv2.imshow(window, img)
                cv2.waitKey(1)      # mandatory for the cv2 GUI event loop
            except cv2.error as e:
                print(f"[DetectorPool:{self._name}] display error: {e} — "
                      f"display disabled", file=sys.stderr, flush=True)
                break
        try:
            cv2.destroyWindow(window)
        except cv2.error:
            pass    # window already gone — nothing to clean

    # ---------------- lifecycle / introspection ----------------
    def stop(self, timeout_s: float = 10.0) -> None:
        """Idempotent; never raises. Drains nothing — pending frames are
        abandoned (mission over); joins every worker (leaked non-daemon
        threads hang pytest/the process on Windows)."""
        with self._cond:
            if self._stopped:
                return
            self._stopped = True
            self._stop_event.set()
            self._cond.notify_all()
        deadline = self._clock() + timeout_s
        for t in self._workers:
            t.join(max(0.1, deadline - self._clock()))
            if t.is_alive():
                print(f"[DetectorPool:{self._name}] WARNING: worker "
                      f"{t.name} still alive after stop({timeout_s:.1f} s) "
                      f"— a hung model call; the thread is non-daemon and "
                      f"WILL block process exit until it returns — check "
                      f"the model/device", file=sys.stderr, flush=True)
        if self._display_thread is not None:
            self._display_thread.join(1.0)

    @property
    def dropped_total(self) -> int:
        with self._count_lock:
            return self._dropped_total

    @property
    def errors_total(self) -> int:
        with self._count_lock:
            return self._errors_total

    @property
    def callback_errors_total(self) -> int:
        with self._count_lock:
            return self._callback_errors_total

    @property
    def processed_total(self) -> int:
        with self._count_lock:
            return self._processed_total

    @property
    def healthy(self) -> bool:
        """Any worker alive. With the per-item guards a worker can only die
        at interpreter shutdown — but the flag keeps death observable."""
        return any(t.is_alive() for t in self._workers)

    def stats(self) -> dict:
        with self._count_lock:
            return {
                "name": self._name,
                "processed_total": self._processed_total,
                "dropped_total": self._dropped_total,
                "errors_total": self._errors_total,
                "callback_errors_total": self._callback_errors_total,
                "queue_len": len(self._dq),
                "healthy": self.healthy,
            }


# ============================================================
# Real backend
# ============================================================
def make_ultralytics_detector(det_cfg: "DetectorConfig",
                              callback: Optional[DetectionCallback], *,
                              save_dir: Optional[str] = None,
                              enable_display: bool = False) -> DetectorPool:
    """Build a DetectorPool over a real YOLO model. Weights come VALIDATED
    from finals.config (exists on disk, COCO placeholders rejected — the
    known-issue #9 guards); there is deliberately no fallback here (root
    bug 2). ultralytics is imported lazily so this module stays importable
    on machines without it."""
    if not det_cfg.weights:
        # Belt over the config gate: reached only by hand-built configs.
        raise ConfigError(
            "make_ultralytics_detector: detector.weights is empty — weights "
            "are REQUIRED (no silent default; root Detector.py bug 2); set "
            'detector.weights (e.g. "best.pt") in the config')
    from ultralytics import YOLO   # lazy: only the selected backend pays
    model = YOLO(det_cfg.weights).to(det_cfg.device)
    conf = det_cfg.conf

    def infer(image: "np.ndarray"):
        # Root worker math (:97-118), minus the silent-death wrapping.
        results = model(image, verbose=False, conf=conf)
        detections: List[Dict[str, Any]] = []
        annotated = None
        for result in results:
            boxes = result.boxes
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                    cls_id = int(box.cls[0].cpu().item())
                    detections.append({
                        "bbox": [x1, y1, x2, y2],
                        "confidence": float(box.conf[0].cpu().item()),
                        "class_id": cls_id,
                        "class_name": model.names[cls_id],
                    })
                annotated = result.plot()
        return detections, annotated

    return DetectorPool(infer, callback, num_workers=det_cfg.workers,
                        save_dir=save_dir, enable_display=enable_display,
                        name="ultralytics")


# ============================================================
# Canned backend (tests + detector-less rehearsals)
# ============================================================
class CannedDetector(DetectorPool):
    """Script-driven DetectorLike: same pool, same WORKER-THREAD callback
    path as the real detector, so threading bugs surface in tests.

    Script: a JSON path or list of {"after_n_submits": N, "detections":
    [...]} entries. The trigger counts SUBMISSIONS; the detections fire with
    the next frame processed at/after that count (identical to the Nth frame
    when nothing is dropped — canned runs never overload the queue). Frames
    between triggers produce NO callback (root only-when-detections parity).
    """

    _CONTRACT_KEYS = ("bbox", "confidence", "class_id", "class_name")

    def __init__(self, script: Union[str, List[dict]],
                 callback: Optional[DetectionCallback], **pool_kwargs):
        entries = self._load_script(script)
        self._by_threshold: Dict[int, List[Dict[str, Any]]] = {
            e["after_n_submits"]: e["detections"] for e in entries}
        self._submit_count = 0
        self._due: "Deque[List[Dict[str, Any]]]" = deque()
        self._script_lock = threading.Lock()
        pool_kwargs.setdefault("name", "canned")
        super().__init__(self._canned_infer, callback, **pool_kwargs)

    @classmethod
    def _load_script(cls, script: Union[str, List[dict]]) -> List[dict]:
        where = "CannedDetector script"
        if isinstance(script, str):
            where = f"CannedDetector script {script!r}"
            try:
                with open(script, "r", encoding="utf-8") as f:
                    script = json.load(f)
            except OSError as e:
                raise ConfigError(
                    f"{where}: cannot read — errno {e.errno} ({e.strerror}) "
                    f"— check detector.canned_script points at a real file"
                ) from e
            except json.JSONDecodeError as e:
                raise ConfigError(f"{where}: invalid JSON — {e}") from e
        if not isinstance(script, list):
            raise ConfigError(
                f"{where}: must be a LIST of entries, got "
                f"{type(script).__name__} — expected "
                f'[{{"after_n_submits": N, "detections": [...]}}, ...]')
        for i, entry in enumerate(script):
            at = f"{where} entry [{i}]"
            if not isinstance(entry, dict) or set(entry) != {
                    "after_n_submits", "detections"}:
                raise ConfigError(
                    f"{at}: must be exactly "
                    f'{{"after_n_submits": N, "detections": [...]}}, got '
                    f"{entry!r}")
            n = entry["after_n_submits"]
            if not isinstance(n, int) or isinstance(n, bool) or n < 1:
                raise ConfigError(
                    f"{at}: after_n_submits must be an int >= 1, got {n!r}")
            dets = entry["detections"]
            if not isinstance(dets, list) or not dets:
                raise ConfigError(
                    f"{at}: detections must be a NON-EMPTY list (an empty "
                    f"list would never fire the callback — root "
                    f"only-when-detections parity); got {dets!r}")
            for j, det in enumerate(dets):
                if not isinstance(det, dict) or sorted(det) != sorted(
                        cls._CONTRACT_KEYS):
                    raise ConfigError(
                        f"{at} detection [{j}]: must carry exactly the root "
                        f"contract keys {sorted(cls._CONTRACT_KEYS)}, got "
                        f"{sorted(det) if isinstance(det, dict) else det!r}")
                cls._check_detection_values(det, f"{at} detection [{j}]")
        return script

    @staticmethod
    def _check_detection_values(det: dict, at: str) -> None:
        """VALUES too, not just keys: a wrong-arity bbox / NaN confidence /
        newline class_name builds a Sighting the CSV codec REFUSES at append
        time (finals/sightings.py) — failing the whole run's CSV recording
        from one bad script entry. Canned scripts are the only detector
        whose values come from user JSON (which accepts literal NaN), so
        they die HERE, at load, with the entry named."""
        bbox = det["bbox"]
        if (not isinstance(bbox, (list, tuple)) or len(bbox) != 4
                or not all(isinstance(v, (int, float))
                           and not isinstance(v, bool)
                           and math.isfinite(v) for v in bbox)):
            raise ConfigError(
                f"{at}: bbox must be 4 finite numbers [x1, y1, x2, y2], got "
                f"{bbox!r}")
        conf = det["confidence"]
        if (not isinstance(conf, (int, float)) or isinstance(conf, bool)
                or not math.isfinite(conf)):
            raise ConfigError(
                f"{at}: confidence must be a finite number, got {conf!r}")
        cid = det["class_id"]
        if not isinstance(cid, int) or isinstance(cid, bool):
            raise ConfigError(f"{at}: class_id must be an int, got {cid!r}")
        name = det["class_name"]
        if (not isinstance(name, str) or not name
                or "\n" in name or "\r" in name):
            raise ConfigError(
                f"{at}: class_name must be a non-empty single-line string "
                f"(the CSV codec refuses newlines), got {name!r}")

    def submit_image(self, image: Any,
                     context: Optional[Dict[str, Any]] = None) -> None:
        with self._script_lock:
            self._submit_count += 1
            dets = self._by_threshold.get(self._submit_count)
            if dets is not None:
                self._due.append([dict(d) for d in dets])  # caller-safe copies
        super().submit_image(image, context)

    def _canned_infer(self, image: Any):
        with self._script_lock:
            if self._due:
                return self._due.popleft(), None
        return [], None
