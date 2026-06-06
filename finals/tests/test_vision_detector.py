"""finals.vision.detector — the vendored pool's three bug-fix pins +
CannedDetector parity. Runs WITHOUT cv2/numpy/ultralytics: images are
sentinel objects (the pool never introspects them with save/display off) and
inference is an injected callable — exactly the seam the module ships."""
from __future__ import annotations

import json
import os
import threading
import time

import pytest

from finals.config import DetectorConfig
from finals.errors import ConfigError
from finals.vision.detector import (CannedDetector, DetectorPool,
                                    make_ultralytics_detector)

# Detections fixture in the root Detector.py contract shape.
DET = {"bbox": [1.0, 2.0, 3.0, 4.0], "confidence": 0.9,
       "class_id": 0, "class_name": "car"}


class FakeClock:
    """Deterministic monotonic source — tests set .t explicitly."""

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


def wait_until(pred, timeout_s: float = 5.0, what: str = "condition") -> None:
    """Bounded poll (convention 3) — worker threads need real time."""
    deadline = time.monotonic() + timeout_s
    while not pred():
        if time.monotonic() > deadline:
            pytest.fail(f"timed out after {timeout_s} s waiting for {what}")
        time.sleep(0.01)


class CallbackSpy:
    """Thread-safe callback recorder."""

    def __init__(self, raise_on: int = 0):
        self.calls = []
        self.threads = []
        self._lock = threading.Lock()
        self._raise_on = raise_on            # 1-based call index; 0 = never
        self.fired = threading.Event()

    def __call__(self, detections, annotated, context):
        with self._lock:
            self.calls.append((detections, annotated, context))
            self.threads.append(threading.get_ident())
            n = len(self.calls)
        self.fired.set()
        if n == self._raise_on:
            raise RuntimeError("callback bug (scripted)")


@pytest.fixture
def pools():
    """Every pool built in a test is stopped at teardown — a leaked
    non-daemon worker thread hangs pytest on Windows."""
    built = []
    yield built.append
    for p in built:
        p.stop()


# ============================================================
# Bug pin 1: a raising model must NOT kill the worker
# ============================================================
def test_worker_survives_raising_infer(pools, capsys):
    calls = []

    def infer(image):
        calls.append(image)
        if len(calls) == 1:
            raise RuntimeError("model exploded (scripted)")
        return [dict(DET)], None

    spy = CallbackSpy()
    pool = DetectorPool(infer, spy)
    pools(pool)
    pool.submit_image({"i": 0})
    pool.submit_image({"i": 1})
    assert spy.fired.wait(5.0), (
        "callback never fired for the SECOND frame — the worker died on the "
        "first frame's exception (the root Detector.py finally/del NameError "
        "bug class)")
    assert pool.errors_total == 1
    assert pool.healthy, "worker thread died instead of surviving the error"
    assert spy.calls[0][0] == [DET]
    err = capsys.readouterr().err
    assert "model exploded" in err and "Traceback" in err, (
        "the inference error must be LOUD (full traceback), never silent")


def test_callback_exception_counted_not_fatal(pools):
    def infer(image):
        return [dict(DET)], None

    spy = CallbackSpy(raise_on=1)            # first callback raises
    pool = DetectorPool(infer, spy)
    pools(pool)
    pool.submit_image(object())
    pool.submit_image(object())
    wait_until(lambda: len(spy.calls) == 2, what="two callback invocations")
    assert pool.callback_errors_total == 1
    assert pool.healthy
    assert pool.errors_total == 0            # the model itself never failed


# ============================================================
# Bug pin 2: no silent weights fallback
# ============================================================
def test_make_ultralytics_requires_weights():
    """The empty-weights gate fires BEFORE the lazy ultralytics import —
    this test passes on a machine where ultralytics is not installed."""
    with pytest.raises(ConfigError, match="weights"):
        make_ultralytics_detector(DetectorConfig(backend="ultralytics"),
                                  callback=None)


# ============================================================
# Bug pin 3: bounded queue, drop-OLDEST, loud counter
# ============================================================
def test_bounded_queue_drops_oldest(pools, capsys):
    gate = threading.Event()
    processed = []

    def infer(image):
        processed.append(image["i"])
        gate.wait(5.0)                       # first frame wedges the worker
        return [], None

    pool = DetectorPool(infer, None, queue_maxlen=4)
    pools(pool)
    pool.submit_image({"i": 0})              # worker pops + blocks on this
    wait_until(lambda: processed == [0], what="worker to pick up frame 0")
    for i in range(1, 8):                    # 7 into a 4-slot queue -> 3 drops
        pool.submit_image({"i": i})
    assert pool.dropped_total == 3
    gate.set()
    wait_until(lambda: pool.processed_total == 5,
               what="the 4 surviving frames after release")
    assert processed == [0, 4, 5, 6, 7], (
        f"drop-OLDEST must keep the freshest frames; processed {processed}")
    assert "dropped the OLDEST" in capsys.readouterr().err


def test_drop_logging_rate_limited(pools, capsys):
    gate = threading.Event()
    clock = FakeClock(100.0)

    def infer(image):
        gate.wait(5.0)
        return [], None

    pool = DetectorPool(infer, None, queue_maxlen=1, clock=clock)
    pools(pool)
    pool.submit_image(object())              # worker takes it and blocks
    wait_until(lambda: len(pool._dq) == 0, what="worker to drain slot 0")
    for _ in range(4):                       # fills slot, then 3 quick drops
        pool.submit_image(object())
    clock.t += 6.0                           # past the 5 s warn period
    pool.submit_image(object())              # 4th drop -> second warning
    gate.set()
    assert pool.dropped_total == 4
    err = capsys.readouterr().err
    assert err.count("dropped the OLDEST") == 2, (
        "drop warnings must be rate-limited: first drop + one per period, "
        f"not one per drop — got:\n{err}")
    assert "4 dropped so far" in err         # running total in the 2nd warn


# ============================================================
# Root-contract parity
# ============================================================
def test_callback_only_when_detections(pools):
    """Pins the root semantics perception relies on: a processed frame with
    zero detections produces NO callback — silence != 'nothing found'."""
    spy = CallbackSpy()
    pool = DetectorPool(lambda image: ([], None), spy)
    pools(pool)
    pool.submit_image(object())
    wait_until(lambda: pool.processed_total == 1, what="frame processed")
    assert spy.calls == []


def test_submit_after_stop_refused(pools):
    pool = DetectorPool(lambda image: ([], None), None)
    pools(pool)
    pool.stop()
    with pytest.raises(RuntimeError, match="stop"):
        pool.submit_image(object())


def test_stop_joins_all_threads_idempotent():
    pool = DetectorPool(lambda image: ([], None), None, num_workers=3)
    pool.submit_image(object())
    pool.stop()
    pool.stop()                              # idempotent
    assert not pool.healthy
    assert all(not t.is_alive() for t in pool._workers)


def test_constructor_rejects_bad_args():
    for kwargs in ({"num_workers": 0}, {"num_workers": True},
                   {"queue_maxlen": 0}):
        with pytest.raises(ValueError):
            DetectorPool(lambda image: ([], None), None, **kwargs)
    with pytest.raises(ValueError, match="infer"):
        DetectorPool(None, None)
    with pytest.raises(ValueError, match="callback"):
        DetectorPool(lambda image: ([], None), "not callable")


# ============================================================
# CannedDetector
# ============================================================
def test_canned_fires_on_nth_submit_only(pools):
    spy = CallbackSpy()
    pool = CannedDetector([{"after_n_submits": 2, "detections": [dict(DET)]}],
                          spy)
    pools(pool)
    pool.submit_image(object())
    wait_until(lambda: pool.processed_total == 1, what="first frame")
    assert spy.calls == [], "fired before the scripted threshold"
    pool.submit_image(object())
    assert spy.fired.wait(5.0), "never fired at the scripted threshold"
    assert len(spy.calls) == 1
    pool.submit_image(object())              # past the threshold: silent again
    wait_until(lambda: pool.processed_total == 3, what="third frame")
    assert len(spy.calls) == 1


def test_canned_callback_on_worker_thread_with_contract_keys(pools):
    spy = CallbackSpy()
    pool = CannedDetector([{"after_n_submits": 1, "detections": [dict(DET)]}],
                          spy)
    pools(pool)
    pool.submit_image(object(), context={"drone_id": "alpha"})
    assert spy.fired.wait(5.0)
    assert spy.threads[0] != threading.get_ident(), (
        "the canned callback must fire on a WORKER thread like the real "
        "detector, so threading bugs surface in tests")
    detections, annotated, context = spy.calls[0]
    assert sorted(detections[0]) == sorted(
        ["bbox", "confidence", "class_id", "class_name"])
    assert annotated is None
    assert context["drone_id"] == "alpha"


def test_canned_script_from_file(pools, tmp_path):
    path = tmp_path / "script.json"
    path.write_text(json.dumps(
        [{"after_n_submits": 1, "detections": [DET]}]), encoding="utf-8")
    spy = CallbackSpy()
    pool = CannedDetector(str(path), spy)
    pools(pool)
    pool.submit_image(object())
    assert spy.fired.wait(5.0)


def test_committed_canned_fixture_is_valid(pools, repo_root):
    fixture = os.path.join(repo_root, "finals", "tests", "fixtures",
                           "canned_script.json")
    pool = CannedDetector(fixture, None)
    pools(pool)                              # constructing it validates it


def _det(**overrides):
    return {**DET, **overrides}


@pytest.mark.parametrize("bad, match", [
    ("missing.json", "cannot read"),
    ({"not": "a list"}, "LIST"),
    ([{"after_n_submits": 1}], "exactly"),
    ([{"after_n_submits": 0, "detections": [DET]}], "after_n_submits"),
    ([{"after_n_submits": 1, "detections": []}], "NON-EMPTY"),
    ([{"after_n_submits": 1, "detections": [{"bbox": [0, 0, 1, 1]}]}],
     "contract keys"),
    # VALUES too, not just keys (a bad value would otherwise build a
    # Sighting the CSV codec refuses MID-RUN — killing CSV recording):
    ([{"after_n_submits": 1, "detections": [_det(bbox=[1, 2, 3])]}],
     "bbox"),
    ([{"after_n_submits": 1,
       "detections": [_det(bbox=[1, 2, 3, float("nan")])]}], "bbox"),
    ([{"after_n_submits": 1,
       "detections": [_det(confidence=float("nan"))]}], "confidence"),
    ([{"after_n_submits": 1, "detections": [_det(class_id="zero")]}],
     "class_id"),
    ([{"after_n_submits": 1, "detections": [_det(class_name="a\nb")]}],
     "class_name"),
    ([{"after_n_submits": 1, "detections": [_det(class_name="")]}],
     "class_name"),
])
def test_canned_script_validation_loud(bad, match):
    with pytest.raises(ConfigError, match=match):
        CannedDetector(bad, None)


def test_stop_abandons_pending_frames(pools):
    """The stop() contract: pending frames are ABANDONED, not drained — a
    slow model must not stretch shutdown by maxlen x inference time, and a
    late callback must never hit an already-closed SightingLog."""
    gate = threading.Event()
    processed = []

    def infer(image):
        processed.append(image["i"])
        gate.wait(5.0)                       # frame 0 wedges the worker
        return [], None

    pool = DetectorPool(infer, None, queue_maxlen=4)
    pools(pool)
    pool.submit_image({"i": 0})
    wait_until(lambda: processed == [0], what="worker to pick up frame 0")
    for i in range(1, 4):
        pool.submit_image({"i": i})          # pending in the queue
    stopper = threading.Thread(target=pool.stop)
    stopper.start()
    wait_until(lambda: pool._stop_event.is_set(), what="stop signal")
    gate.set()                               # frame 0's inference finishes
    stopper.join(10.0)
    assert not stopper.is_alive(), "stop() must return once workers join"
    assert processed == [0], (
        f"pending frames must be ABANDONED on stop, not drained — "
        f"processed {processed}")
    assert pool.processed_total == 1
