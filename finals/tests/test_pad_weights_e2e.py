"""Real-weights pad-decode e2e (PAD-DETECT tail, task #9 sub-(d)).

Proves the user's TRAINED landing-pad YOLO model (`models/pad_v1.pt`, a
pipeline.py artifact — val mAP50 0.984 on 30 frames) decodes a real pad frame
through the PRODUCTION detector seam `make_ultralytics_detector`, emitting the
exact detection dict the perception loop turns into a Sighting(source="yolo",
class_name="landing_pad") that land_on_pad servo_on="pad" consumes
(test_land_on_pad.py covers that downstream half with synthetic sightings; the
replay canned test covers the data CONTRACT; THIS closes the loop on the real
weights actually firing on real imagery).

Skip-gated on artifacts that are gitignored DATA (the weights + the captured
frames), so the suite stays green on a bare checkout / CI where neither exists:
  * weights   <- env PAD_WEIGHTS, else models/pad_v1.pt, else models/latest_path.txt
  * frames    <- env PAD_FRAMES_DIR, else data/validation/images, else data/train/images
ultralytics + cv2 + numpy are likewise importorskip'd (lazy-import parity with
finals/vision/detector.py). When all three are present (the training machine /
onsite C2 laptop) the test runs for real and asserts a landing_pad detection.
"""
import os
import glob
import threading

import pytest

pytest.importorskip("ultralytics")
pytest.importorskip("cv2")
np = pytest.importorskip("numpy")
import cv2  # noqa: E402  (gated above)

from finals.config import DetectorConfig          # noqa: E402
from finals.vision.detector import make_ultralytics_detector  # noqa: E402


# --- repo root: this file is finals/tests/<here>, so two dirs up ------------
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# The PAD class the model was trained on (data/classes.txt, nc=1).
_PAD_CLASS = "landing_pad"


def _resolve_weights():
    """First existing of: env PAD_WEIGHTS, models/pad_v1.pt, the path recorded
    in models/latest_path.txt (the deploy pointer). None -> skip."""
    env = os.environ.get("PAD_WEIGHTS")
    if env and os.path.isfile(env):
        return env
    cand = os.path.join(_REPO, "models", "pad_v1.pt")
    if os.path.isfile(cand):
        return cand
    ptr = os.path.join(_REPO, "models", "latest_path.txt")
    if os.path.isfile(ptr):
        rel = open(ptr, encoding="utf-8").read().strip().replace("\\", os.sep)
        p = rel if os.path.isabs(rel) else os.path.join(_REPO, rel)
        if os.path.isfile(p):
            return p
    return None


def _resolve_frames(limit=12):
    """Up to `limit` real .jpg frames from env PAD_FRAMES_DIR / the val set /
    the train set. Several frames (not one) so a single hard frame the model
    happens to miss can't flake the test — we assert at least one decodes."""
    dirs = [os.environ.get("PAD_FRAMES_DIR"),
            os.path.join(_REPO, "data", "validation", "images"),
            os.path.join(_REPO, "data", "train", "images")]
    for d in dirs:
        if d and os.path.isdir(d):
            jpgs = sorted(glob.glob(os.path.join(d, "*.jpg")))
            if jpgs:
                return jpgs[:limit]
    return []


def test_trained_pad_weights_decode_a_real_frame():
    weights = _resolve_weights()
    if weights is None:
        pytest.skip("pad weights absent (gitignored artifact) — set PAD_WEIGHTS "
                    "or run pipeline.py to produce models/pad_v1.pt")
    frames = _resolve_frames()
    if not frames:
        pytest.skip("no pad frames found — set PAD_FRAMES_DIR or populate "
                    "data/validation/images")

    got = []                       # detections collected off the worker thread
    fired = threading.Event()

    def _cb(detections, _annotated, _ctx):
        got.extend(detections)
        fired.set()

    det_cfg = DetectorConfig(backend="ultralytics", weights=weights,
                             conf=0.25, device="cpu", workers=1)
    pool = make_ultralytics_detector(det_cfg, _cb)
    try:
        for i, fp in enumerate(frames):
            img = cv2.imread(fp)
            assert img is not None, f"cv2 could not read frame {fp!r}"
            pool.submit_image(img, {"drone_id": "test", "frame": i})
        # Bounded wait for the worker pool to drain (CPU YOLO ~tens of ms/frame).
        fired.wait(timeout=30.0)
    finally:
        pool.stop(timeout_s=15.0)

    pad_hits = [d for d in got if d.get("class_name") == _PAD_CLASS]
    assert pad_hits, (
        f"trained weights {os.path.basename(weights)} produced NO "
        f"{_PAD_CLASS!r} detection across {len(frames)} real frames "
        f"(got classes={sorted({d.get('class_name') for d in got})}) — the "
        f"model/seam is not decoding pads")

    # Contract the pad servo relies on: a real bbox, a sane confidence.
    h, w = cv2.imread(frames[0]).shape[:2]
    for d in pad_hits:
        x0, y0, x1, y1 = d["bbox"]
        assert 0.0 <= x0 < x1 and 0.0 <= y0 < y1, f"degenerate bbox {d['bbox']}"
        assert x1 <= w + 1 and y1 <= h + 1, f"bbox {d['bbox']} outside {w}x{h}"
        assert 0.0 < d["confidence"] <= 1.0
        assert d["class_id"] == 0          # nc=1 -> the single pad class
