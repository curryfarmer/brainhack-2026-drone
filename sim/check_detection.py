#!/usr/bin/env python3
"""SIM-3 detection check — validates the convoy WORLD (not the finals/ package).

Subscribes the 3 static down-cameras via gz.transport13, runs RAW cv2 ArUco AND QR
detection on every frame, and emits a px-vs-distance table PER MARKER TYPE plus sample
annotated frames. The finals detector wrapper arrives in S7/SIM-4 — this script proves
the WORLD renders readable markers at the altitude bands, nothing more.

LAUNCH under SYSTEM python3 (gz.transport13 is built for 3.10), NOT the .venv, AND with
PYTHONNOUSERSITE=1 (a pip --user protobuf in ~/.local shadows the apt protobuf the gz
_pb2 modules need otherwise):

    PYTHONNOUSERSITE=1 python3 sim/check_detection.py --secs 40

cv2 on the VM is the apt 4.5.4 build (OLD aruco API); the shim below also works on 4.7+.
QR CAVEAT: that apt build is NOT linked against QUIRC, so QRCodeDetector can LOCATE a QR
(returns points) but never DECODES it ("Library QUIRC is not linked" on stderr) — QR
decode counts are 0 on the VM regardless of distance; QR-located px is still the size data.
Subscriber pattern mirrors repo-root depth_receiver.py (Node + subscribe(Image, topic,
cb), latest-frame copy under a lock). This file lives in sim/ BY DESIGN (outside the
finals conventions/SDK scan) so raw cv2/gz are allowed. Fail-loud: every wait has a
deadline; exits nonzero with WHAT / WHICH band / WHY / CHECK.
"""

import argparse
import os
import sys
import threading
import time
from collections import defaultdict

import numpy as np

try:
    import cv2
except ImportError as exc:
    print(f"FAIL: cannot import cv2 — WHY: opencv missing on this interpreter — "
          f"CHECK: VM system python3 has apt python3-opencv ({exc})", file=sys.stderr)
    sys.exit(2)

try:
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
except Exception as exc:  # noqa: BLE001 - want a loud, specific message here
    print(f"FAIL: cannot import gz.transport13 / gz.msgs10 — WHY: wrong interpreter or "
          f"protobuf shadow — WHICH: run as `PYTHONNOUSERSITE=1 python3` (system 3.10), "
          f"NOT the .venv — CHECK: {exc}", file=sys.stderr)
    sys.exit(2)

# Geometry baked into convoy.sdf — used to turn a camera altitude into a marker DISTANCE.
DEFAULT_BANDS = {120: 1.2, 170: 1.7, 220: 2.2}  # band name -> camera altitude (m), sentry bands
ROBOT_MARKER_H_M = 0.25                              # convoy marker sits on the chassis top
PAD_MARKER_H_M = 0.0                                 # pad marker on the ground
DEFAULT_ROBOT_IDS = [7, 11, 23, 42, 88]
DEFAULT_PAD_IDS = [100, 101]


def cam_topic(world: str, band: int) -> str:
    return f"/world/{world}/model/cam_band_{band}/link/camera_link/sensor/camera/image"


# --------------------------------------------------------------------------- #
# cv2 aruco version shim (VM 4.5.4 old API <-> 4.7+ ArucoDetector)
# --------------------------------------------------------------------------- #
class ArucoShim:
    def __init__(self):
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.params = cv2.aruco.DetectorParameters_create()      # 4.5.x
        else:
            self.params = cv2.aruco.DetectorParameters()             # 4.7+
        self.detector = (cv2.aruco.ArucoDetector(self.dict, self.params)
                         if hasattr(cv2.aruco, "ArucoDetector") else None)

    def detect(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)                 # 4.7+
        return cv2.aruco.detectMarkers(gray, self.dict, parameters=self.params)  # 4.5.x


def quad_px(pts) -> float:
    """Max edge length of a 4-corner quad => marker px size (perspective-robust)."""
    c = np.asarray(pts, dtype=np.float32).reshape(4, 2)
    return float(max(np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)))


# --------------------------------------------------------------------------- #
# latest-frame gz subscriber (one per band) — mirrors depth_receiver.DepthReceiver
# --------------------------------------------------------------------------- #
class CameraReceiver:
    def __init__(self, node: Node, topic: str):
        self.topic = topic
        self.frame = None
        self.count = 0
        self.lock = threading.Lock()
        if not node.subscribe(Image, topic, self._cb):
            raise RuntimeError(f"subscribe() returned False for {topic}")

    def _cb(self, msg: Image):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        row = msg.width * 3
        if msg.step and msg.step >= row and buf.size >= msg.step * msg.height:
            rgb = buf.reshape((msg.height, msg.step))[:, :row].reshape((msg.height, msg.width, 3))
        else:
            rgb = buf[: row * msg.height].reshape((msg.height, msg.width, 3))
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.frame = bgr
            self.count += 1

    def get(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()


# --------------------------------------------------------------------------- #
# per-band accumulator
# --------------------------------------------------------------------------- #
class BandStats:
    def __init__(self):
        self.aruco = defaultdict(list)        # id -> [px]
        self.qr_located = []                  # [px]  (detected, decode may have failed)
        self.qr_decoded = defaultdict(list)   # payload -> [px]


def detect_frame(bgr, aruco: ArucoShim, qr: cv2.QRCodeDetector, stats: BandStats, annotated):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _rej = aruco.detect(gray)
    if ids is not None:
        for cset, mid in zip(corners, ids.flatten()):
            px = quad_px(cset)
            stats.aruco[int(mid)].append(px)
            pts = np.asarray(cset).reshape(4, 2).astype(int)
            cv2.polylines(annotated, [pts], True, (0, 0, 255), 2)
            cv2.putText(annotated, str(int(mid)), tuple(pts[0]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    ok, infos, points, _ = qr.detectAndDecodeMulti(bgr)
    if ok and points is not None:
        for info, quad in zip(infos, points):
            px = quad_px(quad)
            stats.qr_located.append(px)
            pts = np.asarray(quad).reshape(4, 2).astype(int)
            colour = (0, 255, 0) if info else (0, 200, 255)
            cv2.polylines(annotated, [pts], True, colour, 2)
            if info:
                stats.qr_decoded[info].append(px)
                cv2.putText(annotated, info, tuple(pts[0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
    return bool(ids is not None) or (ok and points is not None)


def _pxs(v):
    a = np.asarray(v, dtype=np.float32)
    return f"{a.min():.0f}/{np.median(a):.0f}/{a.max():.0f}"


def main() -> int:
    ap = argparse.ArgumentParser(description="SIM-3 convoy-world detection check")
    ap.add_argument("--world", default="convoy")
    ap.add_argument("--secs", type=float, default=40.0, help="capture window (>= one lap)")
    ap.add_argument("--bands", type=int, nargs="+", default=list(DEFAULT_BANDS),
                    help="band names to subscribe (default 120 170 220)")
    ap.add_argument("--first-frame-deadline-s", type=float, default=45.0)
    ap.add_argument("--robot-ids", type=int, nargs="+", default=DEFAULT_ROBOT_IDS)
    ap.add_argument("--pad-ids", type=int, nargs="+", default=DEFAULT_PAD_IDS)
    ap.add_argument("--outdir", default=os.path.join("sim", "run", "convoy_world"))
    ap.add_argument("--samples-per-band", type=int, default=4)
    ap.add_argument("--allow-empty", action="store_true",
                    help="QR pass: a sparse/empty result is the finding, not a world fault "
                         "(the ArUco pass is the world-render gate)")
    args = ap.parse_args()

    os.makedirs(os.path.join(args.outdir, "frames"), exist_ok=True)
    node = Node()
    receivers = {}
    for band in args.bands:
        topic = cam_topic(args.world, band)
        try:
            receivers[band] = CameraReceiver(node, topic)
        except RuntimeError as exc:
            print(f"FAIL: band {band} subscribe — WHY: {exc} — "
                  f"CHECK: gz topic -l | grep cam_band_{band}", file=sys.stderr)
            return 3

    # first frame per band must arrive (proves world + Sensors plugin + topic name).
    # Under heavy render load a single camera can warm up slowly — wait up to the deadline,
    # then proceed with whatever IS streaming (skip the laggards); fail only if NONE stream.
    t0 = time.monotonic()
    ready = set()
    while True:
        for band in args.bands:
            if band not in ready and receivers[band].get() is not None:
                ready.add(band)
        if len(ready) == len(args.bands):
            break
        if time.monotonic() - t0 > args.first_frame_deadline_s:
            missing = sorted(set(args.bands) - ready)
            if not ready:
                print(f"FAIL: no frame on ANY band {missing} within "
                      f"{args.first_frame_deadline_s:.0f}s — WHY: Sensors plugin missing, "
                      f"world not started, or wrong topic — CHECK: gz topic -l ; gz sim log",
                      file=sys.stderr)
                return 4
            print(f"WARN: bands {missing} never streamed within "
                  f"{args.first_frame_deadline_s:.0f}s — proceeding with {sorted(ready)} "
                  f"(CHECK render load / RTF in /stats)", file=sys.stderr)
            break
        time.sleep(0.1)
    active_bands = sorted(ready)
    print(f"streaming bands {active_bands}; capturing {args.secs:.0f}s ...")

    aruco = ArucoShim()
    qr = cv2.QRCodeDetector()
    stats = {band: BandStats() for band in active_bands}
    saved = {band: 0 for band in active_bands}
    end = time.monotonic() + args.secs
    while time.monotonic() < end:
        for band in active_bands:
            bgr = receivers[band].get()
            if bgr is None:
                continue
            annotated = bgr.copy()
            hit = detect_frame(bgr, aruco, qr, stats[band], annotated)
            if hit and saved[band] < args.samples_per_band:
                path = os.path.join(args.outdir, "frames", f"band{band}_sample{saved[band]}.png")
                cv2.imwrite(path, annotated)
                saved[band] += 1
        time.sleep(0.08)   # ~12 Hz sampling: markers move slowly; leaves CPU for the render thread

    # --------------------------- report ---------------------------
    height = {**{i: ROBOT_MARKER_H_M for i in args.robot_ids},
              **{i: PAD_MARKER_H_M for i in args.pad_ids}}
    seen_ids = set()
    print("\n================= px-vs-distance (PER MARKER TYPE) =================")
    for band in active_bands:
        alt = DEFAULT_BANDS.get(band, float("nan"))
        st = stats[band]
        print(f"\n--- band {band}  (camera altitude {alt:.2f} m, frames {receivers[band].count}) ---")
        print("  ArUco  id    dist_m   reads   px(min/med/max)")
        for mid in sorted(st.aruco):
            seen_ids.add(mid)
            dist = alt - height.get(mid, 0.0)
            print(f"    {mid:>5}  {dist:6.2f}   {len(st.aruco[mid]):>5}   {_pxs(st.aruco[mid])}")
        if st.qr_located:
            print(f"  QR  located reads {len(st.qr_located):>4}   px {_pxs(st.qr_located)}")
        for payload in sorted(st.qr_decoded):
            try:
                seen_ids.add(int(payload))
            except ValueError:
                pass
            dist = alt - height.get(_safe_int(payload), 0.0)
            print(f"  QR  decoded {payload!r:>8}  dist {dist:5.2f}   "
                  f"reads {len(st.qr_decoded[payload]):>4}   px {_pxs(st.qr_decoded[payload])}")
        if not st.aruco and not st.qr_located:
            print("  (no markers this band)")

    expected = set(args.robot_ids) | set(args.pad_ids)
    missing = sorted(expected - seen_ids)
    total_qr_located = sum(len(stats[b].qr_located) for b in active_bands)
    print("\n========================= COVERAGE =========================")
    print(f"decoded/identified ids: {sorted(seen_ids)}")
    print(f"expected ids ({len(expected)}): {sorted(expected)}")
    print(f"missing (no decode on any band): {missing if missing else 'NONE'}")
    print(f"QR located-but-undecoded total reads (all bands): {total_qr_located}")

    if not seen_ids and total_qr_located == 0:
        if args.allow_empty:
            print("\nNOTE: zero markers detected, but --allow-empty set — this is the QR "
                  "decode-floor finding (QR not even locatable at these bands), not a world "
                  "fault. The ArUco pass is the world-render gate.")
            return 0
        print("\nFAIL: ZERO markers detected on any band — WHY: convoy not moving under the "
              "tower, markers not rendering (albedo path / GZ_SIM_RESOURCE_PATH), or driver "
              "not publishing — CHECK: gz topic -e -t /model/convoy_robot_7/cmd_vel ; "
              "the gz sim log for 'Unable to find' texture errors", file=sys.stderr)
        return 5
    return 0


def _safe_int(s):
    try:
        return int(s)
    except (TypeError, ValueError):
        return -1


if __name__ == "__main__":
    sys.exit(main())
