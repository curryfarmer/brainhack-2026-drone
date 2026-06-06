"""Generate the committed synthetic vision fixtures (run ONCE, output is
committed — tests never regenerate; determinism comes from the PNGs on disk).

    python finals/tests/fixtures/gen_fixtures.py        # from the repo root

Outputs (PNG only — lossless, so decode is byte-deterministic):
- frames/000..003.png : 640x480 white canvases with cv2.aruco
  DICT_6X6_250 markers pasted at the positions in MARKER_LAYOUT below.
  EVERY marker-bearing frame carries ALL THREE ids {17, 23, 42} so
  set-based e2e assertions survive a latest-frame sampler missing a frame;
  002.png is deliberately marker-free (the "nothing to see" pin).
- frames_qr/000.png   : one QR code (payload "7") via cv2.QRCodeEncoder, in
  a SEPARATE dir so the replay-profile CSV stays ArUco-deterministic.

Tests hardcode MARKER_LAYOUT as constants (finals/tests/test_vision_aruco.py)
— if you change this table, regenerate AND update the tests.

This file lives under finals/tests/ and is exempt from the conventions scans
(test_conventions.py excludes tests/); it still keeps the house style.
"""
from __future__ import annotations

import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FRAME_W, FRAME_H = 640, 480
MARKER_PX = 120                     # 8 modules (6x6 + border) -> 15 px/module

# frame index -> list of (marker_id, x_topleft, y_topleft); bbox =
# (x, y, x + MARKER_PX, y + MARKER_PX). Frame 2 is deliberately empty.
MARKER_LAYOUT = {
    0: [(17, 40, 60), (23, 260, 180), (42, 480, 300)],
    1: [(17, 300, 40), (23, 60, 280), (42, 400, 200)],
    2: [],
    3: [(17, 500, 40), (23, 40, 40), (42, 240, 300)],
}

QR_PAYLOAD = "7"


def _blank() -> "np.ndarray":
    return np.full((FRAME_H, FRAME_W, 3), 255, dtype=np.uint8)


def gen_aruco_frames(out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    for idx, layout in sorted(MARKER_LAYOUT.items()):
        canvas = _blank()
        for marker_id, x, y in layout:
            marker = cv2.aruco.generateImageMarker(dictionary, marker_id,
                                                   MARKER_PX)
            canvas[y:y + MARKER_PX, x:x + MARKER_PX] = cv2.cvtColor(
                marker, cv2.COLOR_GRAY2BGR)
        path = os.path.join(out_dir, f"{idx:03d}.png")
        if not cv2.imwrite(path, canvas):
            raise RuntimeError(f"cv2.imwrite failed for {path}")
        print(f"wrote {path}: "
              + (", ".join(f"id {m} bbox=({x},{y},{x + MARKER_PX},{y + MARKER_PX})"
                           for m, x, y in layout) or "(no markers)"))


def gen_qr_frame(out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    enc = cv2.QRCodeEncoder.create()
    qr = enc.encode(QR_PAYLOAD)                     # small binary matrix
    scale = 10                                      # ~10 px/module
    qr_big = cv2.resize(qr, (qr.shape[1] * scale, qr.shape[0] * scale),
                        interpolation=cv2.INTER_NEAREST)
    canvas = _blank()
    h, w = qr_big.shape[:2]
    y0 = (FRAME_H - h) // 2
    x0 = (FRAME_W - w) // 2
    canvas[y0:y0 + h, x0:x0 + w] = cv2.cvtColor(qr_big, cv2.COLOR_GRAY2BGR)
    path = os.path.join(out_dir, "000.png")
    if not cv2.imwrite(path, canvas):
        raise RuntimeError(f"cv2.imwrite failed for {path}")
    print(f"wrote {path}: QR payload {QR_PAYLOAD!r} centered "
          f"({w}x{h} px at ({x0},{y0}))")


def main() -> int:
    gen_aruco_frames(os.path.join(HERE, "frames"))
    gen_qr_frame(os.path.join(HERE, "frames_qr"))
    print("done — commit the PNGs; tests hardcode MARKER_LAYOUT")
    return 0


if __name__ == "__main__":
    sys.exit(main())
