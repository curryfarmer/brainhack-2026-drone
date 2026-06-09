#!/usr/bin/env python3
"""gz_video_record — subscribe a gz camera topic and SAVE it to an mp4 (NAV-9).

The watchable-footage sibling of sim/gz_camera_bridge.py: instead of forwarding
frames to finals over TCP, it writes each NEW gz camera frame to a video file so
the operator can review a flight asynchronously (the user asked to "physically
view it + review the footage"). Used by sim/run_landing.sh `viewtest` to record
BOTH the third-person overview camera (landing_view.sdf) and the drone's onboard
down-camera while the single drone flies takeoff -> navigate (crate detour) ->
land_on_pad.

Same interpreter contract as gz_camera_bridge.py: launch under SYSTEM python3 with
PYTHONNOUSERSITE=1 (the gz.transport13 bindings are apt/system 3.10, not the venv;
a pip --user protobuf shadows the apt one). cv2 is imported for the encoder — the
system python has it (it is the same one gen_markers.py / check_detection use).
This file lives in sim/ BY DESIGN (outside the finals conventions/SDK scan) so raw
gz/cv2/numpy and `except Exception` are allowed.

    PYTHONNOUSERSITE=1 python3 sim/gz_video_record.py \
        --topic /world/landing_view/model/overview_cam/link/link/sensor/camera/image \
        --out sim/run/landing_overview.mp4 --secs 240 --fps 10

Multiple subscribers to one gz topic are fine, so the onboard recorder can run
ALONGSIDE the onboard TCP bridge on the same topic (render load is per-CAMERA,
not per-subscriber). Fail-loud: no first frame within the deadline exits nonzero
with WHAT/WHY/CHECK. On SIGTERM/SIGINT it finalizes the file cleanly so a killed
recorder still leaves a playable video.
"""

import argparse
import os
import signal
import sys
import threading
import time

import numpy as np

try:
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
except Exception as exc:  # noqa: BLE001 — loud, specific
    print(f"FAIL: cannot import gz.transport13 / gz.msgs10 — WHY: wrong "
          f"interpreter or protobuf shadow — WHICH: run as `PYTHONNOUSERSITE=1 "
          f"python3` (system 3.10), NOT the .venv — CHECK: {exc}", file=sys.stderr)
    sys.exit(2)

try:
    import cv2
except Exception as exc:  # noqa: BLE001
    print(f"FAIL: cannot import cv2 — WHY: no opencv in this interpreter — "
          f"CHECK: the system python3 has cv2 (gen_markers/check_detection use "
          f"it)  ({exc})", file=sys.stderr)
    sys.exit(2)


class _Receiver:
    """Latest-frame gz subscriber (mirrors gz_camera_bridge.CameraReceiver) — keeps
    the most recent RGB frame + a monotonically increasing count."""

    def __init__(self, node: Node, topic: str):
        self.frame = None
        self.w = 0
        self.h = 0
        self.count = 0
        self.lock = threading.Lock()
        if not node.subscribe(Image, topic, self._cb):
            raise RuntimeError(f"subscribe() returned False for {topic}")

    def _cb(self, msg: Image):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        row = msg.width * 3
        if msg.step and msg.step >= row and buf.size >= msg.step * msg.height:
            rgb = buf.reshape((msg.height, msg.step))[:, :row].reshape(
                (msg.height, msg.width, 3))
        else:
            rgb = buf[: row * msg.height].reshape((msg.height, msg.width, 3))
        with self.lock:
            self.frame = np.ascontiguousarray(rgb)
            self.w = msg.width
            self.h = msg.height
            self.count += 1

    def get(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame, self.w, self.h, self.count


_STOP = threading.Event()


def _on_signal(signum, _frame):
    _STOP.set()


def main() -> int:
    ap = argparse.ArgumentParser(description="NAV-9 gz camera -> mp4 recorder")
    ap.add_argument("--topic", required=True, help="gz camera Image topic")
    ap.add_argument("--out", required=True, help="output .mp4 path")
    ap.add_argument("--secs", type=float, default=240.0,
                    help="record this many WALL seconds after the first frame")
    ap.add_argument("--fps", type=float, default=10.0,
                    help="playback fps written into the mp4 (each NEW gz frame is "
                         "written once; at low RTF the gz arrival rate is slower "
                         "than this, so playback is faster than wall — fine for review)")
    ap.add_argument("--first-frame-deadline-s", type=float, default=60.0,
                    help="llvmpipe warms up slowly; matches the bridge")
    args = ap.parse_args()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    node = Node()
    try:
        rx = _Receiver(node, args.topic)
    except RuntimeError as exc:
        print(f"FAIL: subscribe to {args.topic} — WHY: {exc} — "
              f"CHECK: gz topic -l | grep image", file=sys.stderr)
        return 3

    t0 = time.monotonic()
    while rx.get() is None:
        if _STOP.is_set():
            return 0
        if time.monotonic() - t0 > args.first_frame_deadline_s:
            print(f"FAIL: no frame on {args.topic} within "
                  f"{args.first_frame_deadline_s:.0f}s — WHY: Sensors plugin "
                  f"missing / wrong topic / world not started / BLANK render "
                  f"under ogre2 — CHECK: gz topic -l ; launch under llvmpipe "
                  f"(LIBGL_ALWAYS_SOFTWARE=1)", file=sys.stderr)
            return 4
        time.sleep(0.1)

    frame, w, h, _ = rx.get()
    out_dir = os.path.dirname(os.path.abspath(args.out))
    try:
        os.makedirs(out_dir, exist_ok=True)
    except OSError as e:
        print(f"FAIL: cannot create output dir {out_dir} — {e}", file=sys.stderr)
        return 5

    # mp4v is the most widely-available encoder in stock opencv builds. If the
    # writer fails to open we fall back to a PNG sequence so footage is NEVER lost.
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(args.out, fourcc, args.fps, (w, h))
    png_fallback = not writer.isOpened()
    png_dir = None
    if png_fallback:
        png_dir = os.path.splitext(args.out)[0] + "_frames"
        os.makedirs(png_dir, exist_ok=True)
        print(f"[record] WARN VideoWriter could not open {args.out!r} (no mp4v "
              f"codec?) — falling back to a PNG sequence in {png_dir}",
              file=sys.stderr, flush=True)

    print(f"[record] topic={args.topic} -> {args.out} "
          f"({w}x{h} @ {args.fps:g} fps, {args.secs:g}s, first frame after "
          f"{time.monotonic() - t0:.1f}s)", flush=True)

    start = time.monotonic()
    last = 0
    written = 0
    while not _STOP.is_set() and time.monotonic() - start < args.secs:
        got = rx.get()
        if got is None:
            time.sleep(0.02)
            continue
        frame, w, h, count = got
        if count == last:
            time.sleep(0.01)              # no new frame yet
            continue
        last = count
        bgr = frame[:, :, ::-1]           # gz is RGB; cv2 wants BGR
        if png_fallback:
            cv2.imwrite(os.path.join(png_dir, f"frame_{written:06d}.png"), bgr)
        else:
            writer.write(np.ascontiguousarray(bgr))
        written += 1

    if not png_fallback:
        writer.release()
    elapsed = max(time.monotonic() - start, 1e-3)
    print(f"[record] done: {written} frames over {elapsed:.1f}s wall "
          f"(arrival ~{written / elapsed:.1f} fps) -> {args.out}", flush=True)
    if written == 0:
        print("[record] WARN wrote ZERO frames — the camera produced no new "
              "frames in the window (starved render? check RTF)", file=sys.stderr)
        return 6
    return 0


if __name__ == "__main__":
    sys.exit(main())
