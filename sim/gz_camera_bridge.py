#!/usr/bin/env python3
"""gz_camera_bridge — the SIM-4 frame transport: gz camera -> localhost TCP.

WHY THIS EXISTS: finals/ runs on a Python 3.11 venv (asyncio.timeout), but the
apt `gz.transport13` bindings are compiled for system 3.10 and will NOT import
inside the venv (sim/README "gz bindings wrinkle"). So this sidecar runs the
PROVEN check_detection.py gz subscriber under SYSTEM python 3.10 and forwards raw
RGB frames over a localhost TCP socket; the venv-side finals.vision.gazebo_video
.GazeboRgbSource is the client. This file lives in sim/ BY DESIGN (outside the
finals conventions/SDK scan) so raw gz/numpy and `except Exception` are allowed.

LAUNCH under SYSTEM python3 (NOT the .venv) with PYTHONNOUSERSITE=1 (a pip --user
protobuf shadows the apt protobuf the gz _pb2 modules need — see check_detection):

    PYTHONNOUSERSITE=1 python3 sim/gz_camera_bridge.py \
        --topic /world/convoy/model/cam_band_170/link/camera_link/sensor/camera/image \
        --port 5600

or build the structural topic from parts:

    PYTHONNOUSERSITE=1 python3 sim/gz_camera_bridge.py \
        --world convoy --model x500_mono_cam_640_0 --port 5600

It blocks until the FIRST gz frame (proves the world + Sensors plugin + topic),
prints `BRIDGE READY` (and optionally writes --ready-file) so the run-script can
gate finals on it, then serves the LATEST frame to one connected client at a time
(latest-drop: a slow client never back-pressures the gz callback thread). The gz
camera renders BLANK under ogre2 on the VM — launch the world under llvmpipe
(LIBGL_ALWAYS_SOFTWARE=1) or ogre1 (sim_sessions.md SIM-3 render finding).

Wire format (KEEP IN SYNC with finals/vision/gazebo_video.py), big-endian:
  [u32 total_len][u64 frame_no][u32 width][u32 height][u8 channels][raw RGB bytes]
total_len counts everything after itself (header + payload). Payload is the
gz-native R8G8B8; the venv source reverses to BGR. Fail-loud: every wait has a
deadline; a missing topic / no first frame exits nonzero with WHAT/WHY/CHECK.
"""

import argparse
import os
import signal
import socket
import struct
import sys
import threading
import time

import numpy as np

try:
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
except Exception as exc:  # noqa: BLE001 — want a loud, specific message here
    print(f"FAIL: cannot import gz.transport13 / gz.msgs10 — WHY: wrong "
          f"interpreter or protobuf shadow — WHICH: run as `PYTHONNOUSERSITE=1 "
          f"python3` (system 3.10), NOT the .venv — CHECK: {exc}", file=sys.stderr)
    sys.exit(2)

# Wire framing — MUST match finals/vision/gazebo_video.py (_LEN_FMT/_HDR_FMT).
_LEN_FMT = ">I"
_HDR_FMT = ">QIIB"


def build_topic(world: str, model: str, link: str, sensor: str) -> str:
    return f"/world/{world}/model/{model}/link/{link}/sensor/{sensor}/image"


# --------------------------------------------------------------------------- #
# latest-frame gz subscriber — mirrors sim/check_detection.py CameraReceiver,
# but keeps RAW RGB (the venv source normalizes to BGR — no cv2 needed here).
# --------------------------------------------------------------------------- #
class CameraReceiver:
    def __init__(self, node: Node, topic: str):
        self.topic = topic
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
            self.frame = np.ascontiguousarray(rgb)   # RGB, contiguous for tobytes()
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


def serve_client(conn: socket.socket, receiver: CameraReceiver) -> None:
    """Send the LATEST frame to one client until it disconnects/stalls. Only a
    NEW frame is sent (latest-drop: intermediate frames that piled up are
    skipped — we always send the most recent), so a slow client cannot stall
    the gz callback thread."""
    conn.settimeout(2.0)
    last_sent = 0
    while not _STOP.is_set():
        got = receiver.get()
        if got is None:
            time.sleep(0.01)
            continue
        frame, w, h, count = got
        if count == last_sent:
            time.sleep(0.005)            # no new frame; latest already sent
            continue
        body = struct.pack(_HDR_FMT, count, w, h, 3) + frame.tobytes()
        msg = struct.pack(_LEN_FMT, len(body)) + body
        try:
            conn.sendall(msg)
        except (BrokenPipeError, ConnectionResetError, socket.timeout,
                OSError) as e:
            print(f"[bridge] client gone/slow ({type(e).__name__}: {e}) — "
                  f"re-accepting", file=sys.stderr, flush=True)
            return
        last_sent = count


def main() -> int:
    ap = argparse.ArgumentParser(description="SIM-4 gz camera -> localhost TCP bridge")
    ap.add_argument("--topic", default=None,
                    help="explicit gz camera Image topic (wins over --world/--model)")
    ap.add_argument("--world", default="convoy")
    ap.add_argument("--model", default="x500_mono_cam_640_0",
                    help="gz model name (e.g. cam_band_170 for the static tower cam)")
    ap.add_argument("--link", default="camera_link")
    ap.add_argument("--sensor", default="camera")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--first-frame-deadline-s", type=float, default=45.0,
                    help="llvmpipe warms up slowly; match check_detection.py")
    ap.add_argument("--ready-file", default=None,
                    help="touch this path once the first gz frame arrives "
                         "(the run-script gates finals on it)")
    ap.add_argument("--count-secs", type=float, default=0.0,
                    help="STATS MODE (SIM-5 probe3): after the first frame, count "
                         "frames for this many seconds, print one `BRIDGE FPS ...` "
                         "line, and exit WITHOUT serving TCP. Used to measure "
                         "per-camera render rate under the 3-cam load.")
    args = ap.parse_args()

    topic = args.topic or build_topic(args.world, args.model, args.link,
                                       args.sensor)
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    node = Node()
    try:
        receiver = CameraReceiver(node, topic)
    except RuntimeError as exc:
        print(f"FAIL: subscribe to {topic} — WHY: {exc} — "
              f"CHECK: gz topic -l | grep image", file=sys.stderr)
        return 3

    # First frame proves world + Sensors plugin + topic name + non-blank render.
    t0 = time.monotonic()
    while receiver.get() is None:
        if _STOP.is_set():
            return 0
        if time.monotonic() - t0 > args.first_frame_deadline_s:
            print(f"FAIL: no frame on {topic} within "
                  f"{args.first_frame_deadline_s:.0f}s — WHY: Sensors plugin "
                  f"missing, wrong topic, world not started, or BLANK render "
                  f"under ogre2 — CHECK: gz topic -l ; launch the world under "
                  f"llvmpipe (LIBGL_ALWAYS_SOFTWARE=1) or ogre1", file=sys.stderr)
            return 4
        time.sleep(0.1)

    if args.ready_file:
        try:
            with open(args.ready_file, "w", encoding="utf-8") as f:
                f.write(f"{topic}\n")
        except OSError as e:
            print(f"[bridge] WARN could not write --ready-file "
                  f"{args.ready_file!r}: {e}", file=sys.stderr, flush=True)
    # STATS MODE (SIM-5 probe3): count frames over the window, report fps, exit.
    # No TCP serve — this is purely the render-load measurement.
    if args.count_secs > 0:
        first = time.monotonic()
        c0 = receiver.get()[3]
        while not _STOP.is_set() and time.monotonic() - first < args.count_secs:
            time.sleep(0.05)
        elapsed = max(time.monotonic() - first, 1e-3)
        c1 = receiver.get()[3]
        fps = (c1 - c0) / elapsed
        print(f"BRIDGE FPS topic={topic} frames={c1 - c0} "
              f"secs={elapsed:.1f} fps={fps:.1f}", flush=True)
        return 0

    print(f"BRIDGE READY: first frame on {topic} after "
          f"{time.monotonic() - t0:.1f}s; serving on {args.host}:{args.port}",
          flush=True)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind((args.host, args.port))
    except OSError as e:
        print(f"FAIL: bind {args.host}:{args.port} — WHY: {e} — "
              f"CHECK: port already in use? (ss -ltnp | grep {args.port})",
              file=sys.stderr)
        return 5
    srv.listen(1)
    srv.settimeout(0.5)                  # so the accept loop can see _STOP

    print(f"[bridge] accepting clients on {args.host}:{args.port} "
          f"(Ctrl-C / SIGTERM to stop)", flush=True)
    try:
        while not _STOP.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError as e:
                if _STOP.is_set():
                    break
                print(f"[bridge] accept failed ({type(e).__name__}: {e})",
                      file=sys.stderr, flush=True)
                continue
            print(f"[bridge] client connected: {addr}", flush=True)
            try:
                serve_client(conn, receiver)
            finally:
                try:
                    conn.close()
                except OSError:
                    pass
    finally:
        try:
            srv.close()
        except OSError:
            pass
    print("[bridge] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
