"""
YOLO Data Collection — Hybrid Keyboard Fly + Auto-Tick + Burst Capture
=======================================================================
Fly the PX4 SITL drone with keyboard while the script continuously fills
a YOLO-ready dataset folder with RGB frames + pose sidecar JSON.

Flight keys (mirror keyboardcontrol.py):
  W / S       Climb / Descend
  A / D       Yaw CCW / CW
  U / J       Forward / Backward
  H / K       Left / Right
  SPACE       Full stop hover
  T           Arm + Takeoff
  L           Land
  Q           Quit

Data-collection keys:
  O           Toggle auto-save on / off
  P           Burst: queue 10 frames to save on next 10 ticks
  C           Single shot now
  [ / ]       Decrease / Increase auto-save interval (cycles 0.25 / 0.5 / 1.0 / 2.0 s)
  N           Cycle class hint filename prefix: yellow -> red -> mixed -> yellow

Sidecar JSON is written next to nothing (separate session_meta/ folder) so it
never leaks into the YOLO training set.

Install:
  pip install mavsdk opencv-python numpy
  (gz.transport13, gz.msgs10 provided by Gazebo Harmonic install)
"""

import asyncio
import json
import os
import select
import sys
import termios
import threading
import time
import tty

import cv2
import grpc
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# ── Tunables ───────────────────────────────────────────────────────────────
MAVSDK_ADDRESS    = "udpin://0.0.0.0:14540"
CAMERA_TOPIC      = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"
TAKEOFF_ALTITUDE  = 2.5

SPEED_XY = 1.0
SPEED_Z  = 1.0
YAW_RATE = 30.0
KEY_HOLD_TIMEOUT = 0.12

INTERVAL_OPTIONS = [0.25, 0.5, 1.0, 2.0]
DEFAULT_INTERVAL_IDX = 2          # 1.0 s
BURST_FRAMES = 10
CLASS_PREFIXES = ["yellow", "red", "mixed"]

DATA_ROOT = "data"
IMG_DIR   = os.path.join(DATA_ROOT, "train", "images")
LBL_DIR   = os.path.join(DATA_ROOT, "train", "labels")
VAL_IMG_DIR = os.path.join(DATA_ROOT, "validation", "images")
VAL_LBL_DIR = os.path.join(DATA_ROOT, "validation", "labels")
META_DIR  = "session_meta"
CLASSES_FILE = "classes.txt"

# ── Shared state ───────────────────────────────────────────────────────────
class State:
    # flight
    running         : bool = True
    takeoff         : bool = False
    land            : bool = False
    offboard_active : bool = False
    # telemetry (set by stream tasks)
    pos_n : float = 0.0
    pos_e : float = 0.0
    pos_d : float = 0.0
    yaw_deg : float = 0.0
    # capture
    auto_save_on : bool = False
    interval_idx : int  = DEFAULT_INTERVAL_IDX
    burst_remaining : int = 0
    single_shot : bool = False
    class_prefix_idx : int = 0

state = State()

# ── Active key tracking ────────────────────────────────────────────────────
_key_lock      = threading.Lock()
_active_key    = ''
_active_key_ts = 0.0

def _update_active_key(k: str):
    global _active_key, _active_key_ts
    with _key_lock:
        _active_key    = k
        _active_key_ts = time.monotonic()

def _get_active_key() -> str:
    with _key_lock:
        if _active_key and (time.monotonic() - _active_key_ts) < KEY_HOLD_TIMEOUT:
            return _active_key
        return ''

# key -> (forward, right, down, yaw_deg_s)
VEL_MAP = {
    'u': ( SPEED_XY,  0.0,       0.0,      0.0     ),
    'j': (-SPEED_XY,  0.0,       0.0,      0.0     ),
    'h': ( 0.0,      -SPEED_XY,  0.0,      0.0     ),
    'k': ( 0.0,       SPEED_XY,  0.0,      0.0     ),
    'w': ( 0.0,       0.0,      -SPEED_Z,  0.0     ),
    's': ( 0.0,       0.0,       SPEED_Z,  0.0     ),
    'a': ( 0.0,       0.0,       0.0,     -YAW_RATE),
    'd': ( 0.0,       0.0,       0.0,      YAW_RATE),
}

# ── Camera (Gazebo) ─────────────────────────────────────────────────────────
_frame_lock = threading.Lock()
_latest_frame_bgr: np.ndarray = None
_frame_count = 0

def _image_callback(msg: Image):
    global _latest_frame_bgr, _frame_count
    try:
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        with _frame_lock:
            _latest_frame_bgr = bgr
            _frame_count += 1
    except Exception as e:
        print(f"\n[CAM] decode error: {e}")

def _grab_latest():
    with _frame_lock:
        return None if _latest_frame_bgr is None else _latest_frame_bgr.copy()

# ── Terminal helpers ───────────────────────────────────────────────────────
class RawTerminal:
    def __enter__(self):
        self.fd  = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self
    def __exit__(self, *_):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
    def read_key(self, timeout=0.05) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if ready:
            return os.read(self.fd, 1).decode('utf-8', errors='ignore').lower()
        return ''

def out(msg: str):
    sys.stdout.write(msg)
    sys.stdout.flush()

def print_banner():
    out("\n" + "=" * 62 + "\n")
    out("  YOLO DATA COLLECTOR — keyboard fly + auto-save + burst\n")
    out("=" * 62 + "\n")
    out("  W/S climb/desc   A/D yaw   U/J fwd/back   H/K left/right\n")
    out("  SPACE hover  T takeoff  L land  Q quit\n")
    out("  O toggle auto-save   P burst-10   C single shot\n")
    out("  [ / ] adjust interval   N cycle class prefix\n")
    out("=" * 62 + "\n\n")

# ── Keyboard thread ─────────────────────────────────────────────────────────
def keyboard_thread():
    print_banner()
    with RawTerminal() as term:
        while state.running:
            key = term.read_key(timeout=0.05)
            if not key:
                continue

            if key in VEL_MAP:
                _update_active_key(key)

            elif key == ' ':
                _update_active_key('')
                out("\n[KEY] SPACE -> Full stop\n")

            elif key == 't':
                state.takeoff = True
                out("\n[KEY] T -> Takeoff requested\n")

            elif key == 'l':
                state.land = True
                out("\n[KEY] L -> Land requested\n")

            elif key == 'q':
                state.running = False
                out("\n[KEY] Q -> Quit\n")
                break

            elif key == 'o':
                state.auto_save_on = not state.auto_save_on
                out(f"\n[CAP] auto-save = {state.auto_save_on}\n")

            elif key == 'p':
                state.burst_remaining = BURST_FRAMES
                out(f"\n[CAP] burst queued: {BURST_FRAMES} frames\n")

            elif key == 'c':
                state.single_shot = True
                out("\n[CAP] single shot queued\n")

            elif key == '[':
                state.interval_idx = max(0, state.interval_idx - 1)
                out(f"\n[CAP] interval = {INTERVAL_OPTIONS[state.interval_idx]:.2f} s\n")

            elif key == ']':
                state.interval_idx = min(len(INTERVAL_OPTIONS) - 1, state.interval_idx + 1)
                out(f"\n[CAP] interval = {INTERVAL_OPTIONS[state.interval_idx]:.2f} s\n")

            elif key == 'n':
                state.class_prefix_idx = (state.class_prefix_idx + 1) % len(CLASS_PREFIXES)
                out(f"\n[CAP] class prefix = {CLASS_PREFIXES[state.class_prefix_idx]}\n")

# ── MAVSDK helpers ──────────────────────────────────────────────────────────
async def connect(drone: System):
    print(f"[MAVSDK] Connecting to {MAVSDK_ADDRESS} ...")
    await drone.connect(system_address=MAVSDK_ADDRESS)
    async for health in drone.telemetry.health():
        print(f"[HEALTH] GPS={health.is_global_position_ok}  "
              f"Home={health.is_home_position_ok}  "
              f"Arm={health.is_armable}")
        if health.is_global_position_ok and health.is_home_position_ok:
            break
    print("[MAVSDK] Connected and healthy.")

async def arm_and_takeoff(drone: System):
    print("[MAVSDK] Arming ...")
    await drone.action.arm()
    print(f"[MAVSDK] Taking off to {TAKEOFF_ALTITUDE} m ...")
    await drone.action.takeoff()
    async for pos in drone.telemetry.position():
        alt = pos.relative_altitude_m
        sys.stdout.write(f"\r[MAVSDK] Alt: {alt:.2f} / {TAKEOFF_ALTITUDE:.2f} m   ")
        sys.stdout.flush()
        if alt >= TAKEOFF_ALTITUDE - 0.20:
            break
    print(f"\n[MAVSDK] Reached {alt:.2f} m – takeoff complete.")

async def start_offboard(drone: System):
    try:
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    except grpc.aio.AioRpcError as e:
        print(f"[ERROR] gRPC link to mavsdk_server dead before offboard prime: "
              f"{e.code()} – {e.details()}")
        print("[HINT]  Check MAVSDK_ADDRESS scheme (must be udpin://) and that "
              "PX4 SITL is publishing on UDP :14540.")
        state.running = False
        raise
    try:
        await drone.offboard.start()
        state.offboard_active = True
        print("[MAVSDK] Offboard mode ACTIVE.")
    except OffboardError as e:
        print(f"[ERROR] Offboard start failed: {e._result.result}")
        raise

# ── Telemetry stream tasks ──────────────────────────────────────────────────
async def stream_pose(drone: System, stop: asyncio.Event):
    try:
        async for pv in drone.telemetry.position_velocity_ned():
            if stop.is_set():
                break
            state.pos_n = pv.position.north_m
            state.pos_e = pv.position.east_m
            state.pos_d = pv.position.down_m
    except asyncio.CancelledError:
        pass
    except grpc.aio.AioRpcError as e:
        print(f"\n[STREAM] pose stream lost: {e.code()} – stopping control loop")
        state.running = False

async def stream_yaw(drone: System, stop: asyncio.Event):
    try:
        async for att in drone.telemetry.attitude_euler():
            if stop.is_set():
                break
            state.yaw_deg = att.yaw_deg
    except asyncio.CancelledError:
        pass
    except grpc.aio.AioRpcError as e:
        print(f"\n[STREAM] yaw stream lost: {e.code()} – stopping control loop")
        state.running = False

# ── Control loop (20 Hz body-velocity setpoints) ────────────────────────────
async def control_loop(drone: System):
    print("[MAVSDK] Control loop running at 20 Hz ...")
    dt = 0.05
    prev_key = ''

    while state.running:
        if state.takeoff:
            state.takeoff = False
            await arm_and_takeoff(drone)
            await start_offboard(drone)

        if state.land:
            state.land            = False
            state.offboard_active = False
            _update_active_key('')
            print("[MAVSDK] Landing ...")
            try:
                await drone.offboard.stop()
            except Exception:
                pass
            await drone.action.land()
            await asyncio.sleep(8)
            print("[MAVSDK] Landed.")

        if not state.offboard_active:
            await asyncio.sleep(dt)
            continue

        active = _get_active_key()
        fwd, rgt, dwn, yaw = VEL_MAP.get(active, (0.0, 0.0, 0.0, 0.0))

        if active != prev_key:
            if active:
                print(f"\n[CTL] '{active.upper()}' ACTIVE  "
                      f"fwd={fwd:+.1f} rgt={rgt:+.1f} dwn={dwn:+.1f} yaw={yaw:+.1f}")
            else:
                print(f"\n[CTL] Released – hovering")
            prev_key = active

        try:
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    forward_m_s    = fwd,
                    right_m_s      = rgt,
                    down_m_s       = dwn,
                    yawspeed_deg_s = yaw,
                )
            )
        except grpc.aio.AioRpcError as e:
            print(f"\n[CTL] gRPC drop in setpoint: {e.code()} – exiting loop. "
                  f"PX4 failsafe will take over.")
            state.running = False
            break

        await asyncio.sleep(dt)

# ── Capture scheduler ───────────────────────────────────────────────────────
def _write_sample(frame_bgr, prefix, ts, pos_n, pos_e, pos_d, yaw_deg, auto_save_on, burst_remaining, single_shot):
    """Runs in default executor — blocking disk I/O off the event loop."""
    ts_ms = int(ts * 1000)
    fname = f"{prefix}_{ts_ms}.jpg"
    img_path  = os.path.join(IMG_DIR,  fname)
    meta_path = os.path.join(META_DIR, f"{prefix}_{ts_ms}.json")

    cv2.imwrite(img_path, frame_bgr)
    sidecar = {
        "ts": ts,
        "image": fname,
        "pose_ned": {"n": pos_n, "e": pos_e, "d": pos_d},
        "yaw_deg": yaw_deg,
        "altitude_m": -pos_d,  # NED down positive -> up = -d
        "class_hint": prefix,
        "trigger": (
            "burst" if burst_remaining > 0 else
            "single" if single_shot else
            "auto" if auto_save_on else
            "unknown"
        ),
    }
    with open(meta_path, "w") as f:
        json.dump(sidecar, f, indent=2)
    return img_path

async def capture_scheduler():
    loop = asyncio.get_running_loop()
    saved = 0
    while state.running:
        interval = INTERVAL_OPTIONS[state.interval_idx]
        await asyncio.sleep(interval)

        want = state.auto_save_on or state.burst_remaining > 0 or state.single_shot
        if not want:
            continue

        frame = _grab_latest()
        if frame is None:
            out("\n[CAP] no camera frame yet — is Gazebo running?\n")
            continue

        # snapshot trigger context BEFORE decrementing
        snap_auto   = state.auto_save_on
        snap_burst  = state.burst_remaining
        snap_single = state.single_shot

        path = await loop.run_in_executor(
            None,
            _write_sample,
            frame,
            CLASS_PREFIXES[state.class_prefix_idx],
            time.time(),
            state.pos_n, state.pos_e, state.pos_d,
            state.yaw_deg,
            snap_auto, snap_burst, snap_single,
        )

        if state.burst_remaining > 0:
            state.burst_remaining -= 1
        state.single_shot = False
        saved += 1
        out(f"\n[CAP] saved {path}  ({saved} total)\n")

# ── Init filesystem ─────────────────────────────────────────────────────────
def init_dirs():
    for d in (IMG_DIR, LBL_DIR, VAL_IMG_DIR, VAL_LBL_DIR, META_DIR):
        os.makedirs(d, exist_ok=True)
    if not os.path.exists(CLASSES_FILE):
        with open(CLASSES_FILE, "w") as f:
            f.write("yellow_barrel\nred_barrel\n")
        print(f"[INIT] wrote {CLASSES_FILE}")

# ── Shutdown ────────────────────────────────────────────────────────────────
async def shutdown(drone: System):
    print("[MAVSDK] Shutting down ...")
    state.offboard_active = False
    try:
        await drone.offboard.stop()
    except Exception:
        pass
    try:
        await drone.action.disarm()
    except Exception:
        pass
    print("[MAVSDK] Done.")

# ── Main ───────────────────────────────────────────────────────────────────
async def main():
    init_dirs()

    # Camera subscriber — node MUST stay referenced for whole process lifetime
    cam_node = Node()
    if not cam_node.subscribe(Image, CAMERA_TOPIC, _image_callback):
        print(f"[CAM] FAILED to subscribe to {CAMERA_TOPIC}. Is Gazebo running?")
        return
    print(f"[CAM] Subscribed to {CAMERA_TOPIC}")

    drone = System()
    await connect(drone)

    stop = asyncio.Event()
    pose_task = asyncio.create_task(stream_pose(drone, stop))
    yaw_task  = asyncio.create_task(stream_yaw(drone, stop))
    cap_task  = asyncio.create_task(capture_scheduler())

    kb = threading.Thread(target=keyboard_thread, daemon=True)
    kb.start()

    print("[INFO] Press T to takeoff. Press O to start auto-save once in air.\n")

    try:
        await control_loop(drone)
    except asyncio.CancelledError:
        pass
    finally:
        stop.set()
        for t in (pose_task, yaw_task, cap_task):
            t.cancel()
            try:
                await t
            except (asyncio.CancelledError, Exception):
                pass
        await shutdown(drone)
        # keep cam_node alive until here so callback isn't called on freed obj
        del cam_node

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
