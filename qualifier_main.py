#!/usr/bin/env python3
"""
qualifier_main.py
=================
Brainhack 2026 RoboVerse Flight Challenge — Qualifier mission script.

Strategy
--------
- Discretise the depot floor into 1m x 1m cells.
- DFS from the takeoff cell. At each cell, try N -> E -> S -> W in order.
- Before moving to a neighbour cell, rotate to face it and check depth
  clearance in the center column. If blocked, mark that cell BLOCKED and try
  the next direction.
- If all four neighbours are visited or blocked, pop the DFS stack and
  backtrack along visited cells.
- Throughout the flight, YOLO runs in a background thread on the RGB
  camera. Each barrel detection gets projected to world NED and added to a
  deduplicated list (BarrelLogger).
- When the stack is empty the search is complete. Save outputs and land.

Outputs
-------
- barrels.json         : list of detected barrels in world NED coordinates.
- visited_cells.json   : map coverage record for scoring / debugging.

Tunables (top of file)
----------------------
- CELL_SIZE_M, ALTITUDE_M, MAX_RUNTIME_S, etc.

Dependencies (already on the VM)
--------------------------------
- mavsdk, ultralytics, opencv-python, numpy, gz-transport13, gz-msgs10
- All other files in the same folder:
    drone_control.py, depth_receiver.py, AvoidancePlanner.py,
    get_position_with_task.py, Detector.py, barrel_logger.py
"""

import asyncio
import json
import math
import os
import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# Gazebo transport for RGB camera
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image as GzImage

# Local modules
from depth_receiver import DepthReceiver
from drone_control import Drone
from AvoidancePlanner import AvoidancePlanner
from get_position_with_task import SharedState, position_monitor_task
from Detector import Detector
from barrel_logger import BarrelLogger


# ==========================================================================
# CONFIGURATION
# ==========================================================================

# Topics (confirmed from save_photo.py / depth_receiver default)
DEPTH_TOPIC = "/depth_camera"
RGB_TOPIC = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"

# Camera intrinsics (matches AvoidancePlanner default in deck)
CAMERA_K = np.array([
    [433.0,   0.0, 320.0],
    [  0.0, 433.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float32)
IMAGE_W, IMAGE_H = 640, 480

# YOLO
YOLO_MODEL_PATH = "best.pt"
# IMPORTANT: set this to whatever your trained model calls the barrel class.
# Run a single test detection and check det['class_name']. Common possibilities:
#   "barrel", "fuel_barrel", "canister", "drum"
# Leave list empty [] to accept ALL detected classes (useful for first runs).
BARREL_CLASS_NAMES: List[str] = ["Yellow Barrel", "Red Barrel"]
YOLO_CONFIDENCE = 0.5

# Mission geometry
CELL_SIZE_M = 2.0           # grid cell size
ALTITUDE_M = 1.0            # flight altitude (positive = up)
DOWN_SETPOINT = -ALTITUDE_M # NED down value (negative = up)

# Safety
MAX_CELLS_VISITED = 2000     # hard cap on cells before giving up
MAX_RUNTIME_S = 600         # 10 minute hard cap
WAYPOINT_TOLERANCE_M = 0.4  # consider arrival when within this distance
WAYPOINT_TIMEOUT_S = 12.0   # give up on a single waypoint after this
BLOCKED_DISTANCE_M = 2.0    # if center clearance < this when facing target, blocked
LOOP_HZ = 10.0              # main loop rate (must keep > 2 Hz for offboard)

# Heading constants (NED, deg, clockwise from North)
HEADINGS = {
    "N": 0.0,
    "E": 90.0,
    "S": 180.0,
    "W": -90.0,
}
DIRECTION_ORDER = ["N", "E", "W", "S"]  # DFS preference order
DELTA = {  # (dn, de) offsets per direction in cells
    "N": (1, 0),
    "E": (0, 1),
    "S": (-1, 0),
    "W": (0, -1),
}
OPPOSITE = {"N": "S", "S": "N", "E": "W", "W": "E"}


# ==========================================================================
# RGB CAMERA RECEIVER
# Mirrors DepthReceiver but for RGB uint8 frames, and pumps frames into
# the Detector instead of holding them.
# ==========================================================================
class RGBReceiver:
    def __init__(self, topic: str, detector: Detector):
        self.topic = topic
        self.detector = detector
        self._latest: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self.node = Node()
        ok = self.node.subscribe(GzImage, topic, self._callback)
        if not ok:
            print(f"[RGBReceiver] WARNING: subscribe to {topic} returned False")

    def _callback(self, msg: GzImage) -> None:
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8)
            frame = frame.reshape((msg.height, msg.width, 3))
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception as e:
            print(f"[RGBReceiver] decode error: {e}")
            return

        with self._lock:
            self._latest = frame_bgr

        # Push to detector (drops to backlog queue; detector worker pulls)
        # NOTE: Detector.submit_image is non-blocking append.
        self.detector.submit_image(frame_bgr.copy(), context={"timestamp": time.time()})

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._latest is None else self._latest.copy()


# ==========================================================================
# MISSION
# ==========================================================================
class QualifierMission:
    def __init__(self):
        # ---- Drone & telemetry ----
        self.drone = Drone()
        self.position_state = SharedState()
        self.stop_event = asyncio.Event()
        self.monitor_task: Optional[asyncio.Task] = None

        # ---- Perception ----
        self.depth_receiver = DepthReceiver(DEPTH_TOPIC)
        self.planner = AvoidancePlanner(
            K=CAMERA_K,
            width=IMAGE_W,
            height=IMAGE_H,
            safe_distance=3.0,
            critical_distance=BLOCKED_DISTANCE_M,
        )

        # ---- Barrel detection ----
        self.barrel_logger = BarrelLogger(
            K=CAMERA_K,
            barrel_class_names=BARREL_CLASS_NAMES,
            get_pose_fn=self._get_pose_dict_radians,
            get_depth_fn=self.depth_receiver.get_frame,
            min_confidence=YOLO_CONFIDENCE,
        )
        self.detector = Detector(
            model_path=YOLO_MODEL_PATH,
            confidence_threshold=YOLO_CONFIDENCE,
            callback=self.barrel_logger.on_detection,
            num_workers=2,
            device="cpu",
            save_dir="./detected_images",
            enable_display=True,
            display_window_name="Barrel Detector",
        )
        self.rgb_receiver = RGBReceiver(RGB_TOPIC, self.detector)

        # ---- DFS state ----
        # Cells are (cell_n, cell_e) integer coordinates relative to start
        self.visited: set = set()
        self.blocked: set = set()
        self.stack: List[Tuple[int, int]] = []
        self.current_cell: Tuple[int, int] = (0, 0)
        # Origin in NED meters (set after takeoff to the actual NED position)
        self.origin_north: float = 0.0
        self.origin_east: float = 0.0
        # Movement record: list of {cell, time}
        self.path_log: List[Dict] = []

        # ---- Timing ----
        self.start_time: float = 0.0

        # ---- Runtime ----
        self.running = True

    # ----------------------------------------------------------------------
    # Pose helpers
    # ----------------------------------------------------------------------
    def _have_pose(self) -> bool:
        return (
            self.position_state.latest_position is not None
            and self.position_state.latest_yaw is not None
        )

    def _get_pose_dict_radians(self) -> Optional[Dict[str, float]]:
        """Thread-safe pose snapshot used by BarrelLogger from detector thread."""
        pos = self.position_state.latest_position
        yaw_deg = self.position_state.latest_yaw
        if pos is None or yaw_deg is None:
            return None
        return {
            "north": float(pos.north_m),
            "east": float(pos.east_m),
            "down": float(pos.down_m),
            "yaw": math.radians(float(yaw_deg)),
        }

    def _current_ned(self) -> Tuple[float, float, float, float]:
        """Returns (north, east, down, yaw_deg). Assumes _have_pose() is True."""
        pos = self.position_state.latest_position
        return (
            float(pos.north_m),
            float(pos.east_m),
            float(pos.down_m),
            float(self.position_state.latest_yaw),
        )

    def _cell_to_ned(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        """Convert cell coords -> world NED (north, east) meters."""
        cn, ce = cell
        return (
            self.origin_north + cn * CELL_SIZE_M,
            self.origin_east + ce * CELL_SIZE_M,
        )

    # ----------------------------------------------------------------------
    # Yaw helpers
    # ----------------------------------------------------------------------
    @staticmethod
    def _yaw_error(target_deg: float, current_deg: float) -> float:
        err = target_deg - current_deg
        while err > 180:
            err -= 360
        while err < -180:
            err += 360
        return err

    async def _wait_for_pose(self, timeout_s: float = 15.0) -> bool:
        """Block until we have at least one pose telemetry reading."""
        start = time.monotonic()
        while not self._have_pose():
            if time.monotonic() - start > timeout_s:
                print("[Mission] ERROR: timed out waiting for pose telemetry")
                return False
            await asyncio.sleep(0.1)
        return True

    async def _wait_for_depth(self, timeout_s: float = 10.0) -> bool:
        start = time.monotonic()
        while self.depth_receiver.get_frame() is None:
            if time.monotonic() - start > timeout_s:
                print("[Mission] ERROR: timed out waiting for depth frames")
                return False
            await asyncio.sleep(0.1)
        return True

    # ----------------------------------------------------------------------
    # Heartbeat / setpoint streaming
    # As long as a target is "active", we keep sending it at LOOP_HZ.
    # Yields after each setpoint so other awaits can run.
    # ----------------------------------------------------------------------
    async def _stream_position_until_arrived(
        self,
        target_n: float,
        target_e: float,
        target_d: float,
        target_yaw_deg: float,
        tolerance_m: float = WAYPOINT_TOLERANCE_M,
        timeout_s: float = WAYPOINT_TIMEOUT_S,
    ) -> bool:
        """
        Keep sending the same NED setpoint at LOOP_HZ until the drone is
        within tolerance OR the timeout elapses. Returns True on arrival.
        """
        dt = 1.0 / LOOP_HZ
        start = time.monotonic()
        while self.running:
            await self.drone.send_position_setpoint(
                north=target_n, east=target_e, down=target_d, yaw_deg=target_yaw_deg
            )
            if self._have_pose():
                n, e, d, _ = self._current_ned()
                dn = n - target_n
                de = e - target_e
                horiz = math.sqrt(dn * dn + de * de)
                if horiz < tolerance_m and abs(d - target_d) < 0.5:
                    return True
            if time.monotonic() - start > timeout_s:
                print(f"[Mission] waypoint timeout at "
                      f"({target_n:.2f},{target_e:.2f})")
                return False
            await asyncio.sleep(dt)
        return False

    async def _hold_position(self, duration_s: float, yaw_deg: float) -> None:
        """Hold current NED position for duration_s, streaming setpoints."""
        if not self._have_pose():
            await asyncio.sleep(duration_s)
            return
        n, e, d, _ = self._current_ned()
        dt = 1.0 / LOOP_HZ
        steps = max(1, int(duration_s / dt))
        for _ in range(steps):
            if not self.running:
                return
            await self.drone.send_position_setpoint(
                north=n, east=e, down=DOWN_SETPOINT, yaw_deg=yaw_deg
            )
            await asyncio.sleep(dt)

    # ----------------------------------------------------------------------
    # Direction check: is the cell in 'direction' from current cell blocked?
    # ----------------------------------------------------------------------
    def _check_blocked_ahead(self) -> bool:
        """Reads depth front-center and returns True if blocked."""
        depth = self.depth_receiver.get_frame()
        if depth is None:
            return False  # don't claim blocked if no data; let timeout handle it
        # Use planner's clearance helper for consistency
        left, center, right = self.planner.compute_clearance(depth)
        # We only care about the forward (center) cell. We check it's enough
        # to clear at least CELL_SIZE_M plus a small margin.
        threshold = CELL_SIZE_M + 0.5
        if math.isnan(center) or center < threshold:
            print(f"[Mission] center clearance = {center:.2f}m < {threshold:.2f}m -> BLOCKED")
            return True
        return False

    # ----------------------------------------------------------------------
    # Rotate to a target yaw, streaming setpoints (NOT using PID helper —
    # we keep the position fixed and let PX4 handle the rotation).
    # ----------------------------------------------------------------------
    async def _rotate_to(self, target_yaw_deg: float, settle_s: float = 1.5) -> None:
        if not self._have_pose():
            return
        n, e, d, _ = self._current_ned()
        dt = 1.0 / LOOP_HZ
        deadline = time.monotonic() + 6.0  # max time to rotate
        # Stream position+yaw setpoint until yaw error is small
        while self.running and time.monotonic() < deadline:
            await self.drone.send_position_setpoint(
                north=n, east=e, down=DOWN_SETPOINT, yaw_deg=target_yaw_deg
            )
            _, _, _, cur_yaw = self._current_ned()
            if abs(self._yaw_error(target_yaw_deg, cur_yaw)) < 5.0:
                break
            await asyncio.sleep(dt)
        # Settle a bit
        steps = max(1, int(settle_s / dt))
        for _ in range(steps):
            if not self.running:
                return
            await self.drone.send_position_setpoint(
                north=n, east=e, down=DOWN_SETPOINT, yaw_deg=target_yaw_deg
            )
            await asyncio.sleep(dt)

    # ----------------------------------------------------------------------
    # DFS step: try to find an unvisited, not-blocked neighbour to move to
    # ----------------------------------------------------------------------
    def _next_unvisited_direction(self) -> Optional[str]:
        for dirn in DIRECTION_ORDER:
            dn, de = DELTA[dirn]
            neighbour = (self.current_cell[0] + dn, self.current_cell[1] + de)
            if neighbour in self.visited or neighbour in self.blocked:
                continue
            return dirn
        return None

    def _backtrack_direction(self) -> Optional[str]:
        """
        Return the direction from current_cell toward the previous cell on the
        stack (which by definition is a neighbour and is visited).
        """
        if not self.stack:
            return None
        prev = self.stack[-1]
        dn = prev[0] - self.current_cell[0]
        de = prev[1] - self.current_cell[1]
        for dirn, (ddn, dde) in DELTA.items():
            if (ddn, dde) == (dn, de):
                return dirn
        return None

    # ----------------------------------------------------------------------
    # Setup / shutdown
    # ----------------------------------------------------------------------
    async def setup(self) -> bool:
        print("[Mission] Connecting...")
        await self.drone.connect()
        await asyncio.sleep(1)

        print("[Mission] Starting position monitor task")
        self.monitor_task = asyncio.create_task(
            position_monitor_task(self.drone, self.position_state, self.stop_event)
        )

        print("[Mission] Arming and taking off")
        await self.drone.arm_and_takeoff()

        if not await self._wait_for_pose():
            return False
        if not await self._wait_for_depth():
            return False

        # Record origin
        n, e, d, yaw = self._current_ned()
        self.origin_north = n
        self.origin_east = e
        print(f"[Mission] Origin set at NED ({n:.2f}, {e:.2f}, {d:.2f}), yaw={yaw:.1f}")

        # Mark start cell visited
        self.visited.add(self.current_cell)
        self.path_log.append({
            "cell": list(self.current_cell),
            "ned": [n, e],
            "time": time.monotonic(),
        })

        # Align to north
        await self._rotate_to(HEADINGS["N"])
        self.start_time = time.monotonic()
        return True

    async def shutdown(self) -> None:
        print("[Mission] Shutdown sequence")
        self.running = False
        self.stop_event.set()

        # Stop detector worker threads
        try:
            self.detector.stop()
        except Exception as e:
            print(f"[Mission] detector.stop() error: {e}")

        # Cancel monitor
        if self.monitor_task and not self.monitor_task.done():
            self.monitor_task.cancel()
            try:
                await self.monitor_task
            except (asyncio.CancelledError, Exception):
                pass

        # Land
        try:
            await self.drone.land()
        except Exception as e:
            print(f"[Mission] land() error: {e}")

        # Save outputs
        try:
            self.barrel_logger.save_json("barrels.json")
        except Exception as e:
            print(f"[Mission] save barrels error: {e}")
        try:
            with open("visited_cells.json", "w") as f:
                json.dump({
                    "cell_size_m": CELL_SIZE_M,
                    "origin_ned": [self.origin_north, self.origin_east],
                    "visited": sorted(list(self.visited)),
                    "blocked": sorted(list(self.blocked)),
                    "path": self.path_log,
                }, f, indent=2)
            print(f"[Mission] Saved visited_cells.json "
                  f"({len(self.visited)} visited, {len(self.blocked)} blocked)")
        except Exception as e:
            print(f"[Mission] save visited error: {e}")

    # ----------------------------------------------------------------------
    # Main DFS loop
    # ----------------------------------------------------------------------
    async def run(self) -> None:
        if not await self.setup():
            print("[Mission] Setup failed, aborting")
            return

        try:
            while self.running:
                # Time guard
                if time.monotonic() - self.start_time > MAX_RUNTIME_S:
                    print("[Mission] Max runtime reached")
                    break
                if len(self.visited) > MAX_CELLS_VISITED:
                    print("[Mission] Max cells reached")
                    break

                # Pick next direction: prefer unvisited; otherwise backtrack
                dirn = self._next_unvisited_direction()
                action = "explore"
                if dirn is None:
                    dirn = self._backtrack_direction()
                    action = "backtrack"
                if dirn is None:
                    print("[Mission] DFS complete: nowhere left to go")
                    break

                target_yaw = HEADINGS[dirn]
                dn, de = DELTA[dirn]
                target_cell = (self.current_cell[0] + dn,
                               self.current_cell[1] + de)
                tn, te = self._cell_to_ned(target_cell)
                print(f"[Mission] {action} {dirn} -> cell {target_cell} "
                      f"NED({tn:.2f},{te:.2f})  stack_depth={len(self.stack)}")

                # 1) Rotate to face target direction
                await self._rotate_to(target_yaw)

                # 2) Probe forward clearance (skip during backtrack — we know
                #    it was passable because we came from there)
                if action == "explore":
                    # Give depth camera a moment after rotation
                    await self._hold_position(0.5, target_yaw)
                    if self._check_blocked_ahead():
                        print(f"[Mission] {dirn} blocked, marking {target_cell}")
                        self.blocked.add(target_cell)
                        continue

                # 3) Fly to the target cell, streaming setpoints
                arrived = await self._stream_position_until_arrived(
                    target_n=tn, target_e=te, target_d=DOWN_SETPOINT,
                    target_yaw_deg=target_yaw,
                )

                if not arrived:
                    # Treat as blocked, do NOT update current_cell
                    print(f"[Mission] failed to reach {target_cell}, marking blocked")
                    self.blocked.add(target_cell)
                    # Try to return to current_cell setpoint so we don't drift
                    cn, ce = self._cell_to_ned(self.current_cell)
                    await self._stream_position_until_arrived(
                        target_n=cn, target_e=ce, target_d=DOWN_SETPOINT,
                        target_yaw_deg=target_yaw,
                        tolerance_m=0.6, timeout_s=6.0,
                    )
                    continue

                # 4) Update DFS state
                if action == "explore":
                    self.stack.append(self.current_cell)
                    self.current_cell = target_cell
                    self.visited.add(target_cell)
                else:  # backtrack
                    self.current_cell = target_cell
                    if self.stack and self.stack[-1] == target_cell:
                        self.stack.pop()

                self.path_log.append({
                    "cell": list(self.current_cell),
                    "ned": [tn, te],
                    "time": time.monotonic(),
                    "action": action,
                })

                # 5) Brief hold so detector / depth have stable frames
                await self._hold_position(0.4, target_yaw)

            print(f"[Mission] Exploration done. "
                  f"Visited={len(self.visited)} Blocked={len(self.blocked)} "
                  f"Barrels={len(self.barrel_logger.get_barrels())}")

        except asyncio.CancelledError:
            print("[Mission] Cancelled")
        except Exception as e:
            print(f"[Mission] EXCEPTION: {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()
        finally:
            await self.shutdown()


# ==========================================================================
# ENTRY POINT
# ==========================================================================
async def main():
    mission = QualifierMission()
    task = asyncio.create_task(mission.run())
    try:
        await task
    except KeyboardInterrupt:
        print("\n[Main] KeyboardInterrupt -> stopping")
        mission.running = False
        await asyncio.gather(task, return_exceptions=True)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Main] interrupted at top level")
