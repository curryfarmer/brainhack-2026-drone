"""RoboVerse Qualifier 2026 — MVP mission runner.

Wires existing modules into a single autonomous attempt:
    - drone_control.Drone         : PX4 offboard NED commands
    - get_position_with_task      : background pose telemetry
    - depth_receiver.DepthReceiver: Gazebo depth stream
    - Detector.Detector           : threaded YOLO worker pool
    - AvoidancePlanner            : reactive deviation when blocked
    - coverage.generate_lawnmower : pre-baked boustrophedon sweep
    - detection_to_world          : bbox + depth + pose -> NED point
    - barrel_log.BarrelLog        : dedup, scoring, CSV persistence

Top-level architecture:

    supervisor()
        loop while wall-clock budget remains:
            try:
                run_attempt()
            except:
                log + continue (clock keeps ticking)

    run_attempt() spawns two cooperating asyncio tasks:
        mission_loop()   - flies the lawnmower waypoints with reactive deviation
        detection_loop() - pulls RGB frames, hands to Detector; callback projects
                           detections to world NED and logs them.

Manual control paths are deliberately absent — qualifier rules disqualify any
joystick/keyboard use during a scored run.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import time
import traceback
from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Optional

import numpy as np

from AvoidancePlanner import AvoidancePlanner
from Detector import Detector
from barrel_log import BarrelLog
from coverage import Waypoint, filter_unvisited, generate_lawnmower
from detection_to_world import Pose as ProjPose, detection_to_world

# Gazebo bindings + modules that depend on them are optional on dev machines
# without Gazebo Harmonic installed. Import lazily so unit tests of pure logic
# still work on macOS without `brew install gz-harmonic`.
try:
    from gz.msgs10.image_pb2 import Image as GzImage  # noqa: F401
    from gz.transport13 import Node as GzNode
    from depth_receiver import DepthReceiver
    _GZ_AVAILABLE = True
except ImportError:
    DepthReceiver = None  # type: ignore[assignment]
    GzNode = None  # type: ignore[assignment]
    _GZ_AVAILABLE = False

# MAVSDK-dependent modules — keep optional only if the user really wants to
# import this file outside the venv. In our venv these always succeed.
from drone_control import Drone
from get_position_with_task import SharedState, position_monitor_task


# ============================================================
# Config
# ============================================================
@dataclass
class MissionConfig:
    # PX4
    px4_address: str = "udpin://0.0.0.0:14540"

    # Arena (released 1 day before competition — edit before run)
    origin_north: float = 0.0
    origin_east: float = 0.0
    arena_north_m: float = 40.0
    arena_east_m: float = 40.0
    cruise_altitude_m: float = 2.5
    lane_spacing_m: float = 3.5
    along_axis: str = "north"

    # Flight
    cruise_speed_mps: float = 1.2
    waypoint_radius_m: float = 0.8
    loop_hz: float = 20.0
    wall_clock_budget_s: float = 600.0  # 10 min judge clock

    # Sensors / topics
    depth_topic: str = "/depth_camera"
    rgb_topic: str = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"
    camera_pitch_deg: float = 0.0

    # Camera intrinsics (640x480, fx=fy=433, principal point centre)
    K: np.ndarray = field(
        default_factory=lambda: np.array(
            [[433.0, 0.0, 320.0],
             [0.0, 433.0, 240.0],
             [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
    )
    img_width: int = 640
    img_height: int = 480

    # Detection
    yolo_weights: str = "yolov10n.pt"          # swap for custom barrel weights
    yolo_conf: float = 0.4
    yolo_device: str = "cpu"                   # "cuda" on competition rig
    yolo_workers: int = 1
    yolo_save_dir: str = "./runs/detections"
    yolo_display: bool = False                 # off for headless competition runs
    detection_class_map: Dict[str, str] = field(default_factory=lambda: {
        # Map YOLO class name -> canonical barrel class.
        # Update after training (e.g. {"yellow": "yellow_barrel", "red": "red_barrel"}).
        "yellow_barrel": "yellow_barrel",
        "red_barrel": "red_barrel",
    })

    # Logging
    run_dir: str = "./runs"

    @classmethod
    def from_json(cls, path: str) -> "MissionConfig":
        with open(path, "r") as f:
            data = json.load(f)
        if "K" in data:
            data["K"] = np.array(data["K"], dtype=np.float64)
        return cls(**data)


# ============================================================
# Shared state container
# ============================================================
@dataclass
class MissionState:
    pose_state: SharedState
    barrel_log: BarrelLog
    depth_rx: DepthReceiver
    detector: Optional[Detector]
    cfg: MissionConfig
    stop_event: asyncio.Event
    loop: asyncio.AbstractEventLoop

    def pose_ned_yaw_rad(self) -> Optional[ProjPose]:
        if self.pose_state.latest_position is None or self.pose_state.latest_yaw is None:
            return None
        p = self.pose_state.latest_position
        return ProjPose(
            north=p.north_m,
            east=p.east_m,
            down=p.down_m,
            yaw_rad=math.radians(self.pose_state.latest_yaw),
        )


# ============================================================
# Gazebo RGB receiver (mirrors DepthReceiver pattern)
# ============================================================
class RgbReceiver:
    """Pulls 8UC3 RGB frames off a gz transport topic; thread-safe latest-frame."""

    def __init__(self, topic: str):
        if not _GZ_AVAILABLE:
            raise RuntimeError("gz.transport13 not installed — install Gazebo Harmonic")
        self.node = GzNode()
        self._frame: Optional[np.ndarray] = None
        import threading
        self._lock = threading.Lock()
        self.node.subscribe(GzImage, topic, self._callback)

    def _callback(self, msg) -> None:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            frame = data.reshape((msg.height, msg.width, 3))
        except ValueError:
            return  # mismatched buffer
        with self._lock:
            self._frame = frame

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._frame is None else self._frame.copy()


# ============================================================
# Detection callback factory
# ============================================================
def make_detection_callback(state: MissionState):
    """Build a Detector callback that projects detections into world NED and logs them."""
    cfg = state.cfg

    def cb(detections: List[Dict[str, Any]], annotated, context: Dict[str, Any]) -> None:
        depth = context.get("depth")
        pose = context.get("pose")
        ts = context.get("ts", time.time())
        if depth is None or pose is None:
            return

        for det in detections:
            mapped = cfg.detection_class_map.get(det["class_name"])
            if mapped is None:
                continue  # not a barrel class we care about
            bbox = tuple(det["bbox"])
            wd = detection_to_world(
                bbox_xyxy=bbox,
                depth_frame=depth,
                pose=pose,
                K=cfg.K,
                class_name=mapped,
                confidence=det["confidence"],
                ts=ts,
                camera_pitch_deg=cfg.camera_pitch_deg,
            )
            if wd is None:
                continue
            entry, is_new = state.barrel_log.add(
                wd.class_name, wd.north, wd.east, wd.down, wd.confidence, ts=wd.ts,
            )
            if is_new:
                score = BarrelLog.SCORES.get(wd.class_name, 0)
                print(
                    f"[barrel] NEW {wd.class_name} #{entry.barrel_id} "
                    f"@ N={wd.north:5.2f} E={wd.east:5.2f} D={wd.down:5.2f} "
                    f"conf={wd.confidence:.2f}  +{score}pt  total={state.barrel_log.score()}"
                )
    return cb


# ============================================================
# Detection loop
# ============================================================
async def detection_loop(state: MissionState, rgb_rx: Optional[RgbReceiver]) -> None:
    """Pull RGB frames, attach pose + depth context, submit to Detector."""
    if state.detector is None or rgb_rx is None:
        print("[detect] disabled (no detector or no RGB receiver)")
        return

    period = 1.0 / 5.0  # 5 Hz submission rate; detector worker decides actual rate
    while not state.stop_event.is_set():
        frame = rgb_rx.get_frame()
        pose = state.pose_ned_yaw_rad()
        depth = state.depth_rx.get_frame()
        if frame is not None and pose is not None and depth is not None:
            state.detector.submit_image(
                frame,
                context={"depth": depth, "pose": pose, "ts": time.time()},
            )
        await asyncio.sleep(period)


# ============================================================
# Mission loop — boustrophedon waypoints + reactive deviation
# ============================================================
async def _await_first_pose(state: MissionState, timeout: float = 10.0) -> None:
    start = time.monotonic()
    while state.pose_ned_yaw_rad() is None:
        if state.stop_event.is_set():
            return
        if time.monotonic() - start > timeout:
            raise TimeoutError("No telemetry pose received within 10 s")
        await asyncio.sleep(0.1)


async def _go_to_waypoint(
    drone: Drone,
    state: MissionState,
    planner: AvoidancePlanner,
    wp: Waypoint,
    deadline: float,
) -> bool:
    """Drive toward wp until within waypoint_radius. Reactive deviation when blocked.

    Returns True on arrival, False on deadline / stop.
    """
    cfg = state.cfg
    period = 1.0 / cfg.loop_hz

    while time.monotonic() < deadline and not state.stop_event.is_set():
        pose = state.pose_ned_yaw_rad()
        if pose is None:
            await asyncio.sleep(period)
            continue

        dn = wp.north - pose.north
        de = wp.east - pose.east
        dist = math.hypot(dn, de)
        if dist < cfg.waypoint_radius_m:
            return True

        # Check depth for obstacle
        depth = state.depth_rx.get_frame()
        blocked = False
        clear_n, clear_e = wp.north, wp.east

        if depth is not None:
            try:
                pose_dict = {
                    "north": pose.north, "east": pose.east, "down": pose.down,
                    "yaw": pose.yaw_rad,
                }
                _, _, _, info = planner.compute_position_ned(depth, pose_dict, step_size=1.5)
                blocked = info["blocked"]
                if blocked:
                    # Reactive sidestep — head toward planner's "best clear" cell instead.
                    clear_n = info["target_ned"]["north"]
                    clear_e = info["target_ned"]["east"]
            except Exception as e:
                # Planner exceptions shouldn't kill flight
                print(f"[avoid] planner error: {e}")

        # Step toward the (possibly deviated) target
        target_n = clear_n if blocked else wp.north
        target_e = clear_e if blocked else wp.east
        target_d = wp.down

        # Yaw: along travel direction unless waypoint forces it
        travel_yaw_deg = wp.yaw_deg
        if blocked:
            # face the cleared direction so the camera sees the right thing
            travel_yaw_deg = math.degrees(math.atan2(target_e - pose.east, target_n - pose.north))

        await drone.send_position_setpoint(
            north=target_n, east=target_e, down=target_d, yaw_deg=travel_yaw_deg,
        )
        await asyncio.sleep(period)

    return False


async def mission_loop(drone: Drone, state: MissionState) -> None:
    cfg = state.cfg
    deadline = state.loop.time() + cfg.wall_clock_budget_s

    print(f"[mission] connecting to {cfg.px4_address}")
    await drone.connect()
    await asyncio.sleep(2)

    # Background telemetry
    monitor = state.loop.create_task(
        position_monitor_task(drone, state.pose_state, state.stop_event)
    )

    try:
        print("[mission] arm + takeoff")
        await drone.arm_and_takeoff()
        await _await_first_pose(state, timeout=10.0)

        pose0 = state.pose_ned_yaw_rad()
        print(f"[mission] takeoff pose: N={pose0.north:.2f} E={pose0.east:.2f} D={pose0.down:.2f}")

        waypoints = generate_lawnmower(
            origin_north=cfg.origin_north,
            origin_east=cfg.origin_east,
            width_north=cfg.arena_north_m,
            width_east=cfg.arena_east_m,
            altitude=cfg.cruise_altitude_m,
            lane_spacing=cfg.lane_spacing_m,
            along_axis=cfg.along_axis,
        )
        waypoints = filter_unvisited(waypoints, pose0.north, pose0.east, visited_radius=1.5)
        print(f"[mission] {len(waypoints)} waypoints queued")

        planner = AvoidancePlanner(
            K=cfg.K,
            width=cfg.img_width,
            height=cfg.img_height,
            safe_distance=3.0,
            critical_distance=1.2,
        )

        for i, wp in enumerate(waypoints):
            if time.monotonic() >= deadline or state.stop_event.is_set():
                break
            print(f"[mission] wp {i+1}/{len(waypoints)}  N={wp.north:.1f} E={wp.east:.1f} yaw={wp.yaw_deg:.0f}")
            reached = await _go_to_waypoint(drone, state, planner, wp, deadline=deadline)
            if not reached:
                print(f"[mission] wp {i+1} timed out / aborted")

        print("[mission] coverage finished, landing")
        await drone.land()

    except Exception:
        # On unexpected error, try to bring the drone down before re-raising so
        # the supervisor doesn't restart with a still-airborne vehicle.
        print("[mission] crash — attempting emergency land")
        try:
            await drone.land()
        except Exception as land_err:
            print(f"[mission] emergency land failed: {land_err}")
        raise
    finally:
        state.stop_event.set()
        monitor.cancel()
        try:
            await monitor
        except asyncio.CancelledError:
            pass


# ============================================================
# Supervisor — restart on crash within wall-clock budget
# ============================================================
async def supervisor(cfg: MissionConfig) -> None:
    os.makedirs(cfg.run_dir, exist_ok=True)
    run_id = time.strftime("%Y%m%d_%H%M%S")
    attempt_dir = os.path.join(cfg.run_dir, run_id)
    os.makedirs(attempt_dir, exist_ok=True)
    print(f"[supervisor] run dir: {attempt_dir}")

    loop = asyncio.get_running_loop()
    deadline = loop.time() + cfg.wall_clock_budget_s

    # Long-lived objects shared across attempts (so a crash doesn't wipe barrel log)
    barrel_log = BarrelLog(os.path.join(attempt_dir, "barrels.csv"), dedup_radius=2.0)
    if not _GZ_AVAILABLE:
        raise RuntimeError(
            "gz.transport13 + gz.msgs10 not installed — install Gazebo Harmonic "
            "(brew install gz-harmonic / apt install gz-harmonic) and re-run."
        )
    depth_rx = DepthReceiver(cfg.depth_topic)
    rgb_rx: Optional[RgbReceiver] = None
    detector: Optional[Detector] = None
    if _GZ_AVAILABLE:
        try:
            rgb_rx = RgbReceiver(cfg.rgb_topic)
        except Exception as e:
            print(f"[supervisor] RGB receiver init failed: {e}")
    else:
        print("[supervisor] gz transport unavailable — RGB pipeline disabled")

    pose_state = SharedState()

    state = MissionState(
        pose_state=pose_state,
        barrel_log=barrel_log,
        depth_rx=depth_rx,
        detector=None,
        cfg=cfg,
        stop_event=asyncio.Event(),
        loop=loop,
    )

    # Detector is built once and reused — model load is expensive.
    # Skip entirely when --no-detector was passed (yolo_weights cleared).
    if cfg.yolo_weights:
        try:
            detector = Detector(
                model_path=cfg.yolo_weights,
                confidence_threshold=cfg.yolo_conf,
                num_workers=cfg.yolo_workers,
                device=cfg.yolo_device,
                save_dir=os.path.join(attempt_dir, "detections"),
                enable_display=cfg.yolo_display,
                callback=make_detection_callback(state),
            )
            state.detector = detector
        except Exception as e:
            print(f"[supervisor] Detector init failed: {e}")
    else:
        print("[supervisor] detector disabled (no weights configured)")

    attempt = 0
    try:
        while loop.time() < deadline:
            attempt += 1
            remaining = deadline - loop.time()
            # dataclasses.replace preserves the numpy K matrix as-is (unlike
            # dict-roundtrip which would silently shred it).
            state.cfg = replace(cfg, wall_clock_budget_s=remaining)
            # Reuse the existing Event rather than reassigning. detection_loop
            # reads state.stop_event.is_set() lazily, so reassignment is
            # technically safe — but .clear() is unambiguous.
            state.stop_event.clear()

            print(f"\n========== attempt {attempt}  remaining={remaining:.1f}s ==========")
            drone = Drone()
            mission_task = loop.create_task(mission_loop(drone, state))
            det_task = loop.create_task(detection_loop(state, rgb_rx))

            try:
                await mission_task
            except asyncio.CancelledError:
                raise
            except Exception:
                print(f"[supervisor] attempt {attempt} crashed:")
                traceback.print_exc()
            finally:
                state.stop_event.set()
                det_task.cancel()
                try:
                    await det_task
                except asyncio.CancelledError:
                    pass

            print(f"[supervisor] attempt {attempt} done. "
                  f"score={barrel_log.score()} counts={barrel_log.count_by_class()}")

            # Tiny breather between attempts — don't hammer the connection
            if loop.time() < deadline:
                await asyncio.sleep(2.0)

    finally:
        if detector is not None:
            detector.stop()
        print(f"\n========== run complete ==========")
        print(f"final score: {barrel_log.score()}")
        print(f"counts:      {barrel_log.count_by_class()}")
        print(f"csv:         {barrel_log.csv_path}")


# ============================================================
# CLI entry
# ============================================================
def parse_args() -> MissionConfig:
    p = argparse.ArgumentParser(description="RoboVerse Qualifier MVP runner")
    p.add_argument("--config", help="Path to JSON config (overrides defaults)")
    p.add_argument("--weights", help="YOLO weights path (overrides config)")
    p.add_argument("--device", help="cpu | cuda | mps")
    p.add_argument("--altitude", type=float, help="Cruise altitude (m)")
    p.add_argument("--budget", type=float, help="Wall clock budget (s)")
    p.add_argument("--no-detector", action="store_true", help="Disable YOLO (flight-only test)")
    p.add_argument("--display", action="store_true", help="Show detection window")
    args = p.parse_args()

    cfg = MissionConfig.from_json(args.config) if args.config else MissionConfig()
    if args.weights:  cfg.yolo_weights = args.weights
    if args.device:   cfg.yolo_device = args.device
    if args.altitude: cfg.cruise_altitude_m = args.altitude
    if args.budget:   cfg.wall_clock_budget_s = args.budget
    if args.display:  cfg.yolo_display = True
    if args.no_detector:
        cfg.yolo_weights = ""  # signal to skip detector — handled in supervisor below
    return cfg


async def _amain():
    cfg = parse_args()
    await supervisor(cfg)


if __name__ == "__main__":
    asyncio.run(_amain())
