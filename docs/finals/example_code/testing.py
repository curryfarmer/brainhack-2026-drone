"""
three_drone_takeoff.py

Discovers three Hula drones on the local WiFi network, connects to each,
and commands all three to take off and hover at 1.1 m altitude concurrently.

Architecture
------------
  - Dola     : UDP broadcast listener that discovers drones by plane_id
  - pyhulax  : DroneAPI for per-drone connection
  - mavsdk   : PX4 offboard velocity control (VelocityNedYaw)
  - ROS2     : UWB positioning (one subscriber node per drone, each on its
                own topic)

Coordinate frame used throughout
---------------------------------
  NED (North-East-Down), consistent with PX4 / mavsdk offboard API.
  Altitude = 1.1 m  →  target_d = -1.1  (Down is negative when above ground)

Usage
-----
  python three_drone_takeoff.py

  The script will wait up to 10 s for all three drones to appear on the
  network, then proceed automatically.  It prompts once per drone before
  arming so you can abort safely.
"""

import asyncio
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw
from pyhulax import DroneAPI

from dola import Dola

# ============================================================
# Configuration
# ============================================================

# Plane IDs to discover (must match what the drones broadcast)
PLANE_IDS = [1, 2, 3]

# Network discovery timeout (seconds)
DISCOVER_TIMEOUT = 10

# Serial port template — edit if your board uses different ports
# e.g. /dev/ttyS0, /dev/ttyUSB0, /dev/ttyACM0
SERIAL_PORT_TEMPLATE = "serial:///dev/ttyS{port}:921600"
DRONE_SERIAL_PORTS = {
    1: SERIAL_PORT_TEMPLATE.format(port=6),
    2: SERIAL_PORT_TEMPLATE.format(port=7),
    3: SERIAL_PORT_TEMPLATE.format(port=8),
}

# UWB ROS2 topic template — one topic per drone
UWB_TOPIC_TEMPLATE = "/drone_{id}/uwb_tag"

# Flight parameters
TAKEOFF_HEIGHT_M = 1.1          # metres above ground
TARGET_D = -TAKEOFF_HEIGHT_M    # NED Down coordinate (negative = up)

# Velocity controller gains
KP_XY = 0.10
KP_Z  = 0.15

# Velocity limits (m/s)
MAX_VEL_XY = 0.50
MAX_VEL_Z  = 0.40

# Arrival thresholds (m)
XY_THRESHOLD = 0.10
Z_THRESHOLD  = 0.08

# Deadband to kill residual corrections
HOVER_DEADBAND = 0.03

# Offboard warm-up setpoints before starting offboard mode
OFFBOARD_WARMUP_COUNT = 20

# ============================================================
# Per-drone UWB ROS2 subscriber node
# ============================================================

class UwbNode(Node):
    """
    Subscribes to a single drone's UWB PoseStamped topic and
    exposes the latest N/E position in a thread-safe way.
    """

    def __init__(self, drone_id: int):
        node_name = f"uwb_listener_drone_{drone_id}"
        super().__init__(node_name)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        topic = UWB_TOPIC_TEMPLATE.format(id=drone_id)
        self.subscription = self.create_subscription(
            PoseStamped,
            topic,
            self._cb,
            qos
        )

        self.n     = 0.0
        self.e     = 0.0
        self.ready = False
        print(f"[Drone {drone_id}] UWB subscriber → {topic}")

    def _cb(self, msg: PoseStamped):
        self.n     = msg.pose.position.y
        self.e     = msg.pose.position.x
        self.ready = True

    def get_position(self):
        return self.n, self.e, self.ready


# ============================================================
# ROS2 multi-node spin thread
# ============================================================

_uwb_nodes: dict[int, UwbNode] = {}

def start_ros2(drone_ids: list[int]):
    """
    Initialise ROS2 once, create one UwbNode per drone, and
    run a MultiThreadedExecutor in a daemon thread.
    """
    global _uwb_nodes

    if not rclpy.ok():
        rclpy.init(args=None)

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()

    for did in drone_ids:
        node = UwbNode(did)
        _uwb_nodes[did] = node
        executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    print("ROS2 executor thread started.")


def get_uwb(drone_id: int):
    node = _uwb_nodes.get(drone_id)
    if node is None:
        return 0.0, 0.0, False
    return node.get_position()


# ============================================================
# Per-drone flight coroutine
# ============================================================

async def fly_drone(drone_id: int, ip: str):
    """
    Full lifecycle for one drone:
      connect → arm → warm-up → offboard → climb to 1.1 m → hover → land
    """
    tag = f"[Drone {drone_id} | {ip}]"

    # ----------------------------------------------------------
    # Wait for UWB data
    # ----------------------------------------------------------
    print(f"{tag} Waiting for UWB data…")
    while True:
        n, e, ready = get_uwb(drone_id)
        if ready:
            break
        await asyncio.sleep(0.2)
    print(f"{tag} UWB ready  N={n:.2f}  E={e:.2f}")

    # ----------------------------------------------------------
    # Connect to PX4 via MAVSDK
    # ----------------------------------------------------------
    serial = DRONE_SERIAL_PORTS[drone_id]
    drone = System()
    print(f"{tag} Connecting via {serial} …")
    await drone.connect(system_address=serial)

    # Telemetry state
    current_yaw  = 0.0
    current_d    = 0.0
    height_ready = False
    battery_pct  = 0.0

    # Start background telemetry tasks
    async def _attitude():
        nonlocal current_yaw
        async for att in drone.telemetry.attitude_euler():
            current_yaw = att.yaw_deg

    async def _position():
        nonlocal current_d, height_ready
        async for pv in drone.telemetry.position_velocity_ned():
            current_d    = pv.position.down_m
            height_ready = True

    async def _battery():
        nonlocal battery_pct
        async for bat in drone.telemetry.battery():
            battery_pct = bat.remaining_percent

    asyncio.create_task(_attitude())
    asyncio.create_task(_position())
    asyncio.create_task(_battery())

    # Wait for local position estimate
    print(f"{tag} Waiting for local position estimate…")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print(f"{tag} Local position OK")
            break

    # Wait for height telemetry
    while not height_ready:
        await asyncio.sleep(0.1)

    # Lock takeoff yaw
    takeoff_yaw = current_yaw
    print(f"{tag} Takeoff yaw locked at {takeoff_yaw:.1f}°")

    # ----------------------------------------------------------
    # Helper: send velocity setpoint
    # ----------------------------------------------------------
    async def send_vel(vn, ve, vd):
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vn, ve, vd, takeoff_yaw)
        )

    # ----------------------------------------------------------
    # Confirm before arming
    # ----------------------------------------------------------
    n, e, _ = get_uwb(drone_id)
    print(f"{tag} Position  N={n:.2f}  E={e:.2f}  D={current_d:.2f}")
    print(f"{tag} Battery   {battery_pct*100:.0f}%")

    loop = asyncio.get_running_loop()
    answer = await loop.run_in_executor(
        None,
        input,
        f"{tag} Proceed with takeoff? (y/n): "
    )
    if answer.strip().lower() not in ("y", "yes"):
        print(f"{tag} Aborted by user.")
        return

    # ----------------------------------------------------------
    # Set takeoff altitude and arm
    # ----------------------------------------------------------
    await drone.action.set_takeoff_altitude(TAKEOFF_HEIGHT_M)
    await asyncio.sleep(0.5)

    print(f"{tag} Arming…")
    await drone.action.arm()

    # ----------------------------------------------------------
    # Offboard warm-up: send zero setpoints first
    # ----------------------------------------------------------
    print(f"{tag} Offboard warm-up…")
    for _ in range(OFFBOARD_WARMUP_COUNT):
        await send_vel(0.0, 0.0, 0.0)
        await asyncio.sleep(0.05)

    print(f"{tag} Starting offboard mode…")
    await drone.offboard.start()

    # ----------------------------------------------------------
    # Climb to TARGET_D (= -1.1 m in NED)
    # ----------------------------------------------------------
    print(f"{tag} Climbing to {TAKEOFF_HEIGHT_M} m…")

    # Record home X/Y to hold position while climbing
    home_n, home_e, _ = get_uwb(drone_id)

    while True:
        cur_n, cur_e, uwb_ok = get_uwb(drone_id)
        cur_d = current_d

        if not uwb_ok or not height_ready:
            await send_vel(0.0, 0.0, 0.0)
            await asyncio.sleep(0.1)
            continue

        err_n = home_n - cur_n
        err_e = home_e - cur_e
        err_d = TARGET_D - cur_d

        # Proportional velocities
        vn = KP_XY * err_n
        ve = KP_XY * err_e
        vd = KP_Z  * err_d

        # Clamp horizontal
        h_speed = (vn**2 + ve**2) ** 0.5
        if h_speed > MAX_VEL_XY:
            scale = MAX_VEL_XY / h_speed
            vn *= scale
            ve *= scale

        # Clamp vertical
        vd = max(-MAX_VEL_Z, min(MAX_VEL_Z, vd))

        # Deadband
        if abs(err_n) < HOVER_DEADBAND: vn = 0.0
        if abs(err_e) < HOVER_DEADBAND: ve = 0.0
        if abs(err_d) < HOVER_DEADBAND: vd = 0.0

        print(
            f"{tag} "
            f"N={cur_n:.2f}(err={err_n:.2f}) "
            f"E={cur_e:.2f}(err={err_e:.2f}) "
            f"D={cur_d:.2f}(err={err_d:.2f}) "
            f"vn={vn:.2f} ve={ve:.2f} vd={vd:.2f}"
        )

        await send_vel(vn, ve, vd)

        # Check arrival (horizontal + vertical)
        if abs(err_n) < XY_THRESHOLD and abs(err_e) < XY_THRESHOLD and abs(err_d) < Z_THRESHOLD:
            await send_vel(0.0, 0.0, 0.0)
            print(f"{tag} ✓ Reached {TAKEOFF_HEIGHT_M} m altitude.")
            break

        await asyncio.sleep(0.1)

    # ----------------------------------------------------------
    # Hover for 5 seconds at altitude, then land
    # ----------------------------------------------------------
    HOVER_SECONDS = 5.0
    print(f"{tag} Hovering for {HOVER_SECONDS:.0f} s…")

    hover_n, hover_e, _ = get_uwb(drone_id)
    hover_d = current_d
    end_t = asyncio.get_running_loop().time() + HOVER_SECONDS

    while asyncio.get_running_loop().time() < end_t:
        cur_n, cur_e, _ = get_uwb(drone_id)
        cur_d = current_d

        err_n = hover_n - cur_n
        err_e = hover_e - cur_e
        err_d = hover_d - cur_d

        vn = KP_XY * err_n
        ve = KP_XY * err_e
        vd = KP_Z  * err_d

        h_speed = (vn**2 + ve**2) ** 0.5
        if h_speed > MAX_VEL_XY:
            scale = MAX_VEL_XY / h_speed
            vn *= scale
            ve *= scale

        vd = max(-MAX_VEL_Z, min(MAX_VEL_Z, vd))

        if abs(err_n) < HOVER_DEADBAND: vn = 0.0
        if abs(err_e) < HOVER_DEADBAND: ve = 0.0
        if abs(err_d) < HOVER_DEADBAND: vd = 0.0

        await send_vel(vn, ve, vd)
        await asyncio.sleep(0.1)

    # ----------------------------------------------------------
    # Stop offboard, then land
    # ----------------------------------------------------------
    print(f"{tag} Stopping offboard…")
    try:
        await drone.offboard.stop()
    except Exception as exc:
        print(f"{tag} offboard.stop() error: {exc}")

    print(f"{tag} Landing…")
    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
        await asyncio.sleep(0.5)

    print(f"{tag} Landed.")

    try:
        await drone.action.disarm()
    except Exception:
        pass

    print(f"{tag} Mission complete ✓")


# ============================================================
# Main entry point
# ============================================================

async def main():

    # ----------------------------------------------------------
    # Step 1 — Discover all three drones via Dola
    # ----------------------------------------------------------
    print("=" * 60)
    print(f"Discovering drones {PLANE_IDS} (timeout={DISCOVER_TIMEOUT}s)…")
    print("=" * 60)

    dola = Dola()
    dola.start()
    try:
        ip_map = dola.get_ips_by_plane_ids(
            PLANE_IDS,
            listen_seconds=DISCOVER_TIMEOUT
        )
    finally:
        dola.stop()

    # Warn about missing drones but continue with whatever was found
    missing = [pid for pid, ip in ip_map.items() if not ip]
    if missing:
        print(f"WARNING: Could not find drones with plane_id(s): {missing}. Continuing with available drones.")

    available = {pid: ip for pid, ip in ip_map.items() if ip}
    if not available:
        print("ERROR: No drones found at all. Check that at least one drone is powered on and on the same WiFi.")
        sys.exit(1)

    print("\nDiscovered drones:")
    for pid, ip in sorted(available.items()):
        print(f"  Plane {pid}  →  {ip}")
    print()

    # ----------------------------------------------------------
    # Step 2 — Connect pyhulax video streams (optional preview)
    # ----------------------------------------------------------
    hula_drones = {}
    for pid, ip in available.items():
        try:
            d = DroneAPI()
            d.connect(ip)
            hula_drones[pid] = d
            print(f"[Drone {pid}] pyhulax connected to {ip}")
        except Exception as exc:
            print(f"[Drone {pid}] pyhulax connect failed: {exc}")
            # Non-fatal — flight control goes through mavsdk directly

    # ----------------------------------------------------------
    # Step 3 — Start ROS2 UWB subscribers (one per drone)
    # ----------------------------------------------------------
    start_ros2(list(available.keys()))
    await asyncio.sleep(1.0)  # Give ROS2 a moment to receive first packets

    # ----------------------------------------------------------
    # Step 4 — Launch takeoff coroutines concurrently for all available drones
    # ----------------------------------------------------------
    print(f"\nLaunching concurrent takeoff for {len(available)} drone(s)...\n")

    tasks = [
        asyncio.create_task(
            fly_drone(pid, available[pid]),
            name=f"drone_{pid}"
        )
        for pid in available
    ]

    results = await asyncio.gather(*tasks, return_exceptions=True)

    # Report any failures
    for pid, result in zip(available.keys(), results):
        if isinstance(result, Exception):
            print(f"[Drone {pid}] FAILED: {result}")

    # ----------------------------------------------------------
    # Step 5 — ROS2 cleanup
    # ----------------------------------------------------------
    try:
        for node in _uwb_nodes.values():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as exc:
        print(f"ROS2 shutdown error: {exc}")

    print("\nAll done.")


if __name__ == "__main__":
    asyncio.run(main())