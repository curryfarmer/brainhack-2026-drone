import asyncio
import math
import threading
from collections import namedtuple

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw

# ROS2 Imports just for grabbing UWB data
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped

# Takeoff height
TAKEOFF_HEIGHT = 0.8

# Navigation gains - Determine speed
KP_XY = 0.1
KP_Z = 0.1

# Navigation velocity limits
MAX_VEL_XY = 0.5
MAX_VEL_Z = 0.3

# Hover velocity limits
MAX_HOVER_XY = 0.15
MAX_HOVER_Z = 0.10

# Position thresholds
WAYPOINT_THRESHOLD = 0.20
N_THRESHOLD = 0.1
E_THRESHOLD = 0.1
D_THRESHOLD = 0.1
KP_SCALE = 0.2

HOVER_DEADBAND = 0.03

current_n = 0.0
current_e = 0.0
current_d = 0.0

current_yaw = 0.0
takeoff_yaw = 0.0
height_telemetry_ready = False

# ==========================================
# ROS2 UWB Integration
# ==========================================
class UwbNode(Node):
    def __init__(self):
        super().__init__('uwb_listener_node')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.subscription = self.create_subscription(
            PoseStamped,
            'uwb_tag',
            self.uwb_callback,
            qos_profile)
        self.n = 0.0
        self.e = 0.0
        self.ready = False

    def uwb_callback(self, msg):
        # Update internal state when a new PoseStamped message arrives
        self.n = msg.pose.position.y
        self.e = msg.pose.position.x
        self.ready = True

uwb_node = None

def get_uwb_position():
    """Thread-safe getter to access UWB data from the main asyncio loop."""
    if uwb_node is not None:
        return (uwb_node.n, uwb_node.e, uwb_node.ready)
    return (0.0, 0.0, False)

def start_ros2_thread():
    """Initializes ROS2 and starts the spin loop in a daemon thread."""
    global uwb_node
    if not rclpy.ok():
        rclpy.init(args=None)
        
    uwb_node = UwbNode()
    
    # Run rclpy.spin in a separate daemon thread so it doesn't block asyncio
    ros_thread = threading.Thread(target=rclpy.spin, args=(uwb_node,), daemon=True)
    ros_thread.start()
    print("ROS2 UWB subscriber thread started.")
    return uwb_node
# ==========================================

### function to get yaw heading from PX4 telemetry
def get_current_yaw_deg():
    return current_yaw

def get_current_height():
    return current_d

async def attitude_task(drone):
    global current_yaw
    async for attitude in drone.telemetry.attitude_euler():
        current_yaw = attitude.yaw_deg

async def battery_task(drone):
    global battery_remain
    async for battery in drone.telemetry.battery():
        battery_remain = battery.remaining_percent

async def pos_task(drone):
    global current_d
    global height_telemetry_ready

    async for pos in drone.telemetry.position_velocity_ned():
        current_d = pos.position.down_m
        height_telemetry_ready = True

### main loop starts here

async def run():
    global takeoff_yaw  # Fix 3: Ensure we update the global takeoff_yaw variable


    # Start UWB subscriber
    start_ros2_thread()   
    await asyncio.sleep(1.0)
    n,e,state = get_uwb_position()
    while state == False:  # Wait until UWB data is ready
        print("Waiting for UWB data...")
        await asyncio.sleep(0.5)
        uwb_pos = get_uwb_position()

    print(f"Initial UWB position -> x: {n:.2f}, y: {e:.2f}, ready: {state}")

    # initiqlize drone connection and telemetry
    drone = System()
    print("Connecting...")
    await drone.connect(system_address="serial:///dev/ttyS6:921600")
    print("Waiting for yaw estimate...")
    asyncio.create_task(attitude_task(drone))
    asyncio.create_task(pos_task(drone))
    asyncio.create_task(battery_task(drone))

    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("Local position estimate OK")
            break

    async def send_velocity(vn, ve, vd):
        global takeoff_yaw
        await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, takeoff_yaw))

    async def fly_to_position_velocity(target_n, target_e, target_d, ignore_height=True, n_threshold=N_THRESHOLD, e_threshold=E_THRESHOLD, d_threshold=D_THRESHOLD,scale=KP_SCALE,test=False):
        """
        Fly to a target NED position using velocity control.
        """
        global height_telemetry_ready
        print(f"Target N={target_n:.2f} E={target_e:.2f} D={target_d:.2f}")

        while True:
            current_n, current_e, state = get_uwb_position()
            current_d = get_current_height()
                
            if state == False:
                print("UWB data not ready, cannot navigate.")
                if test == False:
                    send_velocity(0.0, 0.0, 0.0)  # Stop movement if UWB data is not ready
                await asyncio.sleep(0.5)
                continue

            if height_telemetry_ready == False:
                if test == False:
                    send_velocity(0.0, 0.0, 0.0)  # Stop movement if UWB data is not ready
                print("height data not ready, cannot navigate.")
                continue

            current_d = get_current_height()

            err_n = target_n - current_n
            err_e = target_e - current_e
            err_d = target_d - current_d

            dist = math.sqrt(err_n**2 + err_e**2 + err_d**2)
            vn = 0.0
            ve = 0.0
            vd = 0.0

            if abs(err_n) < n_threshold:
                vn = 0.0
            if abs(err_e) < e_threshold:
                ve = 0.0
            if abs(err_d) < d_threshold:
                vd = 0.0
            
            if ignore_height:
                if abs(err_n) < n_threshold and abs(err_e) < e_threshold:
                    await send_velocity(0.0, 0.0, 0.0)
                    print("Waypoint reached")
                    return
            else:
                if abs(err_n) < n_threshold and abs(err_e) < e_threshold and abs(err_d) < d_threshold:
                    await send_velocity(0.0, 0.0, 0.0)
                    print("Waypoint reached")
                    return
                
            
            if abs(err_n) >= n_threshold:           
                vn = KP_XY * err_n
            if abs(err_e) >= e_threshold:           
                ve = KP_XY * err_e
            if abs(err_d) >= d_threshold:           
                vd = KP_Z * err_d

            horizontal_speed = math.sqrt(vn**2 + ve**2)

            if horizontal_speed > MAX_VEL_XY:
                scale = MAX_VEL_XY / horizontal_speed
                vn *= scale
                ve *= scale

            # Vertical controller
            if abs(vd) >= MAX_VEL_Z:
                if vd > 0:
                    vd = MAX_VEL_Z
                else:
                    if vd < 0:
                        vd = -MAX_VEL_Z

            if ignore_height:
                vd = 0

            print(
                f"N={current_n:.2f} / error={err_n:.2f} /vn={vn:.2f} "
                f"E={current_e:.2f} / error={err_e:.2f} /ve={ve:.2f} "
                f"D={current_d:.2f} / error={err_d:.2f} /vd={vd:.2f} "
                f"Dist={dist:.2f}"
            )
            if test == False:
                await send_velocity(vn, ve, vd)
            await asyncio.sleep(0.1)

    async def hover(seconds,ignore_height=False):
        """
        Hover using position feedback.
        Records current position and actively
        corrects drift for the hover duration.
        """

        hover_n, hover_e, state = get_uwb_position()
        hover_d = get_current_height()
        
        print(F"Hover lock: at N={hover_n:.2f} E={hover_e:.2f} D={hover_d:.2f}")
        end_time = asyncio.get_running_loop().time() + seconds

        while asyncio.get_running_loop().time() < end_time:
            current_n,current_e,state = get_uwb_position()
            current_d = get_current_height()

            err_n = hover_n - current_n
            err_e = hover_e - current_e
            err_d = hover_d - current_d

            # Horizontal correction
            vn = KP_XY * err_n
            ve = KP_XY * err_e
            vd = KP_Z * err_d
            horizontal_speed = math.sqrt(vn**2 + ve**2)

            if horizontal_speed > MAX_HOVER_XY:
                scale = MAX_HOVER_XY / horizontal_speed
                vn *= scale
                ve *= scale

            # Vertical correction
            if abs(vd) >= MAX_VEL_Z:
                if vd > 0:
                    vd = MAX_VEL_Z
                else:
                    if vd < 0:
                        vd = -MAX_VEL_Z

            # Deadband
            if abs(err_n) < HOVER_DEADBAND:
                vn = 0.0
            if abs(err_e) < HOVER_DEADBAND:
                ve = 0.0
            if abs(err_d) < HOVER_DEADBAND:
                vd = 0.0

            if ignore_height:
                vd = 0.0
                
            print(f"Hover error N={err_n:.2f} / {vn:.2f} E={err_e:.2f} / {ve:.2f} D={err_d:.2f} / {vd:.2f} ")
            await send_velocity(vn, ve, vd)
            await asyncio.sleep(0.1)

    # main drone navigation starts here
    try:
        home_n, home_e, state = get_uwb_position()
        while state == False:  # Wait until UWB data is ready
            print("Waiting for UWB data...")
            await asyncio.sleep(0.5)
            home_n, home_e, state = get_uwb_position()
        current_d = get_current_height()

        # Lock takeoff yaw
        takeoff_yaw = get_current_yaw_deg()
        print(f"Takeoff yaw locked at {takeoff_yaw:.1f} deg")

        target_altitude_m = TAKEOFF_HEIGHT
        print(f"Setting takeoff altitude to {target_altitude_m} meters...")
        await drone.action.set_takeoff_altitude(target_altitude_m)
        await asyncio.sleep(1.0)
        loop = asyncio.get_running_loop()
        print(F"Position-> N: {home_n} E: {home_e}")
        print(F"Remaining Battery Level-> {battery_remain}")
        user_input = await loop.run_in_executor(None, input, "Do you want to proceed? (y/n): ")
        choice = user_input.strip().lower()
            
        if choice in ['y', 'yes']:
            print("Proceeding...")
        elif choice in ['n', 'no']:
            print("Quitting program.")
            sys.exit(0)
        else:
            print("Invalid input. Please enter 'y' or 'n'.")

        print("Arming...")
        await drone.action.arm()

        print(f"HOME position N={home_n:.2f} E={home_e:.2f} D={current_d:.2f}")
        
        # Required before starting Offboard
        print("PX4 - Sending initial velocity setpoints...")
        for _ in range(20):
            await send_velocity(0.0, 0.0, 0.0)
            await asyncio.sleep(0.1)

        print("Starting Offboard...")
        await drone.offboard.start()

        print("WAYPOINT 1...")
        # proceeding to waypoint 1 which is 1m in front of the drone.
        current_n, current_e, state = get_uwb_position() 
        current_d = get_current_height()
        print(f"CURRENT position N={current_n:.2f} E={current_e:.2f} D={current_d:.2f}")
        await fly_to_position_velocity(target_n=current_n+1.0, target_e=current_e, target_d=-1.5, ignore_height=True,test=False)

        print("WAYPOINT 2...")
        # proceeding to waypoint 1 which is 1m in front of the drone.
        current_n, current_e, state = get_uwb_position() 
        current_d = get_current_height()
        print(f"CURRENT position N={current_n:.2f} E={current_e:.2f} D={current_d:.2f}")
        await fly_to_position_velocity(target_n=current_n, target_e=current_e-1.0, target_d=-1.5, ignore_height=True,test=False)

        # Stop offboard
        print("Stopping Offboard...")
        try:
            await drone.offboard.stop()
        except Exception as e:
            print(f"Offboard stop error: {e}")

        # Land
        print("Landing...")
        await drone.action.land()

        async for in_air in drone.telemetry.in_air():
            if not in_air:
                break
            await asyncio.sleep(0.5)

        print("Landed")

        try:
            await drone.action.disarm()
        except Exception:
            pass

        print("Mission complete")

    except Exception as e:
        print(f"Exception: {e}")
        try:
            await send_velocity(0.0, 0.0, 0.0)
        except Exception:
            pass
        try:
            await drone.offboard.stop()
        except Exception:
            pass
        try:
            await drone.action.land()
        except Exception:
            pass
    finally:
        # Cleanup ROS2 context gracefully
        try:
            if uwb_node is not None:
                uwb_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"ROS2 shutdown error: {e}")

if __name__ == "__main__":
    asyncio.run(run())