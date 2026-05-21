import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from pynput import keyboard

# State to keep track of current movement
# Values represent: [Forward/North, Right/East, Down/Up, YawRate]
state = {
    'forward': 0.0,
    'right': 0.0,
    'down': 0.0,
    'yaw_rate': 0.0
}

# Movement Constants
SPEED = 2.0      # m/s
YAW_SPEED = 30.0  # deg/s

def on_press(key):
    try:
        char = key.char
        if char == 'w': state['down'] = -SPEED    # Throttle up
        elif char == 's': state['down'] = SPEED    # Throttle down
        elif char == 'a': state['yaw_rate'] = -YAW_SPEED # CCW
        elif char == 'd': state['yaw_rate'] = YAW_SPEED  # CW
        elif char == 'u': state['forward'] = SPEED  # Pitch Forward
        elif char == 'j': state['forward'] = -SPEED # Pitch Backward
        elif char == 'h': state['right'] = -SPEED   # Roll Left
        elif char == 'k': state['right'] = SPEED    # Roll Right
    except AttributeError:
        pass

def on_release(key):
    try:
        char = key.char
        if char in ['w', 's']: state['down'] = 0.0
        elif char in ['a', 'd']: state['yaw_rate'] = 0.0
        elif char in ['u', 'j']: state['forward'] = 0.0
        elif char in ['h', 'k']: state['right'] = 0.0
    except AttributeError:
        pass
    if key == keyboard.Key.esc:
        return False # Stop listener

async def run():
    drone = System()
    # Replace with your connection address (e.g., serial or udp)
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state_check in drone.core.connection_state():
        if state_check.is_connected:
            print("Drone connected!")
            break

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # Initial setpoint before starting offboard
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed: {error._result.result}")
        return

    print("Offboard started. Use keys to control. ESC to quit.")
    
    # Start keyboard listener in the background
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while listener.running:
        # Send setpoints at 20Hz
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                state['forward'], 
                state['right'], 
                state['down'], 
                state['yaw_rate']
            )
        )
        await asyncio.sleep(0.05)

    print("Stopping offboard and landing...")
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())