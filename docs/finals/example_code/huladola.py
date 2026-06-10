# A simple and short reference code of connecting to multiple hula drones in the wifi network.
# Make sure your laptop is in that network of hula drones and make sure all hula drones are online in that network
# This code uses pyhulax python library to connect to Hula Drone. 
# Refer to https://pyhulax.xenops.ae to learn how to use library to control movements of Hula drone swarm
# Ths code only shows you to get all video streams of all drones onto one computer so
# you can do multiple drone detection from one computer.
# You need pyhulax and dola.py

from pyhulax import DroneAPI
from pyhulax.core import Direction
import cv2
from dola import Dola
from pyhulax.video import VideoStream, VideoDisplay

dola = Dola()
dola.start()

try:
    print("Searching for all drones")
    d = dola.get_all_ips( # Dola is drone explorer. Find all drones in the network
        listen_seconds=5
    )
finally:
    dola.stop()
    print("dola stop")

drones = {} # store all drones object for control
streams = {} # store all video object for live straam access

for plane_id, ip in d.items():
    print(f"Plane {plane_id}: {ip}")

    if not ip:
        continue

    if not ip.startswith("192.168.100."):
        print("Skipping invalid IP:", ip)
        continue

    try:
        drone = DroneAPI()
        drone.connect(ip)

        drones[str(ip)] = drone

        v = drone.create_video_stream()
        drone.set_video_stream(True)

        if v:
            streams[str(ip)] = v
            v.start()

    except Exception as e:
        print(f"Failed to connect to {ip}: {e}")
    v = drones[str(ip)].create_video_stream() # Get VideoStream object
    drones[str(ip)].set_video_stream(True) # Turn on video stream
    if v is not None:
        streams[str(ip)] = v # Store Videostream into a dict for future use
        streams[str(ip)].start() #start video

while True: # just to keep looping . Add your drone commands for each respective drones in this loop.
    for i in drones.keys(): # loop through all drones by its ip address
        d = drones[i] # get the drone object of that ip address
        if d is None:
            print(F"{i} has no drone object")
            continue
        # write your control code here. For example, you can make the drone takeoff by calling d.takeoff() or move forward by d.move(Direction.FORWARD, 0.5) and etc. Refer to pyhulax documentation for more details on drone control commands.
        # IMPORTANT CONCEPT: Break your plan into states and use if else statement to control the flow of the drone. 
        # For example, you can have a state variable for each drone that starts at 0. When state is 0, you can make the drone takeoff and then set state to 1. 
        # When state is 1, you can make the drone move forward for 2 seconds and then set state to 2. 
        # This way you can have a sequence of commands for your drone and control the flow of the commands by changing the state variable.
        #  You can also use timers to change states after certain amount of time.
        if streams[i] is None:
            print(F"{i} has no video stream")
        else:
            s = streams[i] # get the stream of that ip address
            if s is None:
                continue
            
            f = s.latest_frame # get the latest frame object
            if f is not None:
                # add your detection code here. f.to_rgb() will give you the np array of the frame that you can do opencv detection on.
                # for example, yolo and etc
                cv2.imshow(str(i),f.to_rgb()) #call to_rgb to get np array of the frame
                cv2.waitKey(1)


