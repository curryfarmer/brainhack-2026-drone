# a sample code to geenerate a top-down occupancy grid from RealSense depth data
# realsense camera frame when the camera is placed facing down
#      Z (Forward)
#         ^
#         |
#         |
#         |
#         O------> X (Right)
#        /
#       /
#      Y (Down)
# Top down occupancy grid will be generated in the X-Z plane, with the camera at the bottom center of the grid
#    Forward (North) y
#            ^
#            |
#            |
# Left <-----+-----> Right (East) x
#            |
#            |

import pyrealsense2 as rs
import numpy as np
import cv2

# ==========================================
# Configuration
# ==========================================

WIDTH = 640
HEIGHT = 480

MAX_DEPTH = 5.0      # meters
MIN_DEPTH = 0.2      # meters

GRID_RESOLUTION = 0.05    # 5 cm/cell
GRID_WIDTH = 200          # 10 m wide
GRID_HEIGHT = 200         # 10 m forward

# ==========================================
# RealSense Setup
# ==========================================

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.depth,
    WIDTH,
    HEIGHT,
    rs.format.z16,
    30
)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

print("Depth Scale:", depth_scale)

intrinsics = (
    profile.get_stream(rs.stream.depth)
    .as_video_stream_profile()
    .get_intrinsics()
)

fx = intrinsics.fx
fy = intrinsics.fy
cx = intrinsics.ppx
cy = intrinsics.ppy

print("\nCamera Intrinsics")
print("--------------------")
print("fx =", fx)
print("fy =", fy)
print("cx =", cx)
print("cy =", cy)

# ==========================================
# Precompute Pixel Coordinates
# ==========================================

u_coords, v_coords = np.meshgrid(
    np.arange(WIDTH),
    np.arange(HEIGHT)
)

u_coords = u_coords.astype(np.float32)
v_coords = v_coords.astype(np.float32)

colorizer = rs.colorizer()

# ==========================================
# Main Loop
# ==========================================

try:

    while True:

        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue

        # ----------------------------------
        # Depth image in meters
        # ----------------------------------

        depth_image = np.asanyarray(
            depth_frame.get_data()
        )

        depth_m = depth_image.astype(
            np.float32
        ) * depth_scale

        # ----------------------------------
        # Valid depth mask
        # ----------------------------------

        valid = (
            (depth_m > MIN_DEPTH) &
            (depth_m < MAX_DEPTH)
        )

        # ----------------------------------
        # Deproject pixels -> 3D
        # ----------------------------------

        z = depth_m

        x = (u_coords - cx) * z / fx
        y = (v_coords - cy) * z / fy

        # Flatten valid points
        X = x[valid]
        Z = z[valid]

        # ----------------------------------
        # Create occupancy grid
        # ----------------------------------

        occupancy = np.zeros(
            (GRID_HEIGHT, GRID_WIDTH),
            dtype=np.uint8
        )

        grid_center_x = GRID_WIDTH // 2

        gx = (
            X / GRID_RESOLUTION
        ).astype(np.int32)

        gz = (
            Z / GRID_RESOLUTION
        ).astype(np.int32)

        gx += grid_center_x

        valid_grid = (
            (gx >= 0) &
            (gx < GRID_WIDTH) &
            (gz >= 0) &
            (gz < GRID_HEIGHT)
        )

        gx = gx[valid_grid]
        gz = gz[valid_grid]

        occupancy[
            GRID_HEIGHT - 1 - gz,
            gx
        ] = 255

        # ----------------------------------
        # Remove isolated noise
        # ----------------------------------

        occupancy = cv2.morphologyEx(
            occupancy,
            cv2.MORPH_CLOSE,
            np.ones((3, 3), np.uint8)
        )

        occupancy = cv2.morphologyEx(
            occupancy,
            cv2.MORPH_OPEN,
            np.ones((3, 3), np.uint8)
        )

        # ----------------------------------
        # Draw camera position
        # ----------------------------------

        cv2.circle(
            occupancy,
            (grid_center_x, GRID_HEIGHT - 1),
            5,
            128,
            -1
        )

        # ----------------------------------
        # Visualization
        # ----------------------------------

        depth_vis = np.asanyarray(
            colorizer.colorize(
                depth_frame
            ).get_data()
        )

        occupancy_vis = cv2.resize(
            occupancy,
            (600, 600),
            interpolation=cv2.INTER_NEAREST
        )

        cv2.putText(
            occupancy_vis,
            "Top Down Occupancy Grid",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            255,
            2
        )

        cv2.imshow(
            "Depth",
            depth_vis
        )

        cv2.imshow(
            "Occupancy Grid",
            occupancy_vis
        )

        key = cv2.waitKey(1)

        if key == 27:
            break

finally:

    pipeline.stop()
    cv2.destroyAllWindows()