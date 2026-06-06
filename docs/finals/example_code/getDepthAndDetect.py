#For YOLO object detection with a RealSense D430/D450, you typically want to:

# 1. Capture both RGB and Depth streams.
# 2. Run YOLO on the RGB image.
# 3. Align the depth image to the RGB image.
# 4. For each detected object:
# 5. Get the center pixel of the bounding box.
# 6. Read the depth at that pixel.
# 7. Use camera intrinsics to convert the pixel into a 3D point.
# 8. Display:
# - Class name
# - Confidence
# - Distance from camera
# - X,Y,Z coordinates in meters


import pyrealsense2 as rs
import cv2
import numpy as np
#from ultralytics import YOLO
from rknnlite.api import RKNNLite
from rknndecoder import decode_yolov11_rknn, draw_detections
# --------------------------------------------------
# Load YOLO model
# --------------------------------------------------


rknn = RKNNLite()

rknn.load_rknn("yolo11n.rknn")
rknn.init_runtime()

# --------------------------------------------------
# Configure RealSense
# --------------------------------------------------

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
profile = pipeline.start(config)

# --------------------------------------------------
# Camera Information
# --------------------------------------------------

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

print("Depth Scale =", depth_scale)

depth_stream = profile.get_stream(rs.stream.depth)
depth_intrinsics = (depth_stream.as_video_stream_profile().get_intrinsics())

print("\nDepth Intrinsics")
print("----------------")
print("Width :", depth_intrinsics.width)
print("Height:", depth_intrinsics.height)
print("fx    :", depth_intrinsics.fx)
print("fy    :", depth_intrinsics.fy)
print("cx    :", depth_intrinsics.ppx)
print("cy    :", depth_intrinsics.ppy)

# --------------------------------------------------
# Align depth to color
# --------------------------------------------------

align = rs.align(rs.stream.color)
colorizer = rs.colorizer()

try:

    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        depth_image = np.asanyarray(depth_frame.get_data())

        # ------------------------------------------
        # YOLO Detection
        # ------------------------------------------
        input_img = cv2.resize(color_image, (640, 640))
        img_rgb = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
        img_rgb = img_rgb.astype(np.uint8)
        img_input = np.expand_dims(img_rgb, axis=0)
        outputs = rknn.inference(inputs=[img_input])
        print("OUTPUT TYPE:", type(outputs))

        boxes, scores, classes = decode_yolov11_rknn(
            outputs,
            img_shape=input_img.shape,
            model_input_size=(640, 640)
        )
        class_names = ["person", "car", "bicycle"]  # update for your model 

        result = draw_detections(input_img, boxes, scores, classes, class_names)

        annotated = result.copy()
        annotated = cv2.resize(annotated, (640, 480))

        for box, score, cls in zip(boxes, scores, classes):
            x1, y1, x2, y2 = box.astype(int)
            if cls in class_names:
                class_name = class_names[cls]
            else:
                class_name = "test"

            label = f"{class_name} {score:.2f}"

            cx = int((x1 + x2) / 2) # find center
            cy = int((y1 + y2) / 2) # find center

            if (cx < 0 or cx >= depth_intrinsics.width or cy < 0 or cy >= depth_intrinsics.height):
                continue
            # ----------------------------------
            # Distance in meters
            # ----------------------------------

            distance = depth_frame.get_distance(cx, cy)

            if distance <= 0:
                continue

            # ----------------------------------
            # Convert pixel -> 3D coordinate
            # ----------------------------------

            point_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics,[cx, cy],distance)

            X = point_3d[0]
            Y = point_3d[1]
            Z = point_3d[2]

            print(f" Detected object {class_name:15s} Dist={distance:.2f}m X={X:.2f} Y={Y:.2f} Z={Z:.2f}")

        # ------------------------------------------
        # Display
        # ------------------------------------------

        depth_vis = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        combined = np.hstack((annotated,depth_vis))

        cv2.imshow("YOLO + RealSense",combined)

        key = cv2.waitKey(1)

        if key == 27:
            break

finally:

    pipeline.stop()
    cv2.destroyAllWindows()