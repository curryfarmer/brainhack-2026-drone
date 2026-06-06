import pyrealsense2 as rs
import cv2
import numpy as np

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()

        left_ir = frames.get_infrared_frame(1)
        right_ir = frames.get_infrared_frame(2)

        if not left_ir or not right_ir:
            continue

        left_img = np.asanyarray(left_ir.get_data())
        right_img = np.asanyarray(right_ir.get_data())

        cv2.imshow("Left IR", left_img)
        cv2.imshow("Right IR", right_img)

        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()