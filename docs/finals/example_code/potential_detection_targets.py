Practise training a object detection model. Besides that, detection of fiducial markers (QR code, Aruco marker, AprilTag) are likely target. OpenCV has libraries to detect to Aruco marker and read QR code.

Sample python code of detecting Aruco marker using OpenCV

import cv2

self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
self.parameters = cv2.aruco.DetectorParameters()
self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.parameters)


img = self.colorimage.copy()
corners, ids,  = self.detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
if ids is not None:
       cv2.aruco.drawDetectedMarkers(img, corners, ids) # draw bounding box
        if len(corners) > 0:
            ids = ids.flatten()


            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                center_v = cY
                center_u = cX
                depth_mm = self.depth_image[center_v, center_u] # get the distance to the image
                if depth_mm == 0:
                    continue
                depth_m = float(depth_mm) / 1000.0
                X = (center_u - self.cx) * depth_m / self.fx #convert to actual distance
                Y = (center_v - self.cy) * depth_m / self.fy
                Z = depth_m