#!/usr/bin/env python3
"""
barrel_logger.py
================
Owns the list of detected barrels in world (NED) coordinates.

Responsibilities:
- Receives YOLO detections (bbox + class) via the Detector callback.
- For each detection, samples depth at the bbox center to estimate distance.
- Projects the pixel + depth into the camera frame, then into NED world coords
  using the current drone pose.
- Clusters detections within MERGE_RADIUS to avoid logging the same barrel
  many times from successive frames.
- Saves the final list to JSON at the end of the run.

Coordinate frames used here:
- Camera optical frame:  X=right, Y=down, Z=forward     (standard CV)
- Body FRD              : X=forward, Y=right, Z=down    (PX4 body)
- World NED             : X=north,   Y=east,  Z=down    (PX4 world)

For x500_vision the camera is forward-facing and level mount, so:
  body_forward = cam_Z
  body_right   = cam_X
  body_down    = cam_Y
"""

import json
import math
import threading
import time
from typing import Any, Dict, List, Optional

import numpy as np


class BarrelLogger:
    def __init__(
        self,
        K: np.ndarray,
        barrel_class_names: List[str],
        get_pose_fn,
        get_depth_fn,
        merge_radius_m: float = 1.0,
        min_confidence: float = 0.5,
        max_depth_m: float = 10.0,
    ):
        """
        Parameters
        ----------
        K : 3x3 camera intrinsic matrix.
        barrel_class_names : list of class name strings that count as "a barrel".
            Anything not in this list is ignored.
        get_pose_fn : callable returning dict with keys 'north','east','down','yaw'
                      (yaw in radians).
        get_depth_fn : callable returning the latest depth frame as np.ndarray
                       (float32, meters). May return None.
        merge_radius_m : two detections within this distance in world NED are
                         considered the same barrel.
        min_confidence : ignore detections below this.
        max_depth_m : ignore detections whose depth reading is implausibly far
                      (likely noise or background).
        """
        self.fx = float(K[0, 0])
        self.fy = float(K[1, 1])
        self.cx = float(K[0, 2])
        self.cy = float(K[1, 2])

        self.barrel_class_names = set(barrel_class_names)
        self.get_pose = get_pose_fn
        self.get_depth = get_depth_fn

        self.merge_radius_m = merge_radius_m
        self.min_confidence = min_confidence
        self.max_depth_m = max_depth_m

        # Each entry: {north, east, down, confidence, hits, class_name, last_seen}
        self.barrels: List[Dict[str, Any]] = []
        self._lock = threading.Lock()

    # ----------------------------------------------------------------------
    # Pixel + depth -> world NED
    # ----------------------------------------------------------------------
    def _pixel_to_world_ned(
        self,
        u: float,
        v: float,
        depth_m: float,
        pose: Dict[str, float],
    ) -> Optional[np.ndarray]:
        """
        Project a pixel (u, v) with depth into world NED coordinates.

        Returns np.array([north, east, down]) or None if invalid.
        """
        if depth_m is None or depth_m <= 0.05 or depth_m > self.max_depth_m:
            return None
        if not math.isfinite(depth_m):
            return None

        # 1) Pixel -> camera optical frame (X right, Y down, Z forward)
        x_cam = (u - self.cx) * depth_m / self.fx
        y_cam = (v - self.cy) * depth_m / self.fy
        z_cam = depth_m

        # 2) Camera optical -> body FRD
        #    body_forward = z_cam, body_right = x_cam, body_down = y_cam
        body_forward = z_cam
        body_right = x_cam
        body_down = y_cam

        # 3) Body FRD -> world NED via yaw rotation
        #    yaw is clockwise from north (standard NED heading)
        yaw = pose["yaw"]
        c = math.cos(yaw)
        s = math.sin(yaw)

        # rotate (forward, right) into (north, east)
        north_offset = body_forward * c - body_right * s
        east_offset = body_forward * s + body_right * c
        down_offset = body_down

        north = pose["north"] + north_offset
        east = pose["east"] + east_offset
        down = pose["down"] + down_offset

        return np.array([north, east, down], dtype=np.float64)

    # ----------------------------------------------------------------------
    # Median depth in a small patch around (u, v) for robustness
    # ----------------------------------------------------------------------
    def _sample_depth(self, depth_img: np.ndarray, u: int, v: int, patch: int = 5) -> Optional[float]:
        h, w = depth_img.shape[:2]
        u0 = max(0, u - patch)
        u1 = min(w, u + patch + 1)
        v0 = max(0, v - patch)
        v1 = min(h, v + patch + 1)
        region = depth_img[v0:v1, u0:u1]
        # Filter NaN, inf, zero
        valid = region[np.isfinite(region) & (region > 0.05)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    # ----------------------------------------------------------------------
    # Cluster merge
    # ----------------------------------------------------------------------
    def _merge_or_add(self, pos_ned: np.ndarray, confidence: float, class_name: str) -> None:
        with self._lock:
            for b in self.barrels:
                existing = np.array([b["north"], b["east"], b["down"]])
                # Use horizontal distance only — altitude estimate is noisy
                dn = pos_ned[0] - existing[0]
                de = pos_ned[1] - existing[1]
                dist = math.sqrt(dn * dn + de * de)
                if dist < self.merge_radius_m:
                    # Merge: weighted average toward higher-confidence reading,
                    # bump hit count, keep best confidence
                    b["hits"] += 1
                    if confidence > b["confidence"]:
                        b["confidence"] = confidence
                    # Running average position
                    w_new = 1.0 / b["hits"]
                    b["north"] = b["north"] * (1 - w_new) + pos_ned[0] * w_new
                    b["east"] = b["east"] * (1 - w_new) + pos_ned[1] * w_new
                    b["down"] = b["down"] * (1 - w_new) + pos_ned[2] * w_new
                    b["last_seen"] = time.time()
                    return

            # New barrel
            self.barrels.append({
                "north": float(pos_ned[0]),
                "east": float(pos_ned[1]),
                "down": float(pos_ned[2]),
                "confidence": float(confidence),
                "hits": 1,
                "class_name": class_name,
                "last_seen": time.time(),
            })
            print(f"[BarrelLogger] NEW barrel #{len(self.barrels)} at "
                  f"N={pos_ned[0]:.2f} E={pos_ned[1]:.2f} ({class_name}, conf={confidence:.2f})")

    # ----------------------------------------------------------------------
    # The Detector callback
    # ----------------------------------------------------------------------
    def on_detection(
        self,
        detections: List[Dict[str, Any]],
        annotated_image: np.ndarray,
        context: Optional[Dict[str, Any]],
    ) -> None:
        """
        Called by Detector worker thread when YOLO finds something.
        Runs in detector worker thread — keep it short and lock-protected.
        """
        if not detections:
            return

        pose = self.get_pose()
        if pose is None:
            return

        depth_img = self.get_depth()
        if depth_img is None:
            return

        for det in detections:
            class_name = det.get("class_name", "")
            if self.barrel_class_names and class_name not in self.barrel_class_names:
                continue

            confidence = det.get("confidence", 0.0)
            if confidence < self.min_confidence:
                continue

            x1, y1, x2, y2 = det["bbox"]
            u = int(round((x1 + x2) / 2.0))
            v = int(round((y1 + y2) / 2.0))

            # Bbox dimensions may differ between RGB image and depth image
            # if the two streams aren't at identical resolution. We assume here
            # they are (640x480 in the deck's intrinsics). If not, scale (u,v).
            depth_m = self._sample_depth(depth_img, u, v)
            if depth_m is None:
                continue

            pos_ned = self._pixel_to_world_ned(u, v, depth_m, pose)
            if pos_ned is None:
                continue

            self._merge_or_add(pos_ned, confidence, class_name)

    # ----------------------------------------------------------------------
    # Output
    # ----------------------------------------------------------------------
    def get_barrels(self) -> List[Dict[str, Any]]:
        with self._lock:
            return [dict(b) for b in self.barrels]

    def save_json(self, path: str = "barrels.json") -> None:
        with self._lock:
            data = {
                "count": len(self.barrels),
                "generated_at": time.time(),
                "barrels": [
                    {
                        "id": i,
                        "north_m": b["north"],
                        "east_m": b["east"],
                        "down_m": b["down"],
                        "confidence": b["confidence"],
                        "hits": b["hits"],
                        "class_name": b["class_name"],
                    }
                    for i, b in enumerate(self.barrels)
                ],
            }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"[BarrelLogger] Saved {data['count']} barrels to {path}")
