"""Project a YOLO bbox + depth frame + drone pose into a world-NED 3D point.

Pure-ish (no I/O). Designed to be called from the detection callback.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class Pose:
    """NED pose snapshot used at the instant a frame was captured."""
    north: float        # m
    east: float         # m
    down: float         # m (negative = up)
    yaw_rad: float      # radians, NED convention (0 = north, +CW from above)


@dataclass
class WorldDetection:
    class_name: str
    confidence: float
    north: float
    east: float
    down: float
    range_m: float      # camera-frame depth used for projection
    ts: float


def median_patch_depth(
    depth: np.ndarray,
    u: int,
    v: int,
    patch: int = 5,
    min_valid: float = 0.2,
    max_valid: float = 20.0,
) -> Optional[float]:
    """Median depth in a (patch x patch) window centred on (u,v).

    Returns None when no pixel in the window has a valid depth.
    """
    h, w = depth.shape
    half = patch // 2
    u0, u1 = max(0, u - half), min(w, u + half + 1)
    v0, v1 = max(0, v - half), min(h, v + half + 1)
    if u1 <= u0 or v1 <= v0:
        return None

    region = depth[v0:v1, u0:u1]
    mask = np.isfinite(region) & (region > min_valid) & (region < max_valid)
    if not mask.any():
        return None
    return float(np.median(region[mask]))


def project_pixel_to_world(
    u: float,
    v: float,
    depth_m: float,
    K: np.ndarray,
    pose: Pose,
    camera_pitch_deg: float = 0.0,
) -> Tuple[float, float, float]:
    """Pinhole back-projection + body→NED rotation.

    Camera frame convention: +Z forward, +X right, +Y down (standard OpenCV).
    Body frame: +X forward (drone nose), +Y right, +Z down.
    NED frame: +N north, +E east, +D down.

    For a forward-mounted camera with optional pitch, we rotate the camera-frame
    ray into the body frame, then apply yaw into NED.

    Args:
        u, v: bbox-center pixel coords.
        depth_m: scene depth at (u,v) in metres.
        K: 3x3 camera intrinsics matrix.
        pose: drone pose at frame timestamp.
        camera_pitch_deg: positive = camera tilted down. 0 = forward-looking.

    Returns:
        (north, east, down) world point.
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    # Camera frame point (OpenCV convention)
    Xc = (u - cx) * depth_m / fx
    Yc = (v - cy) * depth_m / fy
    Zc = depth_m

    # Camera → body (camera forward = +Z_cam → +X_body; right = +X_cam → +Y_body; down = +Y_cam → +Z_body)
    Xb = Zc
    Yb = Xc
    Zb = Yc

    # Optional camera pitch (rotate body x,z by -pitch so +pitch tilts camera down)
    if camera_pitch_deg != 0.0:
        p = math.radians(camera_pitch_deg)
        cp, sp = math.cos(p), math.sin(p)
        Xb2 = cp * Xb + sp * Zb
        Zb2 = -sp * Xb + cp * Zb
        Xb, Zb = Xb2, Zb2

    # Body → NED via yaw
    cy_, sy_ = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    north = pose.north + (cy_ * Xb - sy_ * Yb)
    east  = pose.east  + (sy_ * Xb + cy_ * Yb)
    down  = pose.down  + Zb

    return north, east, down


def detection_to_world(
    bbox_xyxy: Tuple[float, float, float, float],
    depth_frame: np.ndarray,
    pose: Pose,
    K: np.ndarray,
    class_name: str,
    confidence: float,
    ts: float,
    camera_pitch_deg: float = 0.0,
    patch: int = 5,
) -> Optional[WorldDetection]:
    """Top-level entry — returns None when depth at bbox centre is unusable."""
    x1, y1, x2, y2 = bbox_xyxy
    u = int(round((x1 + x2) / 2.0))
    v = int(round((y1 + y2) / 2.0))

    z = median_patch_depth(depth_frame, u, v, patch=patch)
    if z is None:
        return None

    n, e, d = project_pixel_to_world(u, v, z, K, pose, camera_pitch_deg)
    return WorldDetection(
        class_name=class_name,
        confidence=confidence,
        north=n,
        east=e,
        down=d,
        range_m=z,
        ts=ts,
    )


if __name__ == "__main__":
    # Sanity test: barrel directly in front, 3 m away, drone facing north at origin
    K = np.array([[433.0, 0.0, 320.0],
                  [0.0, 433.0, 240.0],
                  [0.0, 0.0, 1.0]])
    pose = Pose(north=0.0, east=0.0, down=-2.5, yaw_rad=0.0)
    depth = np.full((480, 640), 3.0, dtype=np.float32)
    bbox = (300, 220, 340, 260)  # centred
    wd = detection_to_world(bbox, depth, pose, K, "yellow_barrel", 0.9, ts=0.0)
    print(wd)  # expect north~3.0, east~0, down~-2.5
