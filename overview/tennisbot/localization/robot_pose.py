"""Robot world pose from AprilTag + FieldTransform.

Do not treat the image-space heading as world theta. Map the tag center and a
forward pixel through the homography, then atan2 in the world frame.
"""

from __future__ import annotations

import numpy as np

from tennisbot.calibration.homography import FieldTransform
from tennisbot.perception.apriltag_detector import heading_from_corners
from tennisbot.perception.models import AprilTagDetection, RobotPose


def front_uv(
    detection: AprilTagDetection,
    front_edge: str = "top",
) -> tuple[float, float]:
    _heading, _center, front = heading_from_corners(detection.corners_uv, front_edge)
    return front


def correct_elevated_point(
    ground_mapped_xy: tuple[float, float],
    camera_ground_xy: tuple[float, float],
    camera_height_m: float,
    point_height_m: float,
) -> tuple[float, float]:
    """Undo ground-homography parallax for a point above the floor.

    A homography calibrated on the floor maps the elevated point to the place
    where its camera ray intersects the floor. The real horizontal position is
    closer to the camera's floor projection by (H-h)/H.
    """
    camera_height_m = float(camera_height_m)
    point_height_m = float(point_height_m)
    if camera_height_m <= 0.0:
        raise ValueError("camera_height_m must be positive")
    if not 0.0 <= point_height_m < camera_height_m:
        raise ValueError("point_height_m must satisfy 0 <= h < camera_height_m")
    scale = (camera_height_m - point_height_m) / camera_height_m
    gx, gy = ground_mapped_xy
    cx, cy = camera_ground_xy
    return cx + scale * (gx - cx), cy + scale * (gy - cy)


def pose_from_tag(
    detection: AprilTagDetection,
    transform: FieldTransform,
    front_edge: str = "top",
    tag_config: dict | None = None,
) -> RobotPose:
    """Convert an AprilTag detection into RobotPose (meters, radians)."""
    center_uv = detection.center_uv
    fwd_uv = front_uv(detection, front_edge)
    x, y = transform.image_to_world(center_uv)
    fx, fy = transform.image_to_world(fwd_uv)
    tag_config = tag_config or {}
    correction = tag_config.get("height_correction", {})
    if bool(correction.get("enabled", False)):
        camera_xy = (
            float(correction.get("camera_ground_x_m", transform.width_m * 0.5)),
            float(correction.get("camera_ground_y_m", 0.0)),
        )
        camera_height = float(correction.get("camera_height_m", 0.0))
        tag_height = float(correction.get("tag_height_m", 0.0))
        x, y = correct_elevated_point((x, y), camera_xy, camera_height, tag_height)
        fx, fy = correct_elevated_point((fx, fy), camera_xy, camera_height, tag_height)
    theta = float(np.arctan2(fy - y, fx - x))

    # Optional rigid transform from tag center to chassis control center.
    tag_forward = float(tag_config.get("tag_offset_forward_m", 0.0))
    tag_left = float(tag_config.get("tag_offset_left_m", 0.0))
    x -= float(np.cos(theta) * tag_forward - np.sin(theta) * tag_left)
    y -= float(np.sin(theta) * tag_forward + np.cos(theta) * tag_left)
    return RobotPose(x=x, y=y, theta=theta)
