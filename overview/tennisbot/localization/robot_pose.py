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


def pose_from_tag(
    detection: AprilTagDetection,
    transform: FieldTransform,
    front_edge: str = "top",
) -> RobotPose:
    """Convert an AprilTag detection into RobotPose (meters, radians)."""
    center_uv = detection.center_uv
    fwd_uv = front_uv(detection, front_edge)
    x, y = transform.image_to_world(center_uv)
    fx, fy = transform.image_to_world(fwd_uv)
    theta = float(np.arctan2(fy - y, fx - x))
    return RobotPose(x=x, y=y, theta=theta)
