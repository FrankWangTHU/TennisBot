import math

import numpy as np
import pytest

from tennisbot.calibration.homography import FieldTransform
from tennisbot.localization.robot_pose import pose_from_tag
from tennisbot.perception.apriltag_detector import heading_from_corners
from tennisbot.perception.models import AprilTagDetection


def detection(corners) -> AprilTagDetection:
    heading, center, _front = heading_from_corners(corners, "top")
    return AprilTagDetection(7, center, np.asarray(corners, dtype=float), heading)


def test_heading_uses_world_frame_after_perspective_mapping() -> None:
    transform = FieldTransform.from_correspondences(
        image_points=[(100, 900), (1800, 800), (1450, 100), (400, 150)],
        world_points=[(0, 0), (5, 0), (5, 3), (0, 3)],
        width_m=5,
        height_m=3,
    )
    center_xy = (2.5, 1.5)
    half = 0.15
    world_corners = [
        (center_xy[0] - half, center_xy[1] + half),
        (center_xy[0] + half, center_xy[1] + half),
        (center_xy[0] + half, center_xy[1] - half),
        (center_xy[0] - half, center_xy[1] - half),
    ]
    image_corners = [transform.world_to_image(xy) for xy in world_corners]
    pose = pose_from_tag(detection(image_corners), transform, "top")
    assert (pose.x, pose.y) == pytest.approx(center_xy, abs=0.02)
    assert pose.theta == pytest.approx(math.pi / 2, abs=0.02)


@pytest.mark.parametrize(
    ("front_edge", "expected"),
    [("top", -math.pi / 2), ("right", 0.0), ("bottom", math.pi / 2), ("left", math.pi)],
)
def test_image_heading_edge_convention(front_edge: str, expected: float) -> None:
    corners = np.array([(0, 0), (2, 0), (2, 2), (0, 2)], dtype=float)
    heading, center, _front = heading_from_corners(corners, front_edge)
    assert center == pytest.approx((1, 1))
    assert abs(math.atan2(math.sin(heading - expected), math.cos(heading - expected))) < 1e-9

