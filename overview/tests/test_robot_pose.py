import math

import numpy as np
import pytest

from tennisbot.calibration.homography import FieldTransform
from tennisbot.localization.robot_pose import correct_elevated_point, pose_from_tag
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


def test_height_correction_pulls_ground_mapped_point_toward_camera() -> None:
    corrected = correct_elevated_point(
        (1.50, 1.00),
        camera_ground_xy=(1.25, -0.75),
        camera_height_m=2.30,
        point_height_m=0.17,
    )
    scale = (2.30 - 0.17) / 2.30
    assert corrected == pytest.approx(
        (1.25 + scale * 0.25, -0.75 + scale * 1.75)
    )


def test_height_correction_preserves_world_heading() -> None:
    transform = FieldTransform.from_correspondences(
        image_points=[(0, 100), (100, 100), (100, 0), (0, 0)],
        world_points=[(0, 0), (2, 0), (2, 1), (0, 1)],
        width_m=2,
        height_m=1,
    )
    pose = pose_from_tag(
        detection([(40, 40), (60, 40), (60, 60), (40, 60)]),
        transform,
        "right",
        {
            "height_correction": {
                "enabled": True,
                "camera_height_m": 2.30,
                "tag_height_m": 0.17,
                "camera_ground_x_m": 1.0,
                "camera_ground_y_m": -0.75,
            }
        },
    )
    assert pose.theta == pytest.approx(0.0, abs=1e-9)


def test_tag_to_chassis_center_offset_uses_robot_frame() -> None:
    transform = FieldTransform.from_correspondences(
        image_points=[(0, 100), (100, 100), (100, 0), (0, 0)],
        world_points=[(0, 0), (2, 0), (2, 1), (0, 1)],
        width_m=2,
        height_m=1,
    )
    pose = pose_from_tag(
        detection([(40, 40), (60, 40), (60, 60), (40, 60)]),
        transform,
        "right",
        {"tag_offset_forward_m": 0.10, "tag_offset_left_m": 0.05},
    )
    assert (pose.x, pose.y) == pytest.approx((0.90, 0.45), abs=1e-9)
