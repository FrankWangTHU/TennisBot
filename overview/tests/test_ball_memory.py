import numpy as np

from tennisbot.calibration.homography import FieldTransform
from tennisbot.localization.ball_memory import BallMemoryTracker
from tennisbot.perception.models import BallDetection, BallWorld, RobotPose


def transform() -> FieldTransform:
    return FieldTransform.from_correspondences(
        [(0, 100), (100, 100), (100, 0), (0, 0)],
        [(0, 0), (1, 0), (1, 1), (0, 1)],
        1,
        1,
    )


def observation(x: float = 0.5, y: float = 0.5):
    det = BallDetection(center_uv=(x * 100, (1 - y) * 100), radius_px=8, area_px=150)
    return det, BallWorld(x, y)


def test_occluded_ball_is_retained_then_removed_when_floor_is_clear() -> None:
    memory = BallMemoryTracker(removal_confirm_frames=3, robot_occlusion_radius_m=0.25)
    white = np.full((101, 101, 3), 230, dtype=np.uint8)
    dark_occluder = white.copy()
    dark_occluder[35:66, 35:66] = 10
    all_balls, visible = memory.update([observation()], white, transform(), None)
    assert len(all_balls) == len(visible) == 1
    ball_id = all_balls[0].track_id

    for _ in range(5):
        all_balls, _ = memory.update([], dark_occluder, transform(), None)
    assert len(all_balls) == 1
    assert all_balls[0].track_id == ball_id
    assert not all_balls[0].visible

    robot_near = RobotPose(0.5, 0.5, 0.0)
    for _ in range(5):
        all_balls, _ = memory.update([], white, transform(), robot_near)
    assert len(all_balls) == 1

    for _ in range(3):
        all_balls, _ = memory.update([], white, transform(), None)
    assert all_balls == []


def test_reappearing_ball_keeps_persistent_id() -> None:
    memory = BallMemoryTracker(removal_confirm_frames=5)
    frame = np.zeros((101, 101, 3), dtype=np.uint8)
    first, _ = memory.update([observation()], frame, transform(), None)
    ball_id = first[0].track_id
    memory.update([], frame, transform(), None)
    again, visible = memory.update([observation(0.53, 0.49)], frame, transform(), None)
    assert again[0].track_id == ball_id
    assert visible[0][1].track_id == ball_id
    assert again[0].visible
