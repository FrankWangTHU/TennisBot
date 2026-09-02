import cv2
import numpy as np

from tennisbot.perception.ball_auto_tuner import BallAutoTuner
from tennisbot.perception.ball_detector import BallDetector


def test_marked_balls_automatically_tune_and_apply() -> None:
    frame = np.full((240, 360, 3), (180, 180, 180), dtype=np.uint8)
    cv2.circle(frame, (100, 100), 18, (0, 255, 220), -1)
    cv2.circle(frame, (260, 140), 14, (20, 240, 230), -1)
    tuner = BallAutoTuner()
    tuner.add_ball(frame, 100, 100, radius=24)
    result = tuner.add_ball(frame, 260, 140, radius=20)
    detector = BallDetector()
    tuner.apply(detector)
    balls, _mask = detector.detect(frame)

    assert result.marked_balls == 2
    assert result.hsv_lower[0] <= 40
    assert result.hsv_lower[1] >= 40
    assert len(balls) == 2


def test_rejects_gray_background_click() -> None:
    frame = np.full((100, 100, 3), 180, dtype=np.uint8)
    tuner = BallAutoTuner()
    try:
        tuner.add_ball(frame, 50, 50)
    except ValueError as exc:
        assert "不像荧光网球" in str(exc)
    else:
        raise AssertionError("gray background click should be rejected")
