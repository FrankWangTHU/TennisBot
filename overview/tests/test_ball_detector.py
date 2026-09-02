import cv2
import numpy as np
import pytest

from tennisbot.perception.ball_detector import BallDetector, circularity


def test_detects_synthetic_yellow_ball_and_ignores_small_noise() -> None:
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(frame, (170, 110), 24, (0, 255, 255), -1)
    cv2.circle(frame, (20, 20), 2, (0, 255, 255), -1)
    detector = BallDetector(
        hsv_lower=(20, 100, 100),
        hsv_upper=(40, 255, 255),
        min_area=100,
        max_area=5000,
        min_circularity=0.7,
        morph_kernel=3,
    )
    balls, mask = detector.detect(frame)
    assert mask.shape == frame.shape[:2]
    assert len(balls) == 1
    assert balls[0].center_uv == pytest.approx((170, 110), abs=1)
    assert balls[0].radius_px == pytest.approx(24, abs=2)


def test_circularity_handles_zero_perimeter() -> None:
    assert circularity(10, 0) == 0.0


def test_rejects_invalid_hsv_config() -> None:
    with pytest.raises(ValueError, match="HSV"):
        BallDetector(hsv_lower=(200, 0, 0))


def test_hue_range_can_wrap_for_red_without_affecting_yellow_defaults() -> None:
    frame = np.zeros((100, 180, 3), dtype=np.uint8)
    cv2.circle(frame, (40, 50), 12, (0, 0, 255), -1)
    cv2.circle(frame, (140, 50), 12, (0, 255, 255), -1)
    detector = BallDetector(
        hsv_lower=(170, 100, 100),
        hsv_upper=(10, 255, 255),
        min_area=50,
        min_circularity=0.5,
        morph_kernel=3,
    )
    balls, _mask = detector.detect(frame)
    assert len(balls) == 1
    assert balls[0].center_uv == pytest.approx((40, 50), abs=1)
