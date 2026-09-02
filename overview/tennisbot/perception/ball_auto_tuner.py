"""Automatic HSV tuning from user-marked fluorescent tennis balls."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from tennisbot.perception.ball_detector import BallDetector


@dataclass(frozen=True)
class AutoTuneResult:
    hsv_lower: tuple[int, int, int]
    hsv_upper: tuple[int, int, int]
    min_area: float
    marked_balls: int
    sampled_pixels: int


class BallAutoTuner:
    """Learn robust thresholds from small patches centered on marked balls."""

    def __init__(self) -> None:
        self.points: list[tuple[int, int]] = []
        self._samples: list[np.ndarray] = []
        self._areas: list[int] = []

    def clear(self) -> None:
        self.points.clear()
        self._samples.clear()
        self._areas.clear()

    def undo(self) -> None:
        if self.points:
            self.points.pop()
            self._samples.pop()
            self._areas.pop()

    def add_ball(
        self,
        frame_bgr: np.ndarray,
        x: int,
        y: int,
        radius: int | None = None,
    ) -> AutoTuneResult:
        height, width = frame_bgr.shape[:2]
        if not (0 <= x < width and 0 <= y < height):
            raise ValueError("点击位置超出图像范围。")
        sample_radius = radius or max(8, int(round(min(width, height) * 0.02)))
        x0, x1 = max(0, x - sample_radius), min(width, x + sample_radius + 1)
        y0, y1 = max(0, y - sample_radius), min(height, y + sample_radius + 1)
        patch = cv2.cvtColor(frame_bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)

        center_radius = max(1, sample_radius // 5)
        cx0, cx1 = max(0, x - center_radius), min(width, x + center_radius + 1)
        cy0, cy1 = max(0, y - center_radius), min(height, y + center_radius + 1)
        center_patch = cv2.cvtColor(
            frame_bgr[cy0:cy1, cx0:cx1], cv2.COLOR_BGR2HSV
        ).reshape(-1, 3)
        center = np.median(center_patch, axis=0)
        center_h, center_s, center_v = map(float, center)
        if center_s < 70 or center_v < 60:
            raise ValueError("点击位置不像荧光网球，请点击球体中央最鲜艳的位置。")

        pixels = patch.reshape(-1, 3)
        hue_delta = np.abs(((pixels[:, 0].astype(float) - center_h + 90) % 180) - 90)
        keep = (
            (hue_delta <= 12)
            & (pixels[:, 1] >= max(70, center_s - 90))
            & (pixels[:, 2] >= max(50, center_v - 90))
        )
        selected = pixels[keep]
        if selected.shape[0] < 20:
            raise ValueError("采到的球颜色像素太少，请更准确地点击球体中央。")

        self.points.append((int(x), int(y)))
        self._samples.append(selected)
        self._areas.append(int(selected.shape[0]))
        return self.result()

    def result(self) -> AutoTuneResult:
        if not self._samples:
            raise ValueError("还没有标注网球。")
        pixels = np.concatenate(self._samples, axis=0).astype(float)

        # Unwrap hue around the first sample so red (179/0) also works.
        reference = float(np.median(self._samples[0][:, 0]))
        hue_unwrapped = reference + ((pixels[:, 0] - reference + 90) % 180) - 90
        h_low = float(np.percentile(hue_unwrapped, 2)) - 4
        h_high = float(np.percentile(hue_unwrapped, 98)) + 4
        hsv_lower = (
            int(round(h_low)) % 180,
            int(np.clip(np.percentile(pixels[:, 1], 2) - 25, 40, 255)),
            int(np.clip(np.percentile(pixels[:, 2], 2) - 30, 40, 255)),
        )
        hsv_upper = (
            int(round(h_high)) % 180,
            255,
            255,
        )
        min_area = float(np.clip(min(self._areas) * 0.20, 30, 500))
        return AutoTuneResult(
            hsv_lower=hsv_lower,
            hsv_upper=hsv_upper,
            min_area=min_area,
            marked_balls=len(self.points),
            sampled_pixels=int(pixels.shape[0]),
        )

    def apply(self, detector: BallDetector) -> AutoTuneResult:
        result = self.result()
        detector.hsv_lower = np.asarray(result.hsv_lower, dtype=np.uint8)
        detector.hsv_upper = np.asarray(result.hsv_upper, dtype=np.uint8)
        detector.min_area = result.min_area
        detector.min_circularity = 0.55
        detector.morph_kernel = 5
        return result

