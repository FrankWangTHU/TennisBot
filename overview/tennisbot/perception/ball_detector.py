"""HSV tennis-ball detection (no YOLO in V1)."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from tennisbot.perception.models import BallDetection


def circularity(area: float, perimeter: float) -> float:
    if perimeter <= 1e-6:
        return 0.0
    return float(4.0 * np.pi * area / (perimeter * perimeter))


class BallDetector:
    def __init__(
        self,
        hsv_lower: list[int] | tuple[int, ...] = (20, 80, 80),
        hsv_upper: list[int] | tuple[int, ...] = (45, 255, 255),
        min_area: float = 50.0,
        max_area: float = 10000.0,
        min_circularity: float = 0.45,
        morph_kernel: int = 5,
    ) -> None:
        lower = np.asarray(hsv_lower, dtype=np.int32).reshape(-1)
        upper = np.asarray(hsv_upper, dtype=np.int32).reshape(-1)
        if lower.size != 3 or upper.size != 3:
            raise ValueError("hsv_lower 和 hsv_upper 必须各包含 3 个整数。")
        limits = np.array([179, 255, 255], dtype=np.int32)
        if np.any(lower < 0) or np.any(upper < 0) or np.any(lower > limits) or np.any(upper > limits):
            raise ValueError("HSV 范围无效：H 必须为 0..179，S/V 必须为 0..255。")
        if np.any(lower[1:] > upper[1:]):
            raise ValueError("HSV 的 S/V 下限必须小于或等于对应上限。")
        if min_area < 0 or max_area <= 0 or min_area > max_area:
            raise ValueError("面积范围无效：需要 0 <= min_area <= max_area。")
        if not 0.0 <= min_circularity <= 1.0:
            raise ValueError("min_circularity 必须在 0..1 之间。")
        if int(morph_kernel) < 1:
            raise ValueError("morph_kernel 必须大于等于 1。")
        self.hsv_lower = lower.astype(np.uint8)
        self.hsv_upper = upper.astype(np.uint8)
        self.min_area = float(min_area)
        self.max_area = float(max_area)
        self.min_circularity = float(min_circularity)
        self.morph_kernel = int(morph_kernel)

    @classmethod
    def from_config(cls, perception_cfg: dict[str, Any]) -> BallDetector:
        cfg = perception_cfg.get("ball_detection", {})
        return cls(
            hsv_lower=cfg.get("hsv_lower", [20, 80, 80]),
            hsv_upper=cfg.get("hsv_upper", [45, 255, 255]),
            min_area=float(cfg.get("min_area", 50)),
            max_area=float(cfg.get("max_area", 10000)),
            min_circularity=float(cfg.get("min_circularity", 0.45)),
            morph_kernel=int(cfg.get("morph_kernel", 5)),
        )

    def update_from_config(self, perception_cfg: dict[str, Any]) -> None:
        other = BallDetector.from_config(perception_cfg)
        self.hsv_lower = other.hsv_lower
        self.hsv_upper = other.hsv_upper
        self.min_area = other.min_area
        self.max_area = other.max_area
        self.min_circularity = other.min_circularity
        self.morph_kernel = other.morph_kernel

    def mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        if int(self.hsv_lower[0]) <= int(self.hsv_upper[0]):
            mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        else:
            # Red commonly crosses OpenCV Hue's 179 -> 0 boundary. Hmin > Hmax
            # deliberately means [Hmin..179] OR [0..Hmax].
            high_red = cv2.inRange(
                hsv,
                np.array([self.hsv_lower[0], self.hsv_lower[1], self.hsv_lower[2]], dtype=np.uint8),
                np.array([179, self.hsv_upper[1], self.hsv_upper[2]], dtype=np.uint8),
            )
            low_red = cv2.inRange(
                hsv,
                np.array([0, self.hsv_lower[1], self.hsv_lower[2]], dtype=np.uint8),
                self.hsv_upper,
            )
            mask = cv2.bitwise_or(high_red, low_red)
        k = max(1, self.morph_kernel)
        if k % 2 == 0:
            k += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def detect(self, frame_bgr: np.ndarray) -> tuple[list[BallDetection], np.ndarray]:
        mask = self.mask(frame_bgr)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        balls: list[BallDetection] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area or area > self.max_area:
                continue
            peri = float(cv2.arcLength(contour, True))
            if circularity(area, peri) < self.min_circularity:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            balls.append(
                BallDetection(
                    center_uv=(float(cx), float(cy)),
                    radius_px=float(radius),
                    area_px=area,
                )
            )
        balls.sort(key=lambda b: (b.center_uv[0], b.center_uv[1]))
        return balls, mask
