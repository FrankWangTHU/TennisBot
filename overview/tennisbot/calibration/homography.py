"""Image <-> world homography. This is the only place that does the matrix math."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import yaml

from tennisbot.io.paths import HOMOGRAPHY_PATH


def _as_xy(point) -> tuple[float, float]:
    arr = np.asarray(point, dtype=np.float64).reshape(-1)
    if arr.size < 2:
        raise ValueError(f"需要至少 2 个数的坐标，收到: {point!r}")
    return float(arr[0]), float(arr[1])


class FieldTransform:
    """Perspective map between image pixels (u,v) and field meters (x,y)."""

    def __init__(
        self,
        image_points: np.ndarray,
        world_points: np.ndarray,
        width_m: float,
        height_m: float,
    ) -> None:
        self.image_points = np.asarray(image_points, dtype=np.float32).reshape(4, 2)
        self.world_points = np.asarray(world_points, dtype=np.float32).reshape(4, 2)
        self.width_m = float(width_m)
        self.height_m = float(height_m)
        if not np.isfinite(self.image_points).all() or not np.isfinite(self.world_points).all():
            raise ValueError("标定点必须全部是有限数值。")
        if self.width_m <= 0 or self.height_m <= 0:
            raise ValueError("场地 width_m 和 height_m 必须大于 0。")
        if abs(float(cv2.contourArea(self.image_points))) < 1.0:
            raise ValueError("图像标定四点退化或面积过小，请重新点击四个不同的场地角点。")
        if abs(float(cv2.contourArea(self.world_points))) < 1e-9:
            raise ValueError("世界坐标四点退化，无法计算 Homography。")
        self.H_image_to_world = cv2.getPerspectiveTransform(
            self.image_points, self.world_points
        )
        self.H_world_to_image = cv2.getPerspectiveTransform(
            self.world_points, self.image_points
        )

    def image_to_world(self, uv) -> tuple[float, float]:
        u, v = _as_xy(uv)
        pts = np.array([[[u, v]]], dtype=np.float32)
        mapped = cv2.perspectiveTransform(pts, self.H_image_to_world)
        result = mapped[0, 0]
        if not np.isfinite(result).all():
            raise ValueError(f"像素点 {uv!r} 无法映射到有限的世界坐标。")
        return float(result[0]), float(result[1])

    def world_to_image(self, xy) -> tuple[float, float]:
        x, y = _as_xy(xy)
        pts = np.array([[[x, y]]], dtype=np.float32)
        mapped = cv2.perspectiveTransform(pts, self.H_world_to_image)
        result = mapped[0, 0]
        if not np.isfinite(result).all():
            raise ValueError(f"世界点 {xy!r} 无法映射到有限的图像坐标。")
        return float(result[0]), float(result[1])

    def image_to_world_many(self, uvs: np.ndarray) -> np.ndarray:
        pts = np.asarray(uvs, dtype=np.float32).reshape(-1, 1, 2)
        mapped = cv2.perspectiveTransform(pts, self.H_image_to_world)
        return mapped.reshape(-1, 2)

    def world_corners(self) -> list[tuple[float, float]]:
        w, h = self.width_m, self.height_m
        return [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]

    def image_field_polygon(self) -> np.ndarray:
        corners = [self.world_to_image(xy) for xy in self.world_corners()]
        return np.array(corners, dtype=np.int32)

    def to_dict(self) -> dict:
        return {
            "width_m": self.width_m,
            "height_m": self.height_m,
            "image_points": self.image_points.astype(float).tolist(),
            "world_points": self.world_points.astype(float).tolist(),
            "H_image_to_world": self.H_image_to_world.astype(float).tolist(),
        }

    def save(self, path: Path | None = None) -> Path:
        out = path if path is not None else HOMOGRAPHY_PATH
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, allow_unicode=True, sort_keys=False)
        return out

    @classmethod
    def from_correspondences(
        cls,
        image_points,
        world_points,
        width_m: float,
        height_m: float,
    ) -> FieldTransform:
        return cls(
            image_points=image_points,
            world_points=world_points,
            width_m=width_m,
            height_m=height_m,
        )

    @classmethod
    def from_dict(cls, data: dict) -> FieldTransform:
        return cls(
            image_points=np.asarray(data["image_points"], dtype=np.float32),
            world_points=np.asarray(data["world_points"], dtype=np.float32),
            width_m=float(data["width_m"]),
            height_m=float(data["height_m"]),
        )

    @classmethod
    def load(cls, path: Path | None = None) -> FieldTransform:
        src = path if path is not None else HOMOGRAPHY_PATH
        if not src.exists():
            raise FileNotFoundError(
                f"未找到场地标定文件: {src}\n"
                "请先运行: python scripts/calibrate_field.py"
            )
        with src.open(encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if not data or "image_points" not in data or "world_points" not in data:
            raise ValueError(f"标定文件损坏或缺少 image_points/world_points: {src}")
        return cls.from_dict(data)

    @classmethod
    def try_load(cls, path: Path | None = None) -> FieldTransform | None:
        src = path if path is not None else HOMOGRAPHY_PATH
        if not src.exists():
            return None
        return cls.load(src)
