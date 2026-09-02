"""Manual four-point field calibration helpers."""

from __future__ import annotations

from pathlib import Path

from tennisbot.calibration.homography import FieldTransform
from tennisbot.io.config import load_field_config
from tennisbot.io.paths import HOMOGRAPHY_PATH

FIELD_CORNER_LABELS = [
    "world (0, 0) 左下",
    "world (W, 0) 右下",
    "world (W, H) 右上",
    "world (0, H) 左上",
]


def field_world_corners(width_m: float, height_m: float) -> list[tuple[float, float]]:
    return [
        (0.0, 0.0),
        (width_m, 0.0),
        (width_m, height_m),
        (0.0, height_m),
    ]


def build_transform_from_clicks(
    image_points: list[tuple[float, float]],
    width_m: float | None = None,
    height_m: float | None = None,
) -> FieldTransform:
    field = load_field_config()
    w = float(width_m if width_m is not None else field["width_m"])
    h = float(height_m if height_m is not None else field["height_m"])
    if len(image_points) != 4:
        raise ValueError(f"需要 4 个点击点，当前 {len(image_points)} 个。")
    return FieldTransform.from_correspondences(
        image_points=image_points,
        world_points=field_world_corners(w, h),
        width_m=w,
        height_m=h,
    )


def save_homography(transform: FieldTransform, path: Path | None = None) -> Path:
    return transform.save(path if path is not None else HOMOGRAPHY_PATH)
