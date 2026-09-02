import numpy as np
import pytest

from tennisbot.calibration.homography import FieldTransform
from tennisbot.visualization.overlay import format_meters


def make_transform() -> FieldTransform:
    return FieldTransform.from_correspondences(
        image_points=[(120, 900), (1800, 820), (1500, 120), (360, 160)],
        world_points=[(0, 0), (5, 0), (5, 3), (0, 3)],
        width_m=5.0,
        height_m=3.0,
    )


def test_calibration_corners_map_both_ways() -> None:
    transform = make_transform()
    for uv, xy in zip(transform.image_points, transform.world_points):
        assert transform.image_to_world(uv) == pytest.approx(xy, abs=1e-5)
        assert transform.world_to_image(xy) == pytest.approx(uv, abs=1e-3)


def test_round_trip_for_interior_points() -> None:
    transform = make_transform()
    points = np.array([(0.4, 0.2), (2.5, 1.5), (4.7, 2.8)], dtype=np.float32)
    for xy in points:
        uv = transform.world_to_image(xy)
        assert transform.image_to_world(uv) == pytest.approx(xy, abs=1e-4)


def test_rejects_degenerate_calibration() -> None:
    with pytest.raises(ValueError, match="退化"):
        FieldTransform.from_correspondences(
            image_points=[(0, 0), (10, 0), (20, 0), (30, 0)],
            world_points=[(0, 0), (5, 0), (5, 3), (0, 3)],
            width_m=5.0,
            height_m=3.0,
        )


def test_field_dimension_keeps_second_decimal() -> None:
    assert format_meters(1.25) == "1.25"
    assert format_meters(2.0) == "2"
