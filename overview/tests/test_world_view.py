import pytest

from tennisbot.visualization.world_view import world_to_canvas


def test_world_view_uses_equal_pixels_per_metre_on_both_axes() -> None:
    origin = world_to_canvas(0.0, 0.0, 2.5, 1.25, 1100, 650)
    x_one = world_to_canvas(1.0, 0.0, 2.5, 1.25, 1100, 650)
    y_one = world_to_canvas(0.0, 1.0, 2.5, 1.25, 1100, 650)
    x_pixels = x_one[0] - origin[0]
    y_pixels = origin[1] - y_one[1]
    assert x_pixels == pytest.approx(y_pixels, abs=1)


def test_world_view_preserves_two_to_one_field_ratio() -> None:
    bottom_left = world_to_canvas(0.0, 0.0, 2.5, 1.25, 1100, 650)
    bottom_right = world_to_canvas(2.5, 0.0, 2.5, 1.25, 1100, 650)
    top_left = world_to_canvas(0.0, 1.25, 2.5, 1.25, 1100, 650)
    pixel_width = bottom_right[0] - bottom_left[0]
    pixel_height = bottom_left[1] - top_left[1]
    assert pixel_width / pixel_height == pytest.approx(2.0, abs=0.01)
