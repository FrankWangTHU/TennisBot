"""Camo Camera entry point (iPhone as a Windows virtual webcam)."""

from __future__ import annotations

from typing import Any

from tennisbot.camera.camera_source import CameraSource


def open_camo_camera(
    index: int | None = None,
    camera_cfg: dict[str, Any] | None = None,
) -> CameraSource:
    """Open the Camo virtual camera using config/camera.yaml by default."""
    source = CameraSource.from_config(camera_cfg)
    if index is not None:
        source.index = int(index)
    source.open()
    return source
