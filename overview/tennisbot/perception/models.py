"""Shared perception / localization dataclasses."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class AprilTagDetection:
    tag_id: int
    center_uv: tuple[float, float]
    corners_uv: np.ndarray  # (4, 2)
    heading_image_rad: float


@dataclass
class BallDetection:
    center_uv: tuple[float, float]
    radius_px: float
    area_px: float


@dataclass
class RobotPose:
    x: float
    y: float
    theta: float  # radian

    @property
    def theta_deg(self) -> float:
        return float(np.degrees(self.theta))


@dataclass
class BallWorld:
    x: float
    y: float
    track_id: int = -1
    visible: bool = True


@dataclass
class WorldState:
    robot: RobotPose | None = None
    balls: list[BallWorld] = field(default_factory=list)
