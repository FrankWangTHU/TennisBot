from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from tennisbot.perception.models import RobotPose


@dataclass(frozen=True)
class VelocityCommand:
    """Robot-frame command: +vx forward, +vy left, +omega counter-clockwise."""

    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0

    @property
    def is_stopped(self) -> bool:
        return max(abs(self.vx), abs(self.vy), abs(self.omega)) < 1e-6


class NavigationState(str, Enum):
    TRACKING = "tracking"
    PREDICTING = "predicting"
    POSE_LOST = "pose_lost"
    SETTLING = "settling"
    ARRIVED = "arrived"


@dataclass(frozen=True)
class NavigationOutput:
    command: VelocityCommand
    state: NavigationState
    estimated_pose: RobotPose | None
    position_error_m: float | None
    heading_error_rad: float | None
    reason: str

