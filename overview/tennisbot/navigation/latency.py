from __future__ import annotations

from math import cos, pi, sin

from tennisbot.navigation.models import VelocityCommand
from tennisbot.perception.models import RobotPose


def wrap_angle(angle: float) -> float:
    return (float(angle) + pi) % (2.0 * pi) - pi


def predict_pose(pose: RobotPose, command: VelocityCommand, delay_s: float) -> RobotPose:
    """Constant-command prediction across camera/processing latency."""
    dt = max(0.0, float(delay_s))
    theta_mid = pose.theta + command.omega * dt * 0.5
    world_vx = cos(theta_mid) * command.vx - sin(theta_mid) * command.vy
    world_vy = sin(theta_mid) * command.vx + cos(theta_mid) * command.vy
    return RobotPose(
        pose.x + world_vx * dt,
        pose.y + world_vy * dt,
        wrap_angle(pose.theta + command.omega * dt),
    )

