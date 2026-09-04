from __future__ import annotations

from math import cos, hypot, radians, sin

from tennisbot.navigation.latency import wrap_angle
from tennisbot.navigation.models import VelocityCommand
from tennisbot.perception.models import RobotPose


class MecanumPoseController:
    def __init__(self, config: dict) -> None:
        self.kp_xy = float(config.get("kp_xy", 1.0))
        self.kp_theta = float(config.get("kp_theta", 1.8))
        self.max_linear = float(config.get("max_linear_mps", 0.08))
        self.max_angular = float(config.get("max_angular_radps", 0.45))
        self.position_tolerance = float(config.get("position_tolerance_m", 0.05))
        self.heading_tolerance = radians(float(config.get("heading_tolerance_deg", 8.0)))
        self.heading_enabled = bool(config.get("heading_control_enabled", True))

    def errors(self, pose: RobotPose, target: RobotPose) -> tuple[float, float]:
        heading = wrap_angle(target.theta - pose.theta) if self.heading_enabled else 0.0
        return hypot(target.x - pose.x, target.y - pose.y), heading

    def at_target(self, pose: RobotPose, target: RobotPose) -> bool:
        distance, heading = self.errors(pose, target)
        return distance <= self.position_tolerance and abs(heading) <= self.heading_tolerance

    def compute(self, pose: RobotPose, target: RobotPose) -> VelocityCommand:
        ex, ey = target.x - pose.x, target.y - pose.y
        c, s = cos(pose.theta), sin(pose.theta)
        vx = self.kp_xy * (c * ex + s * ey)
        vy = self.kp_xy * (-s * ex + c * ey)
        speed = hypot(vx, vy)
        if speed > self.max_linear:
            scale = self.max_linear / speed
            vx, vy = vx * scale, vy * scale
        omega = self.kp_theta * wrap_angle(target.theta - pose.theta) if self.heading_enabled else 0.0
        omega = max(-self.max_angular, min(self.max_angular, omega))
        return VelocityCommand(vx, vy, omega)
