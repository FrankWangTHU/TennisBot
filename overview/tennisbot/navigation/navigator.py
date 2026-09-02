from __future__ import annotations

from tennisbot.navigation.controller import MecanumPoseController
from tennisbot.navigation.latency import predict_pose
from tennisbot.navigation.models import NavigationOutput, NavigationState, VelocityCommand
from tennisbot.perception.models import RobotPose


class ClosedLoopNavigator:
    """Pose controller with delay prediction, slew limits, and loss stop."""

    def __init__(self, target: RobotPose, config: dict) -> None:
        controller_cfg = config.get("controller", {})
        latency_cfg = config.get("latency", {})
        self.target = target
        self.controller = MecanumPoseController(controller_cfg)
        self.measurement_latency = float(latency_cfg.get("measurement_latency_s", 0.0))
        self.pose_loss_grace = float(latency_cfg.get("pose_loss_grace_s", 0.25))
        self.settle_time = float(controller_cfg.get("settle_time_s", 0.6))
        self.linear_accel = float(controller_cfg.get("max_linear_accel_mps2", 0.12))
        self.angular_accel = float(controller_cfg.get("max_angular_accel_radps2", 0.8))
        self.last_command = VelocityCommand()
        self.last_pose: RobotPose | None = None
        self.last_pose_time: float | None = None
        self.last_update_time: float | None = None
        self.settle_started: float | None = None

    def reset(self) -> None:
        self.last_command = VelocityCommand()
        self.last_pose = None
        self.last_pose_time = None
        self.last_update_time = None
        self.settle_started = None

    @staticmethod
    def _approach(value: float, target: float, limit: float) -> float:
        return max(value - limit, min(value + limit, target))

    def _slew(self, wanted: VelocityCommand, dt: float) -> VelocityCommand:
        linear_step = max(0.0, self.linear_accel * dt)
        angular_step = max(0.0, self.angular_accel * dt)
        return VelocityCommand(
            self._approach(self.last_command.vx, wanted.vx, linear_step),
            self._approach(self.last_command.vy, wanted.vy, linear_step),
            self._approach(self.last_command.omega, wanted.omega, angular_step),
        )

    def update(self, measured_pose: RobotPose | None, now: float) -> NavigationOutput:
        dt = 0.05 if self.last_update_time is None else max(0.001, min(0.25, now - self.last_update_time))
        self.last_update_time = now

        if measured_pose is not None:
            self.last_pose = measured_pose
            self.last_pose_time = now
            estimated = predict_pose(measured_pose, self.last_command, self.measurement_latency)
            state = NavigationState.TRACKING
        elif self.last_pose is not None and self.last_pose_time is not None:
            missing_s = now - self.last_pose_time
            if missing_s <= self.pose_loss_grace:
                estimated = predict_pose(self.last_pose, self.last_command, self.measurement_latency + missing_s)
                state = NavigationState.PREDICTING
            else:
                self.last_command = VelocityCommand()
                self.settle_started = None
                return NavigationOutput(self.last_command, NavigationState.POSE_LOST, None, None, None, f"AprilTag lost for {missing_s:.2f}s; stopped")
        else:
            self.last_command = VelocityCommand()
            return NavigationOutput(self.last_command, NavigationState.POSE_LOST, None, None, None, "No AprilTag pose; stopped")

        distance, heading = self.controller.errors(estimated, self.target)
        if self.controller.at_target(estimated, self.target):
            self.last_command = VelocityCommand()
            if self.settle_started is None:
                self.settle_started = now
            settled = now - self.settle_started >= self.settle_time
            return NavigationOutput(self.last_command, NavigationState.ARRIVED if settled else NavigationState.SETTLING, estimated, distance, heading, "Target reached" if settled else "Inside tolerance; settling")

        self.settle_started = None
        self.last_command = self._slew(self.controller.compute(estimated, self.target), dt)
        reason = "Predicted through brief tag loss" if state == NavigationState.PREDICTING else "Tracking"
        return NavigationOutput(self.last_command, state, estimated, distance, heading, reason)
