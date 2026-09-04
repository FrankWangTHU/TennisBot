from math import pi

import pytest

from tennisbot.navigation.controller import MecanumPoseController
from tennisbot.navigation.latency import predict_pose
from tennisbot.navigation.models import NavigationState, VelocityCommand
from tennisbot.navigation.navigator import ClosedLoopNavigator
from tennisbot.perception.models import RobotPose


CONFIG = {
    "controller": {
        "kp_xy": 1.0,
        "kp_theta": 2.0,
        "max_linear_mps": 0.1,
        "max_angular_radps": 0.5,
        "max_linear_accel_mps2": 1.0,
        "max_angular_accel_radps2": 2.0,
        "position_tolerance_m": 0.05,
        "heading_tolerance_deg": 5,
        "settle_time_s": 0.2,
    },
    "latency": {"measurement_latency_s": 1.0, "pose_loss_grace_s": 0.25},
}


def test_controller_rotates_world_error_into_robot_frame() -> None:
    controller = MecanumPoseController(CONFIG["controller"])
    command = controller.compute(RobotPose(0, 0, pi / 2), RobotPose(1, 0, pi / 2))
    assert command.vx == pytest.approx(0.0, abs=1e-8)
    assert command.vy == pytest.approx(-0.1)


def test_prediction_uses_robot_frame_velocity() -> None:
    pose = predict_pose(RobotPose(0, 0, pi / 2), VelocityCommand(vx=0.1), 1.0)
    assert pose.x == pytest.approx(0.0, abs=1e-8)
    assert pose.y == pytest.approx(0.1)


def test_tag_loss_has_short_prediction_then_hard_stop() -> None:
    nav = ClosedLoopNavigator(RobotPose(1, 0, 0), CONFIG)
    first = nav.update(RobotPose(0, 0, 0), 10.0)
    assert first.state == NavigationState.TRACKING
    short_loss = nav.update(None, 10.1)
    assert short_loss.state == NavigationState.PREDICTING
    long_loss = nav.update(None, 10.3)
    assert long_loss.state == NavigationState.POSE_LOST
    assert long_loss.command.is_stopped


def test_arrival_requires_settle_time() -> None:
    nav = ClosedLoopNavigator(RobotPose(0, 0, 0), CONFIG)
    assert nav.update(RobotPose(0, 0, 0), 1.0).state == NavigationState.SETTLING
    assert nav.update(RobotPose(0, 0, 0), 1.25).state == NavigationState.ARRIVED


def test_position_only_mode_never_commands_rotation() -> None:
    config = {**CONFIG, "controller": {**CONFIG["controller"], "heading_control_enabled": False}}
    nav = ClosedLoopNavigator(RobotPose(1, 0, pi), config)
    output = nav.update(RobotPose(0, 0, 0), 1.0)
    assert output.command.vx > 0
    assert output.command.omega == 0
