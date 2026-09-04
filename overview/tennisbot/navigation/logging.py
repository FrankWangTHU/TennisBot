from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path

from tennisbot.navigation.models import NavigationOutput, VelocityCommand
from tennisbot.perception.models import RobotPose


class NavigationCsvLogger:
    def __init__(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.path = directory / f"navigation-{stamp}.csv"
        self._stream = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._stream)
        self._writer.writerow([
            "monotonic_s", "armed", "state", "fps",
            "measured_x", "measured_y", "measured_theta_deg",
            "estimated_x", "estimated_y", "estimated_theta_deg",
            "target_x", "target_y", "target_theta_deg",
            "position_error_m", "heading_error_deg", "vx", "vy", "omega",
        ])

    @staticmethod
    def _pose_values(pose: RobotPose | None) -> tuple[object, object, object]:
        if pose is None:
            return "", "", ""
        return f"{pose.x:.6f}", f"{pose.y:.6f}", f"{pose.theta_deg:.4f}"

    def write(
        self,
        now: float,
        armed: bool,
        fps: float,
        measured: RobotPose | None,
        target: RobotPose,
        output: NavigationOutput,
        sent_command: VelocityCommand,
    ) -> None:
        heading_deg = "" if output.heading_error_rad is None else f"{output.heading_error_rad * 180.0 / 3.141592653589793:.4f}"
        self._writer.writerow([
            f"{now:.6f}", int(armed), output.state.value, f"{fps:.3f}",
            *self._pose_values(measured), *self._pose_values(output.estimated_pose),
            *self._pose_values(target),
            "" if output.position_error_m is None else f"{output.position_error_m:.6f}",
            heading_deg,
            f"{sent_command.vx:.6f}", f"{sent_command.vy:.6f}", f"{sent_command.omega:.6f}",
        ])
        self._stream.flush()

    def close(self) -> None:
        self._stream.close()
