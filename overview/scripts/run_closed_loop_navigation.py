from __future__ import annotations

import argparse
import time
from math import radians

import cv2

import _bootstrap  # noqa: F401

from tennisbot.io.config import load_yaml
from tennisbot.io.paths import CONFIG_DIR
from tennisbot.navigation.driver import create_driver
from tennisbot.navigation.models import VelocityCommand
from tennisbot.navigation.navigator import ClosedLoopNavigator
from tennisbot.perception.models import RobotPose
from tennisbot.tracker import GlobalTracker
from tennisbot.visualization.overlay import draw_overlay, put_text_bg


def main() -> None:
    config = load_yaml(CONFIG_DIR / "navigation.yaml")
    default_target = config.get("target", {})
    parser = argparse.ArgumentParser(description="Vision closed-loop navigation")
    parser.add_argument("--x", type=float, default=float(default_target.get("x_m", 0.625)))
    parser.add_argument("--y", type=float, default=float(default_target.get("y_m", 1.0)))
    parser.add_argument("--theta", type=float, default=float(default_target.get("theta_deg", 0.0)))
    parser.add_argument("--enable-motion", action="store_true", help="allow real serial output; still starts disarmed")
    args = parser.parse_args()

    target = RobotPose(args.x, args.y, radians(args.theta))
    navigator = ClosedLoopNavigator(target, config)
    driver = create_driver(config, args.enable_motion)
    tracker = GlobalTracker(require_homography=True)
    armed = False
    driver.connect()
    cv2.namedWindow("Closed Loop Navigation", cv2.WINDOW_NORMAL)
    try:
        while True:
            result = tracker.process_frame(tracker.read_frame())
            output = navigator.update(result.world_state.robot, time.monotonic())
            driver.send(output.command if armed else VelocityCommand())
            command = output.command
            hud = [
                f"NAV={output.state.value}  {'ARMED' if armed else 'SAFE/DISARMED'}",
                f"Target=({target.x:.2f},{target.y:.2f},{args.theta:.1f}deg)",
                f"Cmd robot: vx={command.vx:+.3f} vy={command.vy:+.3f} w={command.omega:+.3f}",
                output.reason,
            ]
            vis = draw_overlay(result.frame, transform=tracker.transform, tags=result.tags, balls=result.balls, world_state=result.world_state, visible_ball_worlds=result.visible_world_balls, robot_tag=result.robot_tag, front_edge=tracker.tag_detector.front_edge, hud_lines=hud)
            target_uv = tracker.transform.world_to_image((target.x, target.y))
            cv2.drawMarker(vis, (int(target_uv[0]), int(target_uv[1])), (255, 0, 255), cv2.MARKER_CROSS, 30, 3)
            put_text_bg(vis, "TARGET", (int(target_uv[0]) + 12, int(target_uv[1]) - 12), color=(255, 0, 255))
            put_text_bg(vis, "G arm/start   SPACE emergency stop   Q quit", (12, vis.shape[0] - 16), scale=0.55)
            cv2.imshow("Closed Loop Navigation", vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("g"), ord("G")) and not armed:
                driver.enable()
                navigator.reset()
                armed = True
                print("ARMED: navigation output enabled")
            if key == 32:
                armed = False
                driver.stop()
                navigator.reset()
                print("EMERGENCY STOP: disarmed")
    finally:
        driver.close()
        tracker.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
