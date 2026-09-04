from __future__ import annotations

import argparse
import time
from math import radians

import cv2

import _bootstrap  # noqa: F401

from tennisbot.io.config import load_yaml
from tennisbot.io.paths import CONFIG_DIR, NAVIGATION_LOG_DIR
from tennisbot.navigation.driver import ChassisDriver, create_driver
from tennisbot.navigation.logging import NavigationCsvLogger
from tennisbot.navigation.models import NavigationState
from tennisbot.navigation.navigator import ClosedLoopNavigator
from tennisbot.perception.models import RobotPose
from tennisbot.tracker import GlobalTracker
from tennisbot.visualization.overlay import draw_overlay, put_text_bg


def safe_disable(driver: ChassisDriver) -> None:
    try:
        driver.stop()
    except Exception:
        pass
    try:
        driver.disable()
    except Exception as exc:
        print(f"警告：底盘失能确认失败，将依赖 ESP32 看门狗停车：{exc}")


def main() -> None:
    config = load_yaml(CONFIG_DIR / "navigation.yaml")
    default_target = config.get("target", {})
    safety_cfg = config.get("safety", {})
    parser = argparse.ArgumentParser(description="Vision + UDP/serial closed-loop navigation")
    parser.add_argument("--x", type=float, default=float(default_target.get("x_m", 0.625)))
    parser.add_argument("--y", type=float, default=float(default_target.get("y_m", 1.0)))
    parser.add_argument("--theta", type=float, default=float(default_target.get("theta_deg", 0.0)))
    parser.add_argument("--position-only", action="store_true", help="disable heading control for first translation tests")
    parser.add_argument("--enable-motion", action="store_true", help="allow real output; program still starts disarmed")
    parser.add_argument("--no-log", action="store_true")
    args = parser.parse_args()

    if args.position_only:
        config.setdefault("controller", {})["heading_control_enabled"] = False
    target = RobotPose(args.x, args.y, radians(args.theta))
    min_pose_frames = int(safety_cfg.get("min_pose_frames_to_arm", 5))
    navigator = ClosedLoopNavigator(target, config)
    driver = create_driver(config, args.enable_motion)
    tracker = GlobalTracker(require_homography=True)
    logger = None if args.no_log else NavigationCsvLogger(NAVIGATION_LOG_DIR)
    window = "Closed Loop Navigation"
    armed = False
    pose_streak = 0
    status = "Waiting for stable AprilTag pose"
    fps = 0.0
    last_frame_time = time.perf_counter()

    if not (0.0 <= target.x <= tracker.transform.width_m and 0.0 <= target.y <= tracker.transform.height_m):
        tracker.close()
        if logger is not None:
            logger.close()
        raise SystemExit(
            f"目标 ({target.x:.2f},{target.y:.2f}) 超出场地 "
            f"0..{tracker.transform.width_m:.2f} x 0..{tracker.transform.height_m:.2f} m"
        )

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    try:
        driver.connect()
        while True:
            result = tracker.process_frame(tracker.read_frame())
            now = time.monotonic()
            frame_dt = max(1e-6, time.perf_counter() - last_frame_time)
            last_frame_time = time.perf_counter()
            current_fps = 1.0 / frame_dt
            fps = current_fps if fps <= 0 else 0.9 * fps + 0.1 * current_fps

            measured_pose = result.world_state.robot
            pose_streak = pose_streak + 1 if measured_pose is not None else 0
            output = navigator.update(measured_pose, now)

            if armed and output.state == NavigationState.POSE_LOST:
                safe_disable(driver)
                armed = False
                navigator.reset()
                status = "AUTO STOP: AprilTag lost; press G after pose is stable"
            elif armed:
                try:
                    driver.send(output.command)
                except (RuntimeError, TimeoutError, OSError) as exc:
                    safe_disable(driver)
                    armed = False
                    navigator.reset()
                    status = f"AUTO STOP: control link error: {exc}"
                if armed and output.state == NavigationState.ARRIVED:
                    safe_disable(driver)
                    armed = False
                    status = "ARRIVED: motors disabled"

            command = output.command if armed else type(output.command)()
            if logger is not None:
                logger.write(now, armed, fps, measured_pose, target, output, command)

            heading_mode = "position-only" if args.position_only else "position+heading"
            hud = [
                f"NAV={output.state.value}  {'ARMED' if armed else 'SAFE/DISARMED'}  FPS={fps:.1f}",
                f"Target=({target.x:.2f},{target.y:.2f},{target.theta_deg:.1f}deg)  mode={heading_mode}",
                f"Pose ready={pose_streak}/{min_pose_frames}  error={output.position_error_m if output.position_error_m is not None else -1:.3f}m",
                f"Sent: vx={command.vx:+.3f} vy={command.vy:+.3f} w={command.omega:+.3f}",
                status,
            ]
            vis = draw_overlay(
                result.frame,
                transform=tracker.transform,
                tags=result.tags,
                balls=result.balls,
                world_state=result.world_state,
                visible_ball_worlds=result.visible_world_balls,
                robot_tag=result.robot_tag,
                front_edge=tracker.tag_detector.front_edge,
                hud_lines=hud,
            )
            target_uv = tracker.transform.world_to_image((target.x, target.y))
            target_pixel = (int(target_uv[0]), int(target_uv[1]))
            cv2.drawMarker(vis, target_pixel, (255, 0, 255), cv2.MARKER_CROSS, 30, 3)
            put_text_bg(vis, "TARGET", (target_pixel[0] + 12, target_pixel[1] - 12), color=(255, 0, 255))
            put_text_bg(vis, "G arm/start   SPACE stop+disable   Q quit", (12, vis.shape[0] - 16), scale=0.55)
            cv2.imshow(window, vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("g"), ord("G")) and not armed:
                if pose_streak < min_pose_frames:
                    status = f"ARM REFUSED: need {min_pose_frames} consecutive AprilTag frames"
                    print(status)
                else:
                    try:
                        driver.enable()
                        navigator.reset()
                        armed = True
                        status = "ARMED: closed-loop output enabled"
                        print(status)
                    except (RuntimeError, TimeoutError, OSError) as exc:
                        safe_disable(driver)
                        status = f"ARM FAILED: {exc}"
                        print(status)
            if key == 32:
                safe_disable(driver)
                armed = False
                navigator.reset()
                status = "EMERGENCY STOP: motors disabled"
                print(status)
    except KeyboardInterrupt:
        print("Ctrl+C: stopping")
    except (RuntimeError, TimeoutError, OSError) as exc:
        print(f"闭环导航安全退出：{exc}")
    finally:
        safe_disable(driver)
        driver.close()
        tracker.close()
        if logger is not None:
            logger.close()
            print(f"导航日志已保存：{logger.path}")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
