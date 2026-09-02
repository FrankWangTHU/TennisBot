"""AprilTag live test.

Default: image-space ID / center / corners / heading.
With --world: also print RobotPose after homography is loaded.
"""

from __future__ import annotations

import argparse
import math
import time

import cv2

import _bootstrap  # noqa: F401

from tennisbot.calibration.homography import FieldTransform
from tennisbot.camera.camo_camera import open_camo_camera
from tennisbot.io.config import load_perception_config
from tennisbot.localization.robot_pose import pose_from_tag
from tennisbot.perception.apriltag_detector import AprilTagDetector
from tennisbot.visualization.overlay import draw_overlay


def main() -> None:
    parser = argparse.ArgumentParser(description="Test AprilTag detection")
    parser.add_argument(
        "--world",
        action="store_true",
        help="把 Tag 中心和朝向映射到场地世界坐标（需要先标定）",
    )
    args = parser.parse_args()

    perception = load_perception_config()
    detector = AprilTagDetector.from_config(perception)
    transform = FieldTransform.try_load() if args.world else None
    if args.world and transform is None:
        raise SystemExit(
            "未找到 Homography。请先运行: python scripts/calibrate_field.py"
        )

    cam = open_camo_camera()
    window = "AprilTag Test"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print(f"family={detector.family}  front_edge={detector.front_edge}  robot_tag_id={detector.robot_tag_id}")
    print("朝向约定: Tag 上边中点相对中心 = 小车前进方向（可用 perception.yaml 的 front_edge 修改）")
    if args.world:
        print("已加载世界坐标。将车摆到 0/90/180/270° 检查 theta 符号。")
    print("按 Q / ESC 退出。")

    last = time.perf_counter()
    fps = 0.0
    try:
        while True:
            frame = cam.read()
            now = time.perf_counter()
            dt = now - last
            last = now
            if dt > 1e-6:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else 1.0 / dt
            tags = detector.detect(frame)
            robot_tag = detector.select_robot_tag(tags)
            pose = None
            if args.world and transform is not None and robot_tag is not None:
                pose = pose_from_tag(robot_tag, transform, detector.front_edge)
            hud = [f"FPS={fps:.1f}  tags={len(tags)}"]
            if robot_tag is None:
                hud.append("Robot: not detected")
            else:
                deg = math.degrees(robot_tag.heading_image_rad)
                hud.append(
                    f"ID={robot_tag.tag_id}  center=({robot_tag.center_uv[0]:.1f},{robot_tag.center_uv[1]:.1f})  "
                    f"theta_image={deg:.1f}deg"
                )
                if pose is not None:
                    hud.append(
                        f"Robot: x={pose.x:.2f} m, y={pose.y:.2f} m, theta={pose.theta_deg:.1f} deg"
                    )
            from tennisbot.perception.models import WorldState

            vis = draw_overlay(
                frame,
                transform=transform,
                tags=tags,
                robot_tag=robot_tag,
                world_state=WorldState(robot=pose, balls=[]),
                front_edge=detector.front_edge,
                hud_lines=hud,
            )
            cv2.imshow(window, vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
