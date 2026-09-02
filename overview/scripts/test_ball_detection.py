"""Live tennis-ball detection. Shows world (x,y) if homography exists."""

from __future__ import annotations

import time

import cv2

import _bootstrap  # noqa: F401

from tennisbot.calibration.homography import FieldTransform
from tennisbot.camera.camo_camera import open_camo_camera
from tennisbot.io.config import load_perception_config
from tennisbot.perception.ball_detector import BallDetector
from tennisbot.perception.models import BallWorld, WorldState
from tennisbot.visualization.overlay import draw_overlay


def main() -> None:
    perception = load_perception_config()
    detector = BallDetector.from_config(perception)
    transform = FieldTransform.try_load()
    cam = open_camo_camera()
    window = "Ball Detection"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    if transform is None:
        print("尚未标定场地：只显示像素坐标。标定后会显示世界坐标 (m)。")
    else:
        print("已加载 Homography，将输出网球世界坐标。")
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
            balls, _mask = detector.detect(frame)
            world_balls: list[BallWorld] = []
            if transform is not None:
                paired = []
                for ball in balls:
                    x, y = transform.image_to_world(ball.center_uv)
                    paired.append((ball, BallWorld(x=x, y=y)))
                paired.sort(key=lambda item: (item[1].x, item[1].y))
                balls = [item[0] for item in paired]
                world_balls = [item[1] for item in paired]
            hud = [f"FPS={fps:.1f}  balls={len(balls)}"]
            vis = draw_overlay(
                frame,
                transform=transform,
                balls=balls,
                world_state=WorldState(robot=None, balls=world_balls),
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
