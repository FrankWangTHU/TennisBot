"""HSV trackbars for tennis-ball detection. Press S to save YAML."""

from __future__ import annotations

import cv2
import numpy as np

import _bootstrap  # noqa: F401

from tennisbot.camera.camo_camera import open_camo_camera
from tennisbot.io.config import load_perception_config, save_yaml
from tennisbot.io.paths import PERCEPTION_CONFIG_PATH
from tennisbot.perception.ball_detector import BallDetector
from tennisbot.visualization.overlay import draw_overlay, put_text_bg


def _tb(name: str, window: str) -> int:
    return int(cv2.getTrackbarPos(name, window))


def main() -> None:
    perception = load_perception_config()
    detector = BallDetector.from_config(perception)
    cam = open_camo_camera()
    win = "Tune Ball HSV"
    mask_win = "HSV Mask"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.namedWindow(mask_win, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("H min", win, int(detector.hsv_lower[0]), 179, lambda _v: None)
    cv2.createTrackbar("H max", win, int(detector.hsv_upper[0]), 179, lambda _v: None)
    cv2.createTrackbar("S min", win, int(detector.hsv_lower[1]), 255, lambda _v: None)
    cv2.createTrackbar("S max", win, int(detector.hsv_upper[1]), 255, lambda _v: None)
    cv2.createTrackbar("V min", win, int(detector.hsv_lower[2]), 255, lambda _v: None)
    cv2.createTrackbar("V max", win, int(detector.hsv_upper[2]), 255, lambda _v: None)
    print("拖动滑条直到 mask 里只剩网球。按 S 保存到 config/perception.yaml，Q/ESC 退出。")
    try:
        while True:
            frame = cam.read()
            detector.hsv_lower = np.array([
                _tb("H min", win),
                _tb("S min", win),
                _tb("V min", win),
            ], dtype=np.uint8)
            detector.hsv_upper = np.array([
                _tb("H max", win),
                _tb("S max", win),
                _tb("V max", win),
            ], dtype=np.uint8)
            balls, mask = detector.detect(frame)
            vis = draw_overlay(frame, balls=balls)
            put_text_bg(vis, f"balls={len(balls)}  S save  Q quit", (12, 28))
            put_text_bg(
                vis,
                f"HSV lower={list(map(int, detector.hsv_lower))}  upper={list(map(int, detector.hsv_upper))}",
                (12, 56),
                scale=0.5,
            )
            cv2.imshow(win, vis)
            cv2.imshow(mask_win, mask)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S")):
                perception.setdefault("ball_detection", {})
                perception["ball_detection"]["hsv_lower"] = [int(x) for x in detector.hsv_lower]
                perception["ball_detection"]["hsv_upper"] = [int(x) for x in detector.hsv_upper]
                save_yaml(PERCEPTION_CONFIG_PATH, perception)
                print(f"已保存 HSV 到 {PERCEPTION_CONFIG_PATH}")
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
