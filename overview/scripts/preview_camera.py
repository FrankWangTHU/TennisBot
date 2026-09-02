"""Live preview of the configured Camo / webcam."""

from __future__ import annotations

import time

import cv2

import _bootstrap  # noqa: F401

from tennisbot.camera.camo_camera import open_camo_camera
from tennisbot.visualization.overlay import put_text_bg


def main() -> None:
    cam = open_camo_camera()
    window = "Camera Preview"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    actual_w, actual_h = cam.actual_size()
    print(
        f"已打开摄像头 index={cam.index}，实际 {actual_w}x{actual_h} @ "
        f"{cam.actual_fps():.1f} FPS（请求 {cam.width}x{cam.height} @ {cam.fps}, {cam.fourcc}）"
    )
    print("按 Q 或 ESC 退出。")
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
            h, w = frame.shape[:2]
            vis = frame.copy()
            put_text_bg(vis, f"{w}x{h}  FPS={fps:.1f}  index={cam.index}", (12, 28))
            put_text_bg(vis, "Q/ESC quit", (12, 56), scale=0.5)
            cv2.imshow(window, vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
