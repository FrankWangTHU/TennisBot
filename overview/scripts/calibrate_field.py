"""Click four field corners to save Homography."""

from __future__ import annotations

import cv2
import numpy as np

import _bootstrap  # noqa: F401

from tennisbot.calibration.field_calibration import (
    FIELD_CORNER_LABELS,
    build_transform_from_clicks,
    save_homography,
)
from tennisbot.camera.camo_camera import open_camo_camera
from tennisbot.io.config import load_field_config
from tennisbot.visualization.overlay import draw_field, put_text_bg


def main() -> None:
    field = load_field_config()
    width_m = float(field["width_m"])
    height_m = float(field["height_m"])
    cam = open_camo_camera()
    window = "Calibrate Field"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    points: list[tuple[float, float]] = []

    def on_mouse(event, x, y, _flags, _param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append((float(x), float(y)))
            print(f"点 {len(points)}: ({x}, {y})  -> {FIELD_CORNER_LABELS[len(points) - 1]}")

    cv2.setMouseCallback(window, on_mouse)
    print(f"场地尺寸: {width_m} m x {height_m} m")
    print("按顺序左键点击场地四角:")
    for i, label in enumerate(FIELD_CORNER_LABELS, start=1):
        print(f"  {i}. {label}")
    print("U 撤销上一点   R 全部重来   S 保存   Q/ESC 退出")
    transform = None
    try:
        while True:
            frame = cam.read()
            vis = frame.copy()
            for i, (u, v) in enumerate(points):
                cv2.circle(vis, (int(u), int(v)), 8, (0, 255, 255), -1)
                put_text_bg(vis, f"{i + 1} {FIELD_CORNER_LABELS[i]}", (int(u) + 10, int(v) - 10), scale=0.5)
            if len(points) >= 2:
                pts = np.array(points, dtype=np.int32).reshape(-1, 1, 2)
                cv2.polylines(vis, [pts], isClosed=len(points) == 4, color=(0, 255, 255), thickness=2)
            if len(points) == 4:
                try:
                    transform = build_transform_from_clicks(points, width_m, height_m)
                    draw_field(vis, transform)
                    put_text_bg(vis, "4 points ready  press S to save", (12, 86), color=(0, 255, 0))
                except Exception as exc:
                    transform = None
                    put_text_bg(vis, f"Homography failed: {exc}", (12, 86), color=(0, 0, 255))
            nxt = FIELD_CORNER_LABELS[len(points)] if len(points) < 4 else "press S to save"
            put_text_bg(vis, f"Next click: {nxt}", (12, 28))
            put_text_bg(vis, "U undo  R reset  S save  Q quit", (12, 56), scale=0.5)
            cv2.imshow(window, vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("u"), ord("U")) and points:
                points.pop()
                transform = None
            if key in (ord("r"), ord("R")):
                points.clear()
                transform = None
            if key in (ord("s"), ord("S")):
                if transform is None:
                    print("还没有 4 个有效点，无法保存。")
                    continue
                path = save_homography(transform)
                print(f"已保存 Homography: {path}")
                put_text_bg(vis, f"Saved {path.name}", (12, 114), color=(0, 255, 0))
                cv2.imshow(window, vis)
                cv2.waitKey(600)
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
