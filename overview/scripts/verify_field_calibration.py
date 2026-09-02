"""Click image locations of known ground-truth field points and print error."""

from __future__ import annotations

import cv2

import _bootstrap  # noqa: F401

from tennisbot.calibration.homography import FieldTransform
from tennisbot.camera.camo_camera import open_camo_camera
from tennisbot.io.config import load_field_config
from tennisbot.visualization.overlay import draw_overlay, put_text_bg


def main() -> None:
    transform = FieldTransform.load()
    field = load_field_config()
    gt_points = [tuple(map(float, p)) for p in field.get("verification_points", [])]
    if not gt_points:
        raise SystemExit("config/field.yaml 缺少 field.verification_points")

    cam = open_camo_camera()
    window = "Verify Field Calibration"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    index = 0
    last_click: tuple[float, float] | None = None
    last_est: tuple[float, float] | None = None
    last_err: float | None = None
    print("把网球或 marker 放到当前 GT 点，然后在画面上点击该点。")
    print("N 下一个点  P 上一个点  Q 退出")
    for i, (x, y) in enumerate(gt_points):
        print(f"  GT[{i}]: ({x:.3f}, {y:.3f}) m")

    def on_mouse(event, x, y, _flags, _param) -> None:
        nonlocal last_click, last_est, last_err
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        last_click = (float(x), float(y))
        last_est = transform.image_to_world(last_click)
        gx, gy = gt_points[index]
        last_err = ((last_est[0] - gx) ** 2 + (last_est[1] - gy) ** 2) ** 0.5
        print(
            f"GT:        ({gx:.3f}, {gy:.3f})\n"
            f"Estimated: ({last_est[0]:.3f}, {last_est[1]:.3f})\n"
            f"Error:     {last_err:.3f} m"
        )
        if last_err > 0.10:
            print("误差 > 10 cm，检查点击是否对准、相机是否被移动、四角标定是否准确。")
        elif last_err > 0.05:
            print("误差在 5-10 cm，V1 可接受，后续可再精标。")
        else:
            print("误差 < 5 cm。")

    cv2.setMouseCallback(window, on_mouse)
    try:
        while True:
            frame = cam.read()
            vis = draw_overlay(frame, transform=transform)
            gx, gy = gt_points[index]
            gu, gv = transform.world_to_image((gx, gy))
            cv2.drawMarker(vis, (int(gu), int(gv)), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
            put_text_bg(vis, f"Click GT[{index}]: ({gx:.3f}, {gy:.3f}) m", (12, 28))
            put_text_bg(vis, "N next  P prev  Q quit", (12, 56), scale=0.5)
            if last_est is not None and last_err is not None and last_click is not None:
                cv2.circle(vis, (int(last_click[0]), int(last_click[1])), 7, (0, 0, 255), -1)
                put_text_bg(
                    vis,
                    f"Est ({last_est[0]:.3f},{last_est[1]:.3f})  err={last_err:.3f} m",
                    (12, 84),
                    color=(0, 255, 255),
                )
            cv2.imshow(window, vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("n"), ord("N")):
                index = (index + 1) % len(gt_points)
                last_click = last_est = last_err = None
            if key in (ord("p"), ord("P")):
                index = (index - 1) % len(gt_points)
                last_click = last_est = last_err = None
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
