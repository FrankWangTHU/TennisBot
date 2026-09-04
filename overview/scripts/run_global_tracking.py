"""Integrated V1 global tracking: robot pose + balls + field overlay + bird's-eye view."""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

import _bootstrap  # noqa: F401

from tennisbot.io.debug import save_debug_snapshot
from tennisbot.io.config import save_yaml
from tennisbot.io.paths import PERCEPTION_CONFIG_PATH
from tennisbot.perception.ball_auto_tuner import BallAutoTuner
from tennisbot.tracker import GlobalTracker
from tennisbot.visualization.overlay import draw_overlay, format_meters, put_text_bg
from tennisbot.visualization.world_view import render_world_view


def _save_ball_settings(tracker: GlobalTracker) -> None:
    detector = tracker.ball_detector
    perception = tracker.perception_cfg
    cfg = perception.setdefault("ball_detection", {})
    cfg["hsv_lower"] = [int(v) for v in detector.hsv_lower]
    cfg["hsv_upper"] = [int(v) for v in detector.hsv_upper]
    cfg["min_area"] = float(detector.min_area)
    cfg["max_area"] = float(detector.max_area)
    cfg["min_circularity"] = float(detector.min_circularity)
    cfg["morph_kernel"] = int(detector.morph_kernel)
    save_yaml(PERCEPTION_CONFIG_PATH, perception)


def _render_auto_tuning_panel(
    tracker: GlobalTracker,
    tuner: BallAutoTuner,
    status: str,
) -> np.ndarray:
    panel = np.full((220, 760, 3), (35, 35, 35), dtype=np.uint8)
    detector = tracker.ball_detector
    lines = [
        "Automatic ball tuning (no sliders)",
        "1. Left-click 2-5 real balls in Global Tracking",
        "2. Each click learns + applies + saves automatically",
        "Right-click: clear marks    U: undo last mark    H: mask",
        f"Marked={len(tuner.points)}  HSV={detector.hsv_lower.tolist()} -> {detector.hsv_upper.tolist()}",
        f"Min area={detector.min_area:.0f}  round={detector.min_circularity:.2f}  {status}",
    ]
    for i, line in enumerate(lines):
        color = (80, 230, 255) if i == len(lines) - 1 else (255, 255, 255)
        put_text_bg(panel, line, (12, 30 + i * 34), scale=0.52, color=color)
    return panel


def main() -> None:
    parser = argparse.ArgumentParser(description="Global robot and tennis-ball tracking")
    parser.add_argument(
        "--tune-balls",
        action="store_true",
        help="enable click-to-learn ball colors and show the tuning help window",
    )
    args = parser.parse_args()

    tracker = GlobalTracker(require_homography=True)
    live_win = "Global Tracking"
    tuning_win = "Ball Auto Tuning"
    world_win = "World View"
    mask_win = "HSV Mask"
    cv2.namedWindow(live_win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(live_win, 1280, 720)
    if args.tune_balls:
        cv2.namedWindow(tuning_win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(tuning_win, 760, 250)
    show_world = False
    show_mask = False
    latest_raw_frame: np.ndarray | None = None
    auto_tuner = BallAutoTuner()
    tuning_status = "Waiting for ball marks"

    def sample_ball_color(event, x, y, _flags, _param) -> None:
        nonlocal tuning_status
        if latest_raw_frame is None:
            return
        if event == cv2.EVENT_RBUTTONDOWN:
            auto_tuner.clear()
            tuning_status = "Marks cleared; current saved settings kept"
            return
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        h, w = latest_raw_frame.shape[:2]
        try:
            _rx, _ry, display_w, display_h = cv2.getWindowImageRect(live_win)
            raw_x = int(round(x * w / max(1, display_w)))
            raw_y = int(round(y * h / max(1, display_h)))
            result = auto_tuner.add_ball(latest_raw_frame, raw_x, raw_y)
            auto_tuner.apply(tracker.ball_detector)
            _save_ball_settings(tracker)
            tuning_status = (
                f"Saved: {result.marked_balls} balls, {result.sampled_pixels} color pixels"
            )
            print(
                f"自动调参并保存: marked={result.marked_balls}, "
                f"HSV={result.hsv_lower}->{result.hsv_upper}, min_area={result.min_area:.0f}"
            )
        except ValueError as exc:
            tuning_status = f"Click rejected: {exc}"
            print(tuning_status)

    if args.tune_balls:
        cv2.setMouseCallback(live_win, sample_ball_color)
        print("网球调参模式：在主窗口依次左键点击 2-5 颗真实网球，程序会自动调整并保存。")
    print("快捷键: Q/ESC 退出  S 保存debug  H mask  W 等比例俯视图  R 重载")
    last = time.perf_counter()
    fps = 0.0
    try:
        while True:
            frame = tracker.read_frame()
            latest_raw_frame = frame
            result = tracker.process_frame(frame)
            now = time.perf_counter()
            dt = now - last
            last = now
            if dt > 1e-6:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else 1.0 / dt
            state = result.world_state
            visible_count = sum(ball.visible for ball in state.balls)
            memory_count = len(state.balls) - visible_count
            hud = [
                f"FPS={fps:.1f}  balls={len(state.balls)} "
                f"(visible={visible_count}, memory={memory_count}) "
                f"ball_scale={tracker.processing_scale:.2f} tag_scale={tracker.apriltag_scale:.2f}",
                f"Field: {format_meters(tracker.transform.width_m)} x "
                f"{format_meters(tracker.transform.height_m)} m  (oblique homography)",
                f"HSV: {tracker.ball_detector.hsv_lower.tolist()} -> "
                f"{tracker.ball_detector.hsv_upper.tolist()}",
            ]
            if state.robot is None:
                hud.append("Robot: not detected")
            else:
                r = state.robot
                hud.append(f"Robot: x={r.x:.2f} m, y={r.y:.2f} m, theta={r.theta_deg:.1f} deg")
            vis = draw_overlay(
                result.frame,
                transform=tracker.transform,
                tags=result.tags,
                balls=result.balls,
                world_state=state,
                visible_ball_worlds=result.visible_world_balls,
                robot_tag=result.robot_tag,
                front_edge=tracker.tag_detector.front_edge,
                hud_lines=hud,
            )
            if args.tune_balls:
                for i, (px, py) in enumerate(auto_tuner.points, start=1):
                    cv2.drawMarker(vis, (px, py), (255, 0, 255), cv2.MARKER_CROSS, 28, 3)
                    put_text_bg(vis, f"sample {i}", (px + 12, py - 12), scale=0.45, color=(255, 0, 255))
            footer = "Q quit  S debug  H mask  W proportional world  R reload"
            if args.tune_balls:
                footer += "  Left-click tune  U undo"
            put_text_bg(vis, footer, (12, vis.shape[0] - 16), scale=0.5)
            cv2.imshow(live_win, vis)
            if args.tune_balls:
                cv2.imshow(
                    tuning_win,
                    _render_auto_tuning_panel(tracker, auto_tuner, tuning_status),
                )
            if show_world and tracker.transform is not None:
                cv2.imshow(world_win, render_world_view(state, tracker.transform))
            if show_mask:
                cv2.imshow(mask_win, result.mask)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S")):
                paths = save_debug_snapshot(result.frame, result.mask, state)
                print(f"已保存 debug: {paths['frame'].name}, {paths['json'].name}")
            if args.tune_balls and key in (ord("u"), ord("U")):
                auto_tuner.undo()
                if auto_tuner.points:
                    learned = auto_tuner.apply(tracker.ball_detector)
                    _save_ball_settings(tracker)
                    tuning_status = f"Undo complete; re-saved {learned.marked_balls} marks"
                else:
                    tuning_status = "No marks remain; current saved settings kept"
            if key in (ord("h"), ord("H")):
                show_mask = not show_mask
                if not show_mask:
                    cv2.destroyWindow(mask_win)
            if key in (ord("w"), ord("W")):
                show_world = not show_world
                if show_world:
                    cv2.namedWindow(world_win, cv2.WINDOW_NORMAL)
                else:
                    cv2.destroyWindow(world_win)
            if key in (ord("r"), ord("R")):
                tracker.reload_config(reopen_camera=False)
                auto_tuner.clear()
                tuning_status = "Reloaded saved settings; marks cleared"
                print("已重载 YAML 配置与 Homography。")
    finally:
        tracker.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
