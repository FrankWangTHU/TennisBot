"""Top-down world view drawn from world coordinates (not a warped camera image)."""

from __future__ import annotations

import cv2
import numpy as np

from tennisbot.calibration.homography import FieldTransform
from tennisbot.perception.models import WorldState
from tennisbot.visualization.overlay import format_meters, put_text_bg

BG_COLOR = (32, 32, 32)
FIELD_FILL = (40, 70, 40)
FIELD_LINE = (90, 220, 90)
ROBOT_COLOR = (0, 0, 255)
BALL_COLOR = (0, 220, 255)
MEMORY_BALL_COLOR = (180, 120, 255)
GRID_COLOR = (55, 55, 55)


def world_to_canvas(
    x: float,
    y: float,
    width_m: float,
    height_m: float,
    canvas_w: int,
    canvas_h: int,
    margin: int = 48,
) -> tuple[int, int]:
    usable_w = max(1, canvas_w - 2 * margin)
    usable_h = max(1, canvas_h - 2 * margin)
    u = margin + (x / width_m) * usable_w
    v = canvas_h - margin - (y / height_m) * usable_h
    return int(round(u)), int(round(v))


def render_world_view(
    state: WorldState | None,
    transform: FieldTransform,
    canvas_size: tuple[int, int] = (900, 620),
) -> np.ndarray:
    canvas_w, canvas_h = canvas_size
    canvas = np.full((canvas_h, canvas_w, 3), BG_COLOR, dtype=np.uint8)
    width_m = transform.width_m
    height_m = transform.height_m
    margin = 48

    def pt(x: float, y: float) -> tuple[int, int]:
        return world_to_canvas(x, y, width_m, height_m, canvas_w, canvas_h, margin)

    corners = np.array(
        [pt(0.0, 0.0), pt(width_m, 0.0), pt(width_m, height_m), pt(0.0, height_m)],
        dtype=np.int32,
    )
    cv2.fillConvexPoly(canvas, corners, FIELD_FILL)

    for x in np.arange(0.0, width_m + 1e-6, 1.0):
        cv2.line(canvas, pt(float(x), 0.0), pt(float(x), height_m), GRID_COLOR, 1)
    for y in np.arange(0.0, height_m + 1e-6, 1.0):
        cv2.line(canvas, pt(0.0, float(y)), pt(width_m, float(y)), GRID_COLOR, 1)

    cv2.polylines(canvas, [corners], isClosed=True, color=FIELD_LINE, thickness=2)
    origin = pt(0.0, 0.0)
    cv2.arrowedLine(canvas, origin, pt(min(0.8, width_m * 0.2), 0.0), (0, 165, 255), 2, tipLength=0.2)
    cv2.arrowedLine(canvas, origin, pt(0.0, min(0.8, height_m * 0.2)), (255, 0, 255), 2, tipLength=0.2)
    put_text_bg(canvas, "+X", pt(min(0.85, width_m * 0.22), 0.05), scale=0.5, color=(0, 165, 255))
    put_text_bg(canvas, "+Y", pt(0.05, min(0.85, height_m * 0.22)), scale=0.5, color=(255, 0, 255))
    put_text_bg(canvas, "(0,0)", (origin[0] + 8, origin[1] + 18), scale=0.45, color=FIELD_LINE)

    if state is not None:
        for i, ball in enumerate(state.balls):
            bu, bv = pt(ball.x, ball.y)
            color = BALL_COLOR if ball.visible else MEMORY_BALL_COLOR
            if ball.visible:
                cv2.circle(canvas, (bu, bv), 10, color, -1)
            else:
                cv2.circle(canvas, (bu, bv), 12, color, 2)
                cv2.drawMarker(canvas, (bu, bv), color, cv2.MARKER_TILTED_CROSS, 14, 2)
            ball_id = ball.track_id if ball.track_id >= 0 else i
            suffix = "" if ball.visible else " memory"
            put_text_bg(
                canvas,
                f"Ball {ball_id}{suffix} ({ball.x:.2f},{ball.y:.2f})",
                (bu + 12, bv - 8),
                scale=0.45,
                color=color,
            )
        if state.robot is not None:
            robot = state.robot
            ru, rv = pt(robot.x, robot.y)
            arrow_len = 0.35
            fu, fv = pt(
                robot.x + arrow_len * float(np.cos(robot.theta)),
                robot.y + arrow_len * float(np.sin(robot.theta)),
            )
            cv2.circle(canvas, (ru, rv), 12, ROBOT_COLOR, -1)
            cv2.arrowedLine(canvas, (ru, rv), (fu, fv), ROBOT_COLOR, 2, tipLength=0.25)
            put_text_bg(
                canvas,
                f"Robot ({robot.x:.2f},{robot.y:.2f}) {robot.theta_deg:.1f}deg",
                (ru + 14, rv - 14),
                scale=0.5,
                color=ROBOT_COLOR,
            )

    put_text_bg(
        canvas,
        f"World view  {format_meters(width_m)}m x {format_meters(height_m)}m",
        (12, 24),
        scale=0.55,
    )
    return canvas
