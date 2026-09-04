"""Top-down world view drawn from world coordinates (not a warped camera image)."""

from __future__ import annotations

import cv2
import numpy as np

from tennisbot.calibration.homography import FieldTransform
from tennisbot.perception.models import WorldState
from tennisbot.visualization.overlay import format_meters, put_text_bg

BG_COLOR = (24, 27, 31)
FIELD_FILL = (224, 226, 220)
FIELD_LINE = (60, 210, 90)
ROBOT_COLOR = (230, 95, 35)
ROBOT_FRONT_COLOR = (40, 40, 230)
BALL_COLOR = (20, 235, 245)
MEMORY_BALL_COLOR = (190, 120, 255)
MINOR_GRID_COLOR = (190, 192, 188)
MAJOR_GRID_COLOR = (125, 128, 124)


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
    # One metre must occupy the same number of pixels on both axes.
    scale = min(usable_w / width_m, usable_h / height_m)
    field_w = width_m * scale
    field_h = height_m * scale
    left = (canvas_w - field_w) * 0.5
    bottom = (canvas_h + field_h) * 0.5
    u = left + x * scale
    v = bottom - y * scale
    return int(round(u)), int(round(v))


def render_world_view(
    state: WorldState | None,
    transform: FieldTransform,
    canvas_size: tuple[int, int] = (1100, 650),
) -> np.ndarray:
    canvas_w, canvas_h = canvas_size
    canvas = np.full((canvas_h, canvas_w, 3), BG_COLOR, dtype=np.uint8)
    width_m = transform.width_m
    height_m = transform.height_m
    margin = 64

    def pt(x: float, y: float) -> tuple[int, int]:
        return world_to_canvas(x, y, width_m, height_m, canvas_w, canvas_h, margin)

    corners = np.array(
        [pt(0.0, 0.0), pt(width_m, 0.0), pt(width_m, height_m), pt(0.0, height_m)],
        dtype=np.int32,
    )
    cv2.fillConvexPoly(canvas, corners, FIELD_FILL)

    grid_step = 0.25
    for x in np.arange(0.0, width_m + 1e-6, grid_step):
        major = abs((x / 0.5) - round(x / 0.5)) < 1e-6
        cv2.line(
            canvas,
            pt(float(x), 0.0),
            pt(float(x), height_m),
            MAJOR_GRID_COLOR if major else MINOR_GRID_COLOR,
            2 if major else 1,
        )
        if major:
            tick = pt(float(x), 0.0)
            cv2.putText(canvas, f"{x:.1f}", (tick[0] - 12, tick[1] + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (210, 210, 210), 1, cv2.LINE_AA)
    for y in np.arange(0.0, height_m + 1e-6, grid_step):
        major = abs((y / 0.5) - round(y / 0.5)) < 1e-6
        cv2.line(
            canvas,
            pt(0.0, float(y)),
            pt(width_m, float(y)),
            MAJOR_GRID_COLOR if major else MINOR_GRID_COLOR,
            2 if major else 1,
        )
        if major:
            tick = pt(0.0, float(y))
            cv2.putText(canvas, f"{y:.1f}", (tick[0] - 42, tick[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (210, 210, 210), 1, cv2.LINE_AA)

    cv2.polylines(canvas, [corners], isClosed=True, color=FIELD_LINE, thickness=2)
    origin = pt(0.0, 0.0)
    axis_len = min(0.30, width_m * 0.2, height_m * 0.35)
    cv2.arrowedLine(canvas, origin, pt(axis_len, 0.0), (0, 165, 255), 3, tipLength=0.2)
    cv2.arrowedLine(canvas, origin, pt(0.0, axis_len), (255, 0, 255), 3, tipLength=0.2)
    put_text_bg(canvas, "+X", pt(axis_len, 0.03), scale=0.48, color=(0, 165, 255))
    put_text_bg(canvas, "+Y", pt(0.03, axis_len), scale=0.48, color=(255, 0, 255))

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
            # Draw an approximately 0.40 m x 0.30 m chassis at metric scale.
            half_length = 0.20
            half_width = 0.15
            c = float(np.cos(robot.theta))
            s = float(np.sin(robot.theta))
            chassis_world = []
            for forward, left in [
                (half_length, half_width),
                (half_length, -half_width),
                (-half_length, -half_width),
                (-half_length, half_width),
            ]:
                chassis_world.append(
                    pt(
                        robot.x + c * forward - s * left,
                        robot.y + s * forward + c * left,
                    )
                )
            chassis = np.asarray(chassis_world, dtype=np.int32)
            cv2.fillConvexPoly(canvas, chassis, ROBOT_COLOR)
            cv2.polylines(canvas, [chassis], True, (255, 255, 255), 2, cv2.LINE_AA)
            arrow_len = 0.30
            fu, fv = pt(
                robot.x + arrow_len * float(np.cos(robot.theta)),
                robot.y + arrow_len * float(np.sin(robot.theta)),
            )
            cv2.circle(canvas, (ru, rv), 5, (255, 255, 255), -1)
            cv2.arrowedLine(canvas, (ru, rv), (fu, fv), ROBOT_FRONT_COLOR, 4, tipLength=0.25)
            put_text_bg(
                canvas,
                f"Robot ({robot.x:.2f},{robot.y:.2f}) {robot.theta_deg:.1f}deg",
                (ru + 14, rv - 14),
                scale=0.5,
                color=(255, 255, 255),
            )

    put_text_bg(
        canvas,
        f"Metric world view  {format_meters(width_m)}m x {format_meters(height_m)}m  grid=0.25m",
        (12, 28),
        scale=0.60,
    )
    visible = 0 if state is None else sum(ball.visible for ball in state.balls)
    remembered = 0 if state is None else len(state.balls) - visible
    robot_text = "robot=lost"
    if state is not None and state.robot is not None:
        robot_text = (
            f"robot=({state.robot.x:.3f},{state.robot.y:.3f}) "
            f"theta={state.robot.theta_deg:.1f}deg"
        )
    put_text_bg(
        canvas,
        f"{robot_text}  balls visible={visible} memory={remembered}",
        (12, canvas_h - 16),
        scale=0.50,
    )
    return canvas
