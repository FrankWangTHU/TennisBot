"""Image overlay: field, AprilTag, balls, HUD text."""

from __future__ import annotations

import cv2
import numpy as np

from tennisbot.calibration.homography import FieldTransform
from tennisbot.localization.robot_pose import front_uv
from tennisbot.perception.models import (
    AprilTagDetection,
    BallDetection,
    BallWorld,
    RobotPose,
    WorldState,
)

FIELD_COLOR = (80, 220, 80)
TAG_COLOR = (255, 180, 0)
ARROW_COLOR = (0, 0, 255)
BALL_COLOR = (0, 220, 255)
MEMORY_BALL_COLOR = (180, 120, 255)
CENTER_COLOR = (0, 255, 0)
TEXT_COLOR = (255, 255, 255)


def format_meters(value: float) -> str:
    """Keep centimeter-level field dimensions visible (1.25 must not become 1.2)."""
    return f"{value:.2f}".rstrip("0").rstrip(".")


def put_text_bg(
    image: np.ndarray,
    text: str,
    org: tuple[int, int],
    scale: float = 0.6,
    color: tuple[int, int, int] = TEXT_COLOR,
    thickness: int = 1,
) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = org
    cv2.rectangle(
        image,
        (x - 4, y - th - 4),
        (x + tw + 4, y + baseline + 4),
        (0, 0, 0),
        thickness=-1,
    )
    cv2.putText(image, text, org, font, scale, color, thickness, cv2.LINE_AA)


def draw_heading_arrow(
    image: np.ndarray,
    center_uv: tuple[float, float],
    front: tuple[float, float],
    length_px: float = 70.0,
    color: tuple[int, int, int] = ARROW_COLOR,
) -> None:
    c = np.array(center_uv, dtype=np.float64)
    vec = np.array(front, dtype=np.float64) - c
    n = float(np.linalg.norm(vec))
    if n < 1e-6:
        return
    end = c + vec / n * length_px
    cv2.arrowedLine(
        image,
        (int(round(c[0])), int(round(c[1]))),
        (int(round(end[0])), int(round(end[1]))),
        color,
        2,
        tipLength=0.25,
    )


def draw_field(
    image: np.ndarray,
    transform: FieldTransform,
) -> None:
    poly = transform.image_field_polygon().reshape(-1, 1, 2)
    cv2.polylines(image, [poly], isClosed=True, color=FIELD_COLOR, thickness=2)
    corners_world = transform.world_corners()
    w_label = format_meters(transform.width_m)
    h_label = format_meters(transform.height_m)
    labels = ["(0,0)", f"({w_label},0)", f"({w_label},{h_label})", f"(0,{h_label})"]
    for xy, label in zip(corners_world, labels):
        u, v = transform.world_to_image(xy)
        cv2.circle(image, (int(u), int(v)), 6, FIELD_COLOR, -1)
        put_text_bg(image, label, (int(u) + 8, int(v) - 8), scale=0.45, color=FIELD_COLOR)
    origin = transform.world_to_image((0.0, 0.0))
    x_axis = transform.world_to_image((min(0.6, transform.width_m * 0.2), 0.0))
    y_axis = transform.world_to_image((0.0, min(0.6, transform.height_m * 0.2)))
    cv2.arrowedLine(
        image,
        (int(origin[0]), int(origin[1])),
        (int(x_axis[0]), int(x_axis[1])),
        (0, 165, 255),
        2,
        tipLength=0.2,
    )
    cv2.arrowedLine(
        image,
        (int(origin[0]), int(origin[1])),
        (int(y_axis[0]), int(y_axis[1])),
        (255, 0, 255),
        2,
        tipLength=0.2,
    )
    put_text_bg(image, "+X", (int(x_axis[0]) + 6, int(x_axis[1])), scale=0.5, color=(0, 165, 255))
    put_text_bg(image, "+Y", (int(y_axis[0]) + 6, int(y_axis[1])), scale=0.5, color=(255, 0, 255))


def draw_apriltag(
    image: np.ndarray,
    detection: AprilTagDetection,
    pose: RobotPose | None = None,
    front_edge: str = "top",
) -> None:
    corners = detection.corners_uv.astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(image, [corners], isClosed=True, color=TAG_COLOR, thickness=2)
    cu, cv_ = detection.center_uv
    cv2.circle(image, (int(cu), int(cv_)), 5, CENTER_COLOR, -1)
    draw_heading_arrow(image, detection.center_uv, front_uv(detection, front_edge))
    label = f"ID {detection.tag_id}"
    if pose is not None:
        label += f"  {pose.x:.2f}m,{pose.y:.2f}m,{pose.theta_deg:.1f}deg"
    put_text_bg(image, label, (int(cu) + 10, int(cv_) - 12), scale=0.5, color=TAG_COLOR)


def draw_ball(
    image: np.ndarray,
    detection: BallDetection,
    index: int,
    world: BallWorld | None = None,
) -> None:
    u, v = detection.center_uv
    cv2.circle(image, (int(u), int(v)), int(max(2, detection.radius_px)), BALL_COLOR, 2)
    cv2.circle(image, (int(u), int(v)), 3, BALL_COLOR, -1)
    ball_id = world.track_id if world is not None and world.track_id >= 0 else index
    if world is not None:
        text = f"Ball {ball_id}: ({world.x:.2f}, {world.y:.2f})"
    else:
        text = f"Ball {index}: ({u:.0f}, {v:.0f})"
    put_text_bg(image, text, (int(u) + 8, int(v) + 16), scale=0.45, color=BALL_COLOR)


def draw_hud(
    image: np.ndarray,
    lines: list[str],
    origin: tuple[int, int] = (12, 28),
) -> None:
    x, y = origin
    for i, line in enumerate(lines):
        put_text_bg(image, line, (x, y + i * 26), scale=0.6)


def draw_overlay(
    image: np.ndarray,
    *,
    transform: FieldTransform | None = None,
    tags: list[AprilTagDetection] | None = None,
    balls: list[BallDetection] | None = None,
    world_state: WorldState | None = None,
    visible_ball_worlds: list[BallWorld] | None = None,
    robot_tag: AprilTagDetection | None = None,
    front_edge: str = "top",
    hud_lines: list[str] | None = None,
) -> np.ndarray:
    vis = image.copy()
    if transform is not None:
        draw_field(vis, transform)
    robot_pose = world_state.robot if world_state is not None else None
    world_balls = (
        visible_ball_worlds
        if visible_ball_worlds is not None
        else (world_state.balls if world_state is not None else None)
    )
    if tags:
        for det in tags:
            pose = None
            if robot_pose is not None:
                if robot_tag is not None and det.tag_id == robot_tag.tag_id:
                    pose = robot_pose
                elif robot_tag is None and len(tags) == 1:
                    pose = robot_pose
            draw_apriltag(vis, det, pose=pose, front_edge=front_edge)
    if balls:
        for i, ball in enumerate(balls):
            world = world_balls[i] if world_balls is not None and i < len(world_balls) else None
            draw_ball(vis, ball, i, world)
    if transform is not None and world_state is not None:
        for ball in world_state.balls:
            if ball.visible:
                continue
            u, v = transform.world_to_image((ball.x, ball.y))
            center = (int(round(u)), int(round(v)))
            cv2.circle(vis, center, 16, MEMORY_BALL_COLOR, 2, cv2.LINE_AA)
            cv2.drawMarker(vis, center, MEMORY_BALL_COLOR, cv2.MARKER_TILTED_CROSS, 18, 2)
            put_text_bg(
                vis,
                f"Ball {ball.track_id} MEMORY ({ball.x:.2f},{ball.y:.2f})",
                (center[0] + 12, center[1] - 12),
                scale=0.43,
                color=MEMORY_BALL_COLOR,
            )
    if hud_lines:
        draw_hud(vis, hud_lines)
    return vis
