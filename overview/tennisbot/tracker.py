"""V1 global tracker: camera + AprilTag + balls + homography -> WorldState."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from tennisbot.calibration.homography import FieldTransform
from tennisbot.camera.camo_camera import open_camo_camera
from tennisbot.camera.camera_source import CameraSource
from tennisbot.io.config import load_all_config
from tennisbot.localization.ball_memory import BallMemoryTracker
from tennisbot.localization.robot_pose import pose_from_tag
from tennisbot.perception.apriltag_detector import AprilTagDetector
from tennisbot.perception.ball_detector import BallDetector
from tennisbot.perception.models import AprilTagDetection, BallDetection, BallWorld, WorldState


class FrameResult:
    def __init__(
        self,
        frame: np.ndarray,
        world_state: WorldState,
        tags,
        robot_tag,
        balls,
        mask: np.ndarray,
        visible_world_balls: list[BallWorld] | None = None,
    ) -> None:
        self.frame = frame
        self.world_state = world_state
        self.tags = tags
        self.robot_tag = robot_tag
        self.balls = balls
        self.mask = mask
        self.visible_world_balls = visible_world_balls or []


class GlobalTracker:
    def __init__(self, require_homography: bool = True) -> None:
        self.require_homography = require_homography
        self.camera: CameraSource | None = None
        self.camera_cfg: dict[str, Any] = {}
        self.field_cfg: dict[str, Any] = {}
        self.perception_cfg: dict[str, Any] = {}
        self.tag_detector = AprilTagDetector()
        self.ball_detector = BallDetector()
        self.transform: FieldTransform | None = None
        self.processing_scale = 1.0
        self.apriltag_scale = 1.0
        self.ball_memory = BallMemoryTracker()
        self._last_result: FrameResult | None = None
        self.reload_config(reopen_camera=True)

    def reload_config(self, reopen_camera: bool = False) -> None:
        cfg = load_all_config()
        self.camera_cfg = cfg["camera"]
        self.field_cfg = cfg["field"]
        self.perception_cfg = cfg["perception"]
        self.processing_scale = float(
            self.perception_cfg.get("performance", {}).get("processing_scale", 1.0)
        )
        self.apriltag_scale = float(
            self.perception_cfg.get("performance", {}).get("apriltag_scale", 1.0)
        )
        if not 0.1 <= self.processing_scale <= 1.0:
            raise ValueError("performance.processing_scale 必须在 0.1..1.0 之间。")
        if not 0.1 <= self.apriltag_scale <= 1.0:
            raise ValueError("performance.apriltag_scale 必须在 0.1..1.0 之间。")
        self.tag_detector = AprilTagDetector.from_config(self.perception_cfg)
        self.ball_detector = BallDetector.from_config(self.perception_cfg)
        self.ball_memory = BallMemoryTracker.from_config(self.perception_cfg)
        self.transform = FieldTransform.try_load()
        if self.require_homography and self.transform is None:
            raise FileNotFoundError(
                "未找到场地 Homography。请先运行:\n"
                "  python scripts/calibrate_field.py"
            )
        if reopen_camera or self.camera is None:
            self.close()
            self.camera = open_camo_camera(camera_cfg=self.camera_cfg)

    def close(self) -> None:
        if self.camera is not None:
            self.camera.release()
            self.camera = None

    def read_frame(self) -> np.ndarray:
        if self.camera is None:
            raise RuntimeError("摄像头未打开。")
        return self.camera.read()

    def process_frame(self, frame: np.ndarray) -> FrameResult:
        ball_scale = self.processing_scale
        tag_scale = self.apriltag_scale
        if ball_scale < 0.999:
            ball_frame = np.ascontiguousarray(
                cv2.resize(frame, None, fx=ball_scale, fy=ball_scale, interpolation=cv2.INTER_AREA)
            )
        else:
            ball_frame = frame
        if abs(tag_scale - ball_scale) < 1e-6:
            tag_frame = ball_frame
        elif tag_scale < 0.999:
            tag_frame = np.ascontiguousarray(
                cv2.resize(frame, None, fx=tag_scale, fy=tag_scale, interpolation=cv2.INTER_AREA)
            )
        else:
            tag_frame = frame

        tags = self.tag_detector.detect(tag_frame)
        robot_tag = self.tag_detector.select_robot_tag(tags)
        # Area thresholds in YAML are expressed in full-resolution pixels.
        # Keep their meaning unchanged when detection runs on a smaller frame.
        original_min_area = self.ball_detector.min_area
        original_max_area = self.ball_detector.max_area
        if ball_scale < 0.999:
            self.ball_detector.min_area *= ball_scale * ball_scale
            self.ball_detector.max_area *= ball_scale * ball_scale
        try:
            balls, mask = self.ball_detector.detect(ball_frame)
        finally:
            self.ball_detector.min_area = original_min_area
            self.ball_detector.max_area = original_max_area

        if tag_scale < 0.999:
            tag_inv = 1.0 / tag_scale
            tags = [
                AprilTagDetection(
                    tag_id=tag.tag_id,
                    center_uv=(tag.center_uv[0] * tag_inv, tag.center_uv[1] * tag_inv),
                    corners_uv=tag.corners_uv * tag_inv,
                    heading_image_rad=tag.heading_image_rad,
                )
                for tag in tags
            ]
            robot_tag = self.tag_detector.select_robot_tag(tags)
        if ball_scale < 0.999:
            ball_inv = 1.0 / ball_scale
            balls = [
                BallDetection(
                    center_uv=(ball.center_uv[0] * ball_inv, ball.center_uv[1] * ball_inv),
                    radius_px=ball.radius_px * ball_inv,
                    area_px=ball.area_px * ball_inv * ball_inv,
                )
                for ball in balls
            ]
            mask = cv2.resize(
                mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST
            )

        robot = None
        if robot_tag is not None and self.transform is not None:
            robot = pose_from_tag(
                robot_tag,
                self.transform,
                self.tag_detector.front_edge,
                self.perception_cfg.get("apriltag", {}),
            )

        observations: list[tuple[BallDetection, BallWorld]] = []
        if self.transform is not None:
            ball_cfg = self.perception_cfg.get("ball_detection", {})
            restrict_to_field = bool(ball_cfg.get("restrict_to_field", True))
            margin = float(ball_cfg.get("field_margin_m", 0.02))
            for ball in balls:
                x, y = self.transform.image_to_world(ball.center_uv)
                if restrict_to_field and not (
                    -margin <= x <= self.transform.width_m + margin
                    and -margin <= y <= self.transform.height_m + margin
                ):
                    continue
                observations.append((ball, BallWorld(x=x, y=y)))
            tracking_enabled = bool(
                self.perception_cfg.get("ball_tracking", {}).get("enabled", True)
            )
            if tracking_enabled:
                world_balls, visible_pairs = self.ball_memory.update(
                    observations, frame, self.transform, robot
                )
            else:
                visible_pairs = observations
                world_balls = [world for _det, world in observations]
            balls = [item[0] for item in visible_pairs]
            visible_world_balls = [item[1] for item in visible_pairs]
        else:
            world_balls = []
            visible_world_balls = []

        state = WorldState(robot=robot, balls=world_balls)
        result = FrameResult(
            frame=frame,
            world_state=state,
            tags=tags,
            robot_tag=robot_tag,
            balls=balls,
            mask=mask,
            visible_world_balls=visible_world_balls,
        )
        self._last_result = result
        return result

    def get_world_state(self) -> WorldState:
        """Standard V1 output used by later navigation modules."""
        result = self.process_frame(self.read_frame())
        return result.world_state

    def __enter__(self) -> GlobalTracker:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
