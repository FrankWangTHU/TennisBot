"""Short-term world-coordinate memory for stationary tennis balls."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot

import cv2
import numpy as np

from tennisbot.calibration.homography import FieldTransform
from tennisbot.perception.models import BallDetection, BallWorld, RobotPose


@dataclass
class _Track:
    track_id: int
    x: float
    y: float
    radius_px: float
    visible: bool = True
    missed_frames: int = 0
    clear_missing_frames: int = 0


class BallMemoryTracker:
    """Keep occluded balls; delete only after their location is visibly clear."""

    def __init__(
        self,
        match_distance_m: float = 0.12,
        removal_confirm_frames: int = 30,
        robot_occlusion_radius_m: float = 0.35,
        clear_floor_fraction: float = 0.60,
        position_alpha: float = 0.35,
    ) -> None:
        self.match_distance_m = float(match_distance_m)
        self.removal_confirm_frames = int(removal_confirm_frames)
        self.robot_occlusion_radius_m = float(robot_occlusion_radius_m)
        self.clear_floor_fraction = float(clear_floor_fraction)
        self.position_alpha = float(position_alpha)
        self._tracks: list[_Track] = []
        self._next_id = 0

    @classmethod
    def from_config(cls, perception_cfg: dict) -> BallMemoryTracker:
        cfg = perception_cfg.get("ball_tracking", {})
        return cls(
            match_distance_m=cfg.get("match_distance_m", 0.12),
            removal_confirm_frames=cfg.get("removal_confirm_frames", 30),
            robot_occlusion_radius_m=cfg.get("robot_occlusion_radius_m", 0.35),
            clear_floor_fraction=cfg.get("clear_floor_fraction", 0.60),
            position_alpha=cfg.get("position_alpha", 0.35),
        )

    def clear(self) -> None:
        self._tracks.clear()
        self._next_id = 0

    def _location_is_clear_floor(
        self,
        frame_bgr: np.ndarray,
        transform: FieldTransform,
        track: _Track,
    ) -> bool:
        u, v = transform.world_to_image((track.x, track.y))
        radius = max(10, int(round(track.radius_px * 1.5)))
        x, y = int(round(u)), int(round(v))
        h, w = frame_bgr.shape[:2]
        x0, x1 = max(0, x - radius), min(w, x + radius + 1)
        y0, y1 = max(0, y - radius), min(h, y + radius + 1)
        if x0 >= x1 or y0 >= y1:
            return False
        patch = frame_bgr[y0:y1, x0:x1]
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        floor_like = (hsv[..., 1] < 75) & (hsv[..., 2] > 105)
        return float(np.mean(floor_like)) >= self.clear_floor_fraction

    def update(
        self,
        observations: list[tuple[BallDetection, BallWorld]],
        frame_bgr: np.ndarray,
        transform: FieldTransform,
        robot: RobotPose | None,
    ) -> tuple[list[BallWorld], list[tuple[BallDetection, BallWorld]]]:
        """Return (all remembered balls, visible detection/world pairs)."""
        candidates: list[tuple[float, int, int]] = []
        for ti, track in enumerate(self._tracks):
            for oi, (_det, world) in enumerate(observations):
                distance = hypot(track.x - world.x, track.y - world.y)
                if distance <= self.match_distance_m:
                    candidates.append((distance, ti, oi))
        candidates.sort()
        matched_tracks: set[int] = set()
        matched_obs: set[int] = set()
        obs_track: dict[int, _Track] = {}
        for _distance, ti, oi in candidates:
            if ti in matched_tracks or oi in matched_obs:
                continue
            track = self._tracks[ti]
            _det, world = observations[oi]
            a = self.position_alpha
            track.x = (1.0 - a) * track.x + a * world.x
            track.y = (1.0 - a) * track.y + a * world.y
            track.radius_px = (1.0 - a) * track.radius_px + a * observations[oi][0].radius_px
            track.visible = True
            track.missed_frames = 0
            track.clear_missing_frames = 0
            matched_tracks.add(ti)
            matched_obs.add(oi)
            obs_track[oi] = track

        for oi, (det, world) in enumerate(observations):
            if oi in matched_obs:
                continue
            track = _Track(
                track_id=self._next_id,
                x=world.x,
                y=world.y,
                radius_px=det.radius_px,
            )
            self._next_id += 1
            self._tracks.append(track)
            obs_track[oi] = track

        keep: list[_Track] = []
        for ti, track in enumerate(self._tracks):
            if ti in matched_tracks or track in obs_track.values():
                keep.append(track)
                continue
            track.visible = False
            track.missed_frames += 1
            robot_near = robot is not None and hypot(track.x - robot.x, track.y - robot.y) <= self.robot_occlusion_radius_m
            clear = (not robot_near) and self._location_is_clear_floor(frame_bgr, transform, track)
            track.clear_missing_frames = track.clear_missing_frames + 1 if clear else 0
            if track.clear_missing_frames < self.removal_confirm_frames:
                keep.append(track)
        self._tracks = keep

        all_world = [
            BallWorld(t.x, t.y, track_id=t.track_id, visible=t.visible)
            for t in sorted(self._tracks, key=lambda item: item.track_id)
        ]
        visible_pairs: list[tuple[BallDetection, BallWorld]] = []
        for oi, (det, _world) in enumerate(observations):
            track = obs_track[oi]
            visible_pairs.append(
                (det, BallWorld(track.x, track.y, track_id=track.track_id, visible=True))
            )
        visible_pairs.sort(key=lambda pair: pair[1].track_id)
        return all_world, visible_pairs

