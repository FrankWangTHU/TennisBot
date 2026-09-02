"""Save timestamped debug frames, masks, and detections."""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from tennisbot.io.paths import DEBUG_DIR
from tennisbot.perception.models import WorldState


def save_debug_snapshot(
    frame_bgr: np.ndarray,
    mask: np.ndarray | None,
    world_state: WorldState,
    debug_dir: Path | None = None,
) -> dict[str, Path]:
    out_dir = debug_dir if debug_dir is not None else DEBUG_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    frame_path = out_dir / f"frame_{stamp}.jpg"
    mask_path = out_dir / f"mask_{stamp}.png"
    json_path = out_dir / f"detections_{stamp}.json"

    cv2.imwrite(str(frame_path), frame_bgr)
    if mask is not None:
        cv2.imwrite(str(mask_path), mask)

    payload = {
        "robot": None
        if world_state.robot is None
        else {
            "x": world_state.robot.x,
            "y": world_state.robot.y,
            "theta": world_state.robot.theta,
        },
        "balls": [
            {
                "id": ball.track_id,
                "x": ball.x,
                "y": ball.y,
                "visible": ball.visible,
            }
            for ball in world_state.balls
        ],
    }
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    return {"frame": frame_path, "mask": mask_path, "json": json_path}
