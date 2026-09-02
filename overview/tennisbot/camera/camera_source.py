"""OpenCV camera capture for Windows Camo / generic webcams."""

from __future__ import annotations

import sys
from typing import Any

import cv2
import numpy as np

from tennisbot.io.config import load_camera_config


def camera_backend() -> int:
    """Prefer DirectShow on Windows so Camo virtual cameras open reliably."""
    if sys.platform.startswith("win"):
        return cv2.CAP_DSHOW
    return cv2.CAP_ANY


def camera_backend_candidates() -> list[int]:
    """Backends in preference order; Windows camera drivers vary by machine."""
    if sys.platform.startswith("win"):
        candidates = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
        return list(dict.fromkeys(candidates))
    return [cv2.CAP_ANY]


class CameraSource:
    def __init__(
        self,
        index: int,
        width: int = 1920,
        height: int = 1080,
        fps: int = 30,
        fourcc: str = "MJPG",
    ) -> None:
        self.index = int(index)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.fourcc = str(fourcc).upper()
        self.cap: cv2.VideoCapture | None = None

    @classmethod
    def from_config(cls, camera_cfg: dict[str, Any] | None = None) -> CameraSource:
        cfg = camera_cfg if camera_cfg is not None else load_camera_config()
        return cls(
            index=int(cfg.get("index", 0)),
            width=int(cfg.get("width", 1920)),
            height=int(cfg.get("height", 1080)),
            fps=int(cfg.get("fps", 30)),
            fourcc=str(cfg.get("fourcc", "MJPG")),
        )

    def open(self) -> None:
        if self.cap is not None and self.cap.isOpened():
            return
        cap = None
        tried: list[int] = []
        for backend in camera_backend_candidates():
            tried.append(backend)
            candidate = cv2.VideoCapture(self.index, backend)
            if candidate.isOpened():
                cap = candidate
                break
            candidate.release()
        if cap is None:
            raise RuntimeError(
                f"无法打开摄像头 index={self.index}。\n"
                f"已尝试 OpenCV backends={tried}。\n"
                "请先运行: python scripts/list_cameras.py\n"
                "确认 Camo Camera 已启动，并把 config/camera.yaml 里的 camera.index 改成可用编号。"
            )
        if len(self.fourcc) == 4:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self.cap = cap

    def read(self) -> np.ndarray:
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError("摄像头未打开。请先调用 CameraSource.open()。")
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError(
                f"摄像头 index={self.index} 读帧失败。\n"
                "请确认 Camo 正在预览、手机未锁屏，且没有其他程序占用该摄像头。"
            )
        return frame

    def actual_size(self) -> tuple[int, int]:
        if self.cap is None:
            return self.width, self.height
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h

    def actual_fps(self) -> float:
        if self.cap is None:
            return float(self.fps)
        return float(self.cap.get(cv2.CAP_PROP_FPS))

    def release(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self) -> CameraSource:
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.release()


def list_camera_indices(max_index: int = 10) -> list[tuple[int, bool]]:
    """Probe indices 0..max_index. Returns (index, available)."""
    results: list[tuple[int, bool]] = []
    for i in range(max_index + 1):
        available = False
        for backend in camera_backend_candidates():
            cap = cv2.VideoCapture(i, backend)
            if cap.isOpened():
                ok, frame = cap.read()
                available = bool(ok and frame is not None)
            cap.release()
            if available:
                break
        results.append((i, available))
    return results
