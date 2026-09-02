"""AprilTag detection via OpenCV ArUco.

Heading convention (image frame, debug only):
    Tag top edge (corners 0-1) midpoint minus center = robot forward.

World heading is NOT this image angle. Convert center and a front pixel
through FieldTransform, then atan2 in the world frame.
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from tennisbot.perception.models import AprilTagDetection

APRILTAG_FAMILIES = {
    "DICT_APRILTAG_16h5": "DICT_APRILTAG_16h5",
    "DICT_APRILTAG_25h9": "DICT_APRILTAG_25h9",
    "DICT_APRILTAG_36h10": "DICT_APRILTAG_36h10",
    "DICT_APRILTAG_36h11": "DICT_APRILTAG_36h11",
    "DICT_ARUCO_ORIGINAL": "DICT_ARUCO_ORIGINAL",
    "DICT_4X4_50": "DICT_4X4_50",
    "DICT_5X5_50": "DICT_5X5_50",
    "DICT_6X6_50": "DICT_6X6_50",
}

FRONT_EDGE_INDICES = {
    "top": (0, 1),
    "right": (1, 2),
    "bottom": (2, 3),
    "left": (3, 0),
}


def _get_dictionary(family_name: str):
    name = family_name.strip()
    if not hasattr(cv2.aruco, name):
        known = ", ".join(sorted(APRILTAG_FAMILIES))
        raise ValueError(
            f"未知 AprilTag / ArUco family: {family_name!r}。\n"
            f"请改 config/perception.yaml 的 apriltag.family。可用值例如: {known}"
        )
    dict_id = getattr(cv2.aruco, name)
    return cv2.aruco.getPredefinedDictionary(dict_id)


def _make_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        try:
            return cv2.aruco.DetectorParameters()
        except TypeError:
            pass
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    raise RuntimeError("当前 OpenCV 没有 aruco.DetectorParameters，请安装 opencv-contrib-python。")


def heading_from_corners(
    corners_uv: np.ndarray,
    front_edge: str = "top",
) -> tuple[float, tuple[float, float], tuple[float, float]]:
    """Return (heading_image_rad, center_uv, front_midpoint_uv)."""
    corners = np.asarray(corners_uv, dtype=np.float64).reshape(4, 2)
    center = corners.mean(axis=0)
    if front_edge not in FRONT_EDGE_INDICES:
        raise ValueError(
            f"未知 front_edge={front_edge!r}，应为 top/right/bottom/left。"
        )
    i0, i1 = FRONT_EDGE_INDICES[front_edge]
    front_mid = 0.5 * (corners[i0] + corners[i1])
    vec = front_mid - center
    heading = float(np.arctan2(vec[1], vec[0]))
    return heading, (float(center[0]), float(center[1])), (float(front_mid[0]), float(front_mid[1]))


class AprilTagDetector:
    def __init__(
        self,
        family: str = "DICT_APRILTAG_36h11",
        robot_tag_id: int | None = None,
        front_edge: str = "top",
        corner_refinement: str = "APRILTAG",
        error_correction_rate: float = 0.8,
    ) -> None:
        self.family = family
        self.robot_tag_id = robot_tag_id
        self.front_edge = front_edge
        self.dictionary = _get_dictionary(family)
        self.parameters = _make_detector_parameters()
        refinement_methods = {
            "NONE": getattr(cv2.aruco, "CORNER_REFINE_NONE", 0),
            "SUBPIX": getattr(cv2.aruco, "CORNER_REFINE_SUBPIX", 1),
            "CONTOUR": getattr(cv2.aruco, "CORNER_REFINE_CONTOUR", 2),
            "APRILTAG": getattr(cv2.aruco, "CORNER_REFINE_APRILTAG", 3),
        }
        refine_name = corner_refinement.strip().upper()
        if refine_name not in refinement_methods:
            raise ValueError(f"未知 corner_refinement={corner_refinement!r}")
        self.parameters.cornerRefinementMethod = refinement_methods[refine_name]
        self.parameters.errorCorrectionRate = float(error_correction_rate)
        if hasattr(self.parameters, "aprilTagQuadDecimate"):
            self.parameters.aprilTagQuadDecimate = 1.0
            self.parameters.aprilTagMinClusterPixels = 5
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        else:
            self._detector = None

    @classmethod
    def from_config(cls, perception_cfg: dict[str, Any]) -> AprilTagDetector:
        tag_cfg = perception_cfg.get("apriltag", {})
        robot_id = tag_cfg.get("robot_tag_id", None)
        if robot_id is not None:
            robot_id = int(robot_id)
        return cls(
            family=str(tag_cfg.get("family", "DICT_APRILTAG_36h11")),
            robot_tag_id=robot_id,
            front_edge=str(tag_cfg.get("front_edge", "top")),
            corner_refinement=str(tag_cfg.get("corner_refinement", "APRILTAG")),
            error_correction_rate=float(tag_cfg.get("error_correction_rate", 0.8)),
        )

    def detect(self, frame: np.ndarray) -> list[AprilTagDetection]:
        if frame.ndim == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        if self._detector is not None:
            corners_list, ids, _rejected = self._detector.detectMarkers(gray)
        else:
            corners_list, ids, _rejected = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.parameters
            )
        detections: list[AprilTagDetection] = []
        if ids is None:
            return detections
        for corners, tag_id in zip(corners_list, ids.flatten()):
            corners_uv = np.asarray(corners, dtype=np.float64).reshape(4, 2)
            heading, center, _front = heading_from_corners(corners_uv, self.front_edge)
            detections.append(
                AprilTagDetection(
                    tag_id=int(tag_id),
                    center_uv=center,
                    corners_uv=corners_uv,
                    heading_image_rad=heading,
                )
            )
        return detections

    def select_robot_tag(
        self, detections: list[AprilTagDetection]
    ) -> AprilTagDetection | None:
        if not detections:
            return None
        if self.robot_tag_id is not None:
            for det in detections:
                if det.tag_id == self.robot_tag_id:
                    return det
            return None
        return detections[0]
