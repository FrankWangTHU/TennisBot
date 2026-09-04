"""Repository paths. Never hard-code Windows drive letters."""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO_ROOT / "config"
DATA_DIR = REPO_ROOT / "data"
CALIBRATION_DIR = DATA_DIR / "calibration"
DEBUG_DIR = DATA_DIR / "debug"
NAVIGATION_LOG_DIR = DATA_DIR / "navigation"
HOMOGRAPHY_PATH = CALIBRATION_DIR / "homography.yaml"

CAMERA_CONFIG_PATH = CONFIG_DIR / "camera.yaml"
FIELD_CONFIG_PATH = CONFIG_DIR / "field.yaml"
PERCEPTION_CONFIG_PATH = CONFIG_DIR / "perception.yaml"
