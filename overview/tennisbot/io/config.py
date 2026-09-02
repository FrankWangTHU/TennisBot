"""YAML config load / save helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from tennisbot.io.paths import (
    CAMERA_CONFIG_PATH,
    FIELD_CONFIG_PATH,
    PERCEPTION_CONFIG_PATH,
)


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"配置文件不存在: {path}")
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"配置文件格式错误（需要 mapping）: {path}")
    return data


def save_yaml(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(
            data,
            f,
            allow_unicode=True,
            default_flow_style=False,
            sort_keys=False,
        )


def load_camera_config() -> dict[str, Any]:
    cfg = load_yaml(CAMERA_CONFIG_PATH)
    if "camera" not in cfg:
        raise KeyError(f"{CAMERA_CONFIG_PATH} 缺少 camera 段")
    return cfg["camera"]


def load_field_config() -> dict[str, Any]:
    cfg = load_yaml(FIELD_CONFIG_PATH)
    if "field" not in cfg:
        raise KeyError(f"{FIELD_CONFIG_PATH} 缺少 field 段")
    return cfg["field"]


def load_perception_config() -> dict[str, Any]:
    return load_yaml(PERCEPTION_CONFIG_PATH)


def load_all_config() -> dict[str, Any]:
    return {
        "camera": load_camera_config(),
        "field": load_field_config(),
        "perception": load_perception_config(),
    }
