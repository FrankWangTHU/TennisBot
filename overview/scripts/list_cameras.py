"""Scan OpenCV camera indices (Windows uses DirectShow)."""

from __future__ import annotations

import _bootstrap  # noqa: F401

from tennisbot.camera.camera_source import camera_backend_candidates, list_camera_indices


def main() -> None:
    print("正在扫描摄像头 index 0-10 ...")
    print(f"按顺序尝试 OpenCV backends: {camera_backend_candidates()}")
    print("请先打开 Camo Camera，并让 iPhone 保持预览。\n")
    results = list_camera_indices(max_index=10)
    available = []
    for index, ok in results:
        status = "available" if ok else "unavailable"
        print(f"Camera {index}: {status}")
        if ok:
            available.append(index)
    print()
    if not available:
        print("没有可用摄像头。请检查 Camo 是否已启动、iPhone 是否已连接。")
        return
    print(f"可用 index: {available}")
    print("把 config/camera.yaml 里的 camera.index 改成 Camo 对应的编号，然后运行:")
    print("  python scripts/preview_camera.py")


if __name__ == "__main__":
    main()
