from __future__ import annotations

import _bootstrap  # noqa: F401

from serial.tools import list_ports


ports = list(list_ports.comports())
if not ports:
    print("没有发现串口。检查 USB 数据线和 CH340/CH341 驱动。")
for port in ports:
    print(f"{port.device:8}  {port.description}  [{port.hwid}]")
