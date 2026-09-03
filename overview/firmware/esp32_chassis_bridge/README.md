# ESP32-S3 底盘控制烧录包

本目录是完整板的板端程序。`chassis_control.py`、`motor_lib.py`、`robot_config.py`、`ps2_control.py` 和 `ps2_lib.py` 来自原始 `SmartHybridChasisDemo`；原示例没有实现 PC 指令解析，`main.py` 是新增的安全通信入口。

上传前在 `control_config.py` 选择一种模式：

```python
CONTROL_MODE = "serial"  # 首次联调推荐：PC USB 串口
CONTROL_MODE = "udp"     # 后续：PC Wi-Fi UDP
CONTROL_MODE = "ps2"     # 恢复原 PS2 遥控
```

一次性把本目录下这 9 个文件上传至 ESP32 根目录：

```text
boot.py
main.py
control_config.py
wifi_config.py
chassis_control.py
motor_lib.py
robot_config.py
ps2_control.py
ps2_lib.py
```

三种模式互斥，避免两个控制源同时驱动电机。修改模式后只需重新上传 `control_config.py` 并复位，不需要重新刷 `.bin`。

有线模式 300 ms 收不到速度即停车失能；UDP 模式考虑 Windows 调度和无线抖动，使用 600 ms。两种模式失联后都必须重新 ENABLE。
