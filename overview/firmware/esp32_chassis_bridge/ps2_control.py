"""
PS2 遥控业务控制逻辑。

本文件负责把 PS2 按键和摇杆映射到麦克纳姆轮底盘动作。

映射：
- 左摇杆 X：原地转向 / omega
- 右摇杆 Y：vx 前进后退
- 右摇杆 X：vy 左右平移
- R1：停车
- X：电机停车并失能
- Triangle：重新初始化并使能电机
- SELECT：退出 PS2 控制循环

作者 王笑
日期 20260701
"""

import time

from chassis_control import (
    MAX_CHASSIS_LINEAR_SPEED_M_S,
    MAX_CHASSIS_OMEGA_RAD_S,
)
from robot_config import (
    clamp,
    RESERVE_SERVO_ENABLED,
    RESERVE_SERVO_IDS,  
)

_PS2_DEADZONE = 12
_PS2_CONTROL_INTERVAL_MS = 50


def sleep_ms(ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


def map_joystick(raw_val, center=128, deadzone=_PS2_DEADZONE):
    offset = int(raw_val) - center
    if abs(offset) <= deadzone:
        return 0
    sign = 1 if offset > 0 else -1
    active_range = 127.0 - deadzone
    mapped = int(((abs(offset) - deadzone) / active_range) * 100.0) * sign
    return clamp(mapped, -100, 100)


def button_pressed(data, btn):
    return (data & btn) == btn

# ==========================
# 主循环控制：演示如何从底层获取 PS2 控制数据，并映射到底盘动作。
# ==========================
def ps2_loop(chassis, servo_control, ps2, data, serial):
    print("PS2 控制：左摇杆左右原地转向，右摇杆前后控制 vx，右摇杆左右控制 vy，R1停车，X失能，三角使能，SELECT退出。")
    while True:
        ps2.update()
        serial_data = data["value"]
        if serial_data is not None:
            serial.write("ok")
            if len(serial_data) != 0:
                print(f"接受信息 {serial_data}")
            else:
                print(f"警告：数据长度异常，期望6位，实际{len(serial_data)}位。原始数据: {serial_data}")
            data["value"] = None
        fresh, buttons, lx, _ly, rx, ry, _age_ms = ps2.snapshot()
        if not fresh:
            chassis.stop()
            sleep_ms(_PS2_CONTROL_INTERVAL_MS)
            continue

        if button_pressed(buttons, ps2.PS2_BTN_SELECT):
            chassis.stop()
            print("SELECT：退出 PS2 控制。")
            break

        if button_pressed(buttons, ps2.PS2_BTN_R1):
            chassis.stop()
            sleep_ms(100)
            continue

        if button_pressed(buttons, ps2.PS2_BTN_CROSS):
            chassis.disable()
            sleep_ms(200)
            continue

        if button_pressed(buttons, ps2.PS2_BTN_TRIANGLE):
            chassis.enable_motors()
            sleep_ms(200)
            continue
        # 练习1 按下遥控器 up 键 启动指定功能
        if button_pressed(buttons, ps2.PS2_BTN_UP):
            # 在此添加你的程序


            # end 
            continue

        turn = map_joystick(lx)
        forward = -map_joystick(ry)
        strafe_left = -map_joystick(rx)

        vx = forward / 100.0 * MAX_CHASSIS_LINEAR_SPEED_M_S
        vy = strafe_left / 100.0 * MAX_CHASSIS_LINEAR_SPEED_M_S
        omega = -turn / 100.0 * MAX_CHASSIS_OMEGA_RAD_S

        chassis.drive(vx, vy, omega)
        sleep_ms(_PS2_CONTROL_INTERVAL_MS)
