"""
电机 CAN 速度模式库。

推荐学生按顺序显式调用（单电机示例，Kp/Ki 必须在失能状态下写入）：
    motor_bus.disable(motor_id)
    motor_bus.clear_fault(motor_id)
    motor_bus.set_speed_mode(motor_id)
    motor_bus.set_speed_pi(motor_id)
    motor_bus.set_speed_filter_gain(motor_id)
    motor_bus.enable_only(motor_id)
    motor_bus.set_acc(motor_id, DEFAULT_ACC_RAD_S2)
    motor_bus.set_speed(motor_id, speed_rad_s)

批量初始化：init_speed_mode(motor_id) 或 prepare_speed_mode([1, 2, 3, 4])。

作者 王笑
日期 20260701
"""

import time

try:
    import ustruct
except ImportError:
    import struct as ustruct

from robot_config import (
    DEFAULT_ACC_RAD_S2,
    clamp,
)

_DRIVER_SPEED_LIMIT_RAD_S = 44.0
_MOTOR_SPEED_PI_KP = 5.0
_MOTOR_SPEED_PI_KI = 0.02
_MOTOR_SPEED_FILTER_GAIN = 0.1


def sleep_ms(ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


def build_ext_id(comm_type, motor_id, data2=0):
    """
    29 位扩展 CAN ID：
    bit28~24=comm_type, bit23~8=data2, bit7~0=motor_id。
    主机 ID 固定使用 0xFD，即 data2=0x00FD。
    """
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (motor_id & 0xFF)


class MotorBus:
    MODE_SPEED = 2

    def __init__(self, can_bus):
        self.can = can_bus

    def _send(self, motor_id, comm_type, data, data2=0x00FD):
        payload = list(data)[:8]
        while len(payload) < 8:
            payload.append(0)
        self.can.send(payload, build_ext_id(comm_type, motor_id, data2), extframe=True)

    def write_param_uint32(self, motor_id, index, value):
        data = list(ustruct.pack("<H", index))
        data += [0x00, 0x00]
        data += list(ustruct.pack("<I", int(value)))
        self._send(motor_id, 0x12, data)

    def write_param_float(self, motor_id, index, value):
        data = list(ustruct.pack("<H", index))
        data += [0x00, 0x00]
        data += list(ustruct.pack("<f", float(value)))
        self._send(motor_id, 0x12, data)

    def set_speed_mode(self, motor_id):
        self.write_param_uint32(motor_id, 0x7005, self.MODE_SPEED)

    def set_acc(self, motor_id, acc_rad_s2=DEFAULT_ACC_RAD_S2):
        self.write_param_float(motor_id, 0x7022, acc_rad_s2)

    def set_speed_filter_gain(self, motor_id, gain=_MOTOR_SPEED_FILTER_GAIN):
        # spd_filt_gain = 0x7021，须在失能状态下、使能前写入。
        self.write_param_float(motor_id, 0x7021, gain)

    def set_speed_pi(self, motor_id, kp=_MOTOR_SPEED_PI_KP, ki=_MOTOR_SPEED_PI_KI):
        """速度环 Kp/Ki = 0x701F / 0x7020。"""
        self.write_param_float(motor_id, 0x701F, kp)
        sleep_ms(20)
        self.write_param_float(motor_id, 0x7020, ki)

    def enable_only(self, motor_id):
        self._send(motor_id, 0x03, [0] * 8)

    def disable(self, motor_id):
        self._send(motor_id, 0x04, [0] * 8)

    def clear_fault(self, motor_id):
        self._send(motor_id, 0x04, [1] + [0] * 7)

    def set_speed(self, motor_id, speed_rad_s):
        speed_rad_s = clamp(speed_rad_s, -_DRIVER_SPEED_LIMIT_RAD_S, _DRIVER_SPEED_LIMIT_RAD_S)
        self.write_param_float(motor_id, 0x700A, speed_rad_s)

    def stop(self, motor_id):
        self.set_speed(motor_id, 0.0)

    def init_speed_mode(self, motor_id, kp=_MOTOR_SPEED_PI_KP, ki=_MOTOR_SPEED_PI_KI):
        """
        单电机速度模式初始化：失能 -> 清故障 -> 速度模式 -> PI -> 滤波 -> 使能。
        """
        self.disable(motor_id)
        sleep_ms(20)
        self.clear_fault(motor_id)
        sleep_ms(20)
        self.set_speed_mode(motor_id)
        sleep_ms(50)
        self.set_speed_pi(motor_id, kp, ki)
        sleep_ms(20)
        self.set_speed_filter_gain(motor_id)
        sleep_ms(20)
        self.enable_only(motor_id)
        sleep_ms(50)

    def prepare_speed_mode(self, motor_ids):
        """批量初始化。加速度在控制时由底盘层单独下发。"""
        for motor_id in motor_ids:
            self.init_speed_mode(motor_id)

    def stop_all(self, motor_ids):
        for motor_id in motor_ids:
            self.stop(motor_id)
            sleep_ms(10)

    def disable_all(self, motor_ids):
        self.stop_all(motor_ids)
        sleep_ms(50)
        for motor_id in motor_ids:
            self.disable(motor_id)
            sleep_ms(10)
