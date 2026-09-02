"""
麦克纳姆轮底盘组合控制（非底层协议库）。

本文件只依赖 motor_lib.MotorBus，负责把车体速度 vx/vy/omega 转换为
四个电机的速度命令。常用接口命名尽量贴近月球车底盘库，降低迁移成本。

常用学生接口：
- prepare()
- enable_motors()
- disable()
- stop()
- drive(vx, vy, omega)
- forward(speed_m_s)
- strafe_left(speed_m_s)
- pivot_turn(omega_rad_s)

作者 王笑
日期 20260701
"""

import math

from robot_config import (
    DEFAULT_ACC_RAD_S2,
    MAX_MOTOR_RPM,
    MECANUM_LX_M,
    MECANUM_LY_M,
    WHEEL_RADIUS_M,
    clamp,
)

MAX_CHASSIS_LINEAR_SPEED_M_S = 0.60
MAX_CHASSIS_OMEGA_RAD_S = 0.80

_MAX_MOTOR_RAD_S = MAX_MOTOR_RPM * 2.0 * math.pi / 60.0
_STOP_EPS_RAD_S = 0.01
_MOTOR_COMMAND_EPS_RAD_S = 0.01
_ACC_COMMAND_EPS_RAD_S2 = 0.01

_DRIVE_WHEELS = (
    {"name": "left_front", "motor_id": 1, "direction": -1},
    {"name": "right_front", "motor_id": 2, "direction": 1},
    {"name": "left_rear", "motor_id": 3, "direction": -1},
    {"name": "right_rear", "motor_id": 4, "direction": 1},
)


class MecanumChassis:
    def __init__(self, motor_bus):
        self.motor_bus = motor_bus
        self.motor_ids = tuple(w["motor_id"] for w in _DRIVE_WHEELS)
        self.motors_enabled = False
        self._motors_stopped = True
        self._last_motor_acc = {}
        self._last_motor_speeds = {}
        self.last_wheel_speeds = (0.0, 0.0, 0.0, 0.0)

    def _reset_command_cache(self):
        self._last_motor_acc = {}
        self._last_motor_speeds = {}
        self.last_wheel_speeds = (0.0, 0.0, 0.0, 0.0)

    @staticmethod
    def _same_float(a, b, eps):
        return b is not None and abs(float(a) - float(b)) <= eps

    def _set_motor_acc_cached(self, motor_id, acc_rad_s2):
        old_acc = self._last_motor_acc.get(motor_id)
        if self._same_float(acc_rad_s2, old_acc, _ACC_COMMAND_EPS_RAD_S2):
            return False
        self.motor_bus.set_acc(motor_id, acc_rad_s2)
        self._last_motor_acc[motor_id] = float(acc_rad_s2)
        return True

    def _set_motor_speed_cached(self, motor_id, speed_rad_s):
        old_speed = self._last_motor_speeds.get(motor_id)
        if self._same_float(speed_rad_s, old_speed, _MOTOR_COMMAND_EPS_RAD_S):
            return False
        self.motor_bus.set_speed(motor_id, speed_rad_s)
        self._last_motor_speeds[motor_id] = float(speed_rad_s)
        return True

    def prepare(self):
        """初始化并使能四个驱动电机。"""
        self.enable_motors()

    def enable_motors(self):
        """使能四轮：init_speed_mode 内先失能再写速度模式与 PI，最后使能。"""
        self.motor_bus.prepare_speed_mode(self.motor_ids)
        self.motors_enabled = True
        self._motors_stopped = True
        self._reset_command_cache()

    def stop(self):
        if self._motors_stopped:
            return
        self.motor_bus.stop_all(self.motor_ids)
        for motor_id in self.motor_ids:
            self._last_motor_speeds[motor_id] = 0.0
        self.last_wheel_speeds = (0.0, 0.0, 0.0, 0.0)
        self._motors_stopped = True

    def disable(self):
        self.motor_bus.disable_all(self.motor_ids)
        self.motors_enabled = False
        self._motors_stopped = True
        self._reset_command_cache()

    def set_acceleration(self, acc_rad_s2):
        """给四个电机设置相同加速度。"""
        for motor_id in self.motor_ids:
            self._set_motor_acc_cached(motor_id, acc_rad_s2)

    def wheel_speeds(self, vx, vy, omega):
        """
        将车体速度转换为轮子角速度（还未乘电机安装方向）。

        返回顺序：(left_front, right_front, left_rear, right_rear)，单位 rad/s。
        """
        vx = float(vx)
        vy = float(vy)
        omega = float(omega)
        k = MECANUM_LX_M + MECANUM_LY_M
        r = WHEEL_RADIUS_M
        return (
            (vx - vy - k * omega) / r,
            (vx + vy + k * omega) / r,
            (vx + vy - k * omega) / r,
            (vx - vy + k * omega) / r,
        )

    @staticmethod
    def _normalize_wheel_speeds(wheel_speeds):
        max_abs = max(abs(speed) for speed in wheel_speeds)
        if max_abs <= _MAX_MOTOR_RAD_S or max_abs <= 0.0:
            return tuple(wheel_speeds)
        scale = _MAX_MOTOR_RAD_S / max_abs
        return tuple(speed * scale for speed in wheel_speeds)

    @staticmethod
    def _limit_linear_velocity(vx, vy):
        linear_speed = math.sqrt(vx * vx + vy * vy)
        if linear_speed <= MAX_CHASSIS_LINEAR_SPEED_M_S or linear_speed <= 0.0:
            return vx, vy
        scale = MAX_CHASSIS_LINEAR_SPEED_M_S / linear_speed
        return vx * scale, vy * scale

    def drive_wheel_speeds(self, left_front, right_front, left_rear, right_rear,
                           normalize=True, acc_rad_s2=DEFAULT_ACC_RAD_S2):
        """
        直接控制四个轮子的角速度。

        输入单位 rad/s，正数表示该轮向前驱动车体。电机安装方向在内部处理。
        """
        wheel_speeds = (
            float(left_front),
            float(right_front),
            float(left_rear),
            float(right_rear),
        )
        if normalize:
            wheel_speeds = self._normalize_wheel_speeds(wheel_speeds)

        for wheel, wheel_speed in zip(_DRIVE_WHEELS, wheel_speeds):
            motor_id = wheel["motor_id"]
            motor_speed = wheel_speed * wheel["direction"]
            self._set_motor_acc_cached(motor_id, acc_rad_s2)
            self._set_motor_speed_cached(motor_id, motor_speed)

        self.last_wheel_speeds = wheel_speeds
        self._motors_stopped = max(abs(speed) for speed in wheel_speeds) < _STOP_EPS_RAD_S
        return wheel_speeds

    def drive(self, vx, vy, omega, acc_rad_s2=DEFAULT_ACC_RAD_S2):
        """
        麦克纳姆轮底盘速度控制。

        vx：前进速度，单位 m/s，正数前进。
        vy：横移速度，单位 m/s，正数向左。
        omega：转向角速度，单位 rad/s，正数向左原地转向 / 逆时针。
        """
        vx, vy = self._limit_linear_velocity(float(vx), float(vy))
        omega = clamp(float(omega), -MAX_CHASSIS_OMEGA_RAD_S, MAX_CHASSIS_OMEGA_RAD_S)
        wheel_speeds = self.wheel_speeds(vx, vy, omega)
        return self.drive_wheel_speeds(*wheel_speeds, normalize=True, acc_rad_s2=acc_rad_s2)

    def forward(self, speed_m_s):
        return self.drive(speed_m_s, 0.0, 0.0)

    def backward(self, speed_m_s):
        return self.drive(-abs(speed_m_s), 0.0, 0.0)

    def strafe_left(self, speed_m_s):
        return self.drive(0.0, abs(speed_m_s), 0.0)

    def strafe_right(self, speed_m_s):
        return self.drive(0.0, -abs(speed_m_s), 0.0)

    def pivot_turn(self, omega_rad_s):
        """原地转向。正数 omega 向左原地转向 / 逆时针。"""
        return self.drive(0.0, 0.0, omega_rad_s)
