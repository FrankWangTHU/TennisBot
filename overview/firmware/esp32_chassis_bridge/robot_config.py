"""
麦克纳姆轮小车公共配置。

本工程只保留电机底盘控制。学生通常只需要改这里的运行模式、CAN/PS2
引脚、最大电机转速、麦克纳姆轮车体参数和预留舵机参数。

坐标约定：
- vx > 0：前进，单位 m/s
- vy > 0：向左平移，单位 m/s
- omega > 0：向左原地转向 / 逆时针，单位 rad/s

作者 王笑
日期 20260701
"""


def clamp(value, low, high):
    return max(low, min(high, value))


# =============================================================================
# 硬件端口与运行模式
# =============================================================================

RUN_MODE = "ps2"  # 只有 "ps2" 进入遥控，其它值都按 idle 接口测试运行

# CAN 接线
CAN_BUS_ID = 0
CAN_BAUDRATE = 1000000
CAN_TX = 8
CAN_RX = 18

# 预留舵机 UART 接线
SERVO_UART_ID = 2
SERVO_UART_BAUD = 115200
SERVO_UART_TX = 16
SERVO_UART_RX = 17

#摄像头接线
CAMERA_UART_ID   = 1
CAMERA_UART_BAUD = 115200
CAMERA_UART_TX   = 5
CAMERA_UART_RX   = 6

# PS2 手柄接线
PS2_DI = 9
PS2_DO = 10
PS2_CS = 11
PS2_CLK = 12


# =============================================================================
# 电机与麦克纳姆轮底盘配置
# =============================================================================

MAX_MOTOR_RPM = 200.0       #电机最大转速
DEFAULT_ACC_RAD_S2 = 20.0   #默认加速度

# 麦克纳姆轮运动学参数。lx、ly 用在常见公式项 (lx + ly) * omega 中。
# 实测值是整车长/宽，而不是车体中心到轮子的距离，在这里除以 2。
MECANUM_LX_M = 0.41/2
MECANUM_LY_M = 0.41/2
WHEEL_RADIUS_M = 0.0635


# =============================================================================
# 预留舵机配置
# =============================================================================

# 未安装预留舵机时保持 RESERVE_SERVO_ENABLED = False。
# 预留舵机 ID 支持 0~254，可填多个 ID，中间用逗号分隔，如 (13, 14)。
RESERVE_SERVO_ENABLED = False
RESERVE_SERVO_IDS = (14,)
RESERVE_SERVO_SIGNS = {14: 1}             # 每个 ID 的方向符号，1 或 -1
RESERVE_SERVO_INIT_ANGLE_DEG = {14: 0.0}  # 每个 ID 的初始角
RESERVE_SERVO_MIN_DEG = {14: -90.0}       # 每个 ID 的角度下限
RESERVE_SERVO_MAX_DEG = {14: 90.0}        # 每个 ID 的角度上限
