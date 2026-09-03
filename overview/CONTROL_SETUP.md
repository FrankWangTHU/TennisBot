# 从原始示例到 PC 有线控制

## 已核对的原始代码

核对来源：`C:/Users/Frank/Downloads/车体控制部分/车体控制部分/CarControlCode/CarControlCode/CarControlCode/SmartHybridChasisDemo`。

- `main.py` 的 UART1 线程只读取、打印相机数据。
- `ps2_control.py` 对串口数据只回复 `ok`，没有解析速度。
- 原示例真正可复用的是 `MecanumChassis.drive(vx, vy, omega)`、CAN 电机库和 PS2 控制。
- 本项目烧录包中的底盘、CAN 和 PS2 文件与该目录原件一致；新增部分是 PC 通信、安全看门狗和控制源切换。

## 推荐执行顺序

1. 使用项目附带的厂家 `MicroPython1.27.bin`，确认 `from esp32 import CAN` 可导入。
2. 将 `firmware/esp32_chassis_bridge/control_config.py` 设为 `CONTROL_MODE = "serial"`。
3. 用 Thonny 上传烧录包 README 中列出的 9 个文件。
4. 关闭 Thonny，运行只通信测试。
5. 架空轮子测试三轴。
6. 用键盘低速控制。
7. 再运行视觉闭环。
8. 有线稳定后把模式和 PC driver 一起切换到 `udp`。

## 有线通信测试

```powershell
cd C:\Users\Frank\Desktop\大三小学期\TennisBot\overview
.\.venv\Scripts\Activate.ps1
python scripts/list_serial_ports.py
python scripts/test_chassis_serial.py --port COM12
```

把 COM12 换成实际端口。看到 `PING 成功` 表示新增通信桥正常；原始示例本身不会回应这个 PING。

## 架空轮子测试

```powershell
python scripts/test_chassis_serial.py --port COM12 --axis vx --value 0.05 --duration 0.5 --enable-motion
python scripts/test_chassis_serial.py --port COM12 --axis vy --value 0.05 --duration 0.5 --enable-motion
python scripts/test_chassis_serial.py --port COM12 --axis omega --value 0.15 --duration 0.5 --enable-motion
```

预期依次是前进、左移、逆时针。如果不一致，检查电机 1-4 安装位置和接线。

## 电脑键盘控制

```powershell
python scripts/teleop_chassis.py --transport serial --port COM12
```

按键：

- `G`：使能。
- `W/S`：前进/后退。
- `A/D`：左移/右移。
- `Z/C`：逆时针/顺时针。
- `X`：停车并解除软件运行状态。
- `ESC`：停车、失能并退出。

每次按键只保持 0.20 秒，持续运动必须按住；PC 超过 300 ms 不发指令时，ESP32 会停车并失能。

## 有线视觉闭环

在 `config/navigation.yaml` 设置：

```yaml
driver:
  type: serial
  port: COM12
  baudrate: 115200
  command_timeout_s: 0.25
```

先干跑：

```powershell
python scripts/run_closed_loop_navigation.py --x 0.60 --y 1.20 --theta 0
```

再实车：

```powershell
python scripts/run_closed_loop_navigation.py --x 0.60 --y 1.20 --theta 0 --enable-motion
```

视觉窗口按 `G` 才会使能，空格停车，`Q` 停车并退出。

## 恢复 PS2

把板端 `control_config.py` 改为：

```python
CONTROL_MODE = "ps2"
```

只重新上传该文件并复位即可。PS2 模式不会启动 PC 串口或 UDP 控制，避免抢控制权。
