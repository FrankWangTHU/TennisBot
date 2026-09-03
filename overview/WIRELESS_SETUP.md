# ESP32-S3 无线控制：烧录与验收操作手册

## 已确认的硬件条件

项目附带的 `MicroPython1.27.bin` 已核对为 `Generic ESP32S3 module with Octal-SPIRAM`。固件镜像同时包含 Wi-Fi 网络组件和项目代码所需的 `esp32.CAN` 类。因此使用同一块 ESP32-S3 建立热点、接收 UDP，并通过 CAN 控制底盘在软件层面可行。

必须优先使用项目附带的固件，不要换成官网普通 MicroPython：普通构建不保证带有厂家增加的 `esp32.CAN`。

## 需要的工具和文件

1. 一根确认支持数据传输的 USB 线。
2. 板卡上的 `UART 调试口`；硬件资料的控制模块图片在板卡顶部标出了 `UART 调试口` 和 `USB口`，首次烧录优先连接前者。
3. `车体控制部分/驱动/驱动/CH341SER.EXE`：Windows 看不到 CH340 串口时安装。
4. `车体控制部分/thonny-4.1.7.exe`：安装并运行 Thonny。
5. `车体控制部分/MicroPython1.27.bin`：厂家 MicroPython 固件。
6. `firmware/esp32_chassis_bridge/`：需要上传到开发板的控制程序。

固件 SHA-256 应为：

```text
DA9F72452B1B0CB2314DE759C9B0B38C649A5E75837C0BD9A771628E7202E508
```

## 第一阶段：只连接控制板

1. 关闭车体 12 V/48 V 电机动力，只通过 USB 给控制板供电。
2. USB 线插到控制板的 `UART 调试口`。
3. 打开 Windows 设备管理器 -> 端口（COM 和 LPT）。
4. 拔插 USB，记住随之消失/出现的 COM 号，例如 `USB-SERIAL CH340 (COM12)`。
5. 如果没有新端口，运行 `CH341SER.EXE` 安装驱动，然后重新插拔；仍没有则更换数据线。

## 第二阶段：先判断是否需要重刷底层固件

1. 打开 Thonny 4.1.7。
2. 选择 `运行 -> 配置解释器`（也可点右下角解释器名称）。
3. 解释器选择 `MicroPython (ESP32)`，端口选择刚找到的 COM。
4. 点确定，按工具栏红色停止按钮或 `Ctrl+F2`，Shell 应出现 `>>>`。
5. 在 Shell 逐行执行：

```python
import sys
print(sys.implementation)
import network
from esp32 import CAN
print("firmware ok")
```

如果最后打印 `firmware ok`，说明当前固件已经具备 Wi-Fi 和 CAN，直接跳到第三阶段，不必擦除重刷。

如果不能进入 `>>>`、`import network` 失败或 `from esp32 import CAN` 失败，再执行下面的完整烧录。

## 第二阶段 B：用 Thonny 烧录厂家 MicroPython

烧录会清空开发板上原有文件；原厂代码已经保存在电脑资料目录中。

1. 在 Thonny 打开 `运行 -> 配置解释器`。
2. 选择 `MicroPython (ESP32)` 和正确 COM 端口。
3. 点击 `安装或更新 MicroPython (esptool)`。
4. 目标/芯片选择 `ESP32-S3`。
5. 固件选择 `本地文件`，定位到 `车体控制部分/MicroPython1.27.bin`。
6. 勾选 `擦除闪存`，地址保持 ESP32-S3 的 `0x0`，开始安装。
7. 如果无法自动进入下载模式：按住板上的 `BOOT`，短按一下 `RESET/EN`，松开 `RESET/EN`，再松开 `BOOT`，然后重新烧录。
8. 高速烧录中途失败时降低波特率到 115200 再试。
9. 完成后重新选择 `MicroPython (ESP32)` 解释器并运行上面的 `firmware ok` 检查。

不要在烧录过程中断电、拔线或打开其他占用该 COM 口的软件。

## 第三阶段：配置无线参数

打开 `firmware/esp32_chassis_bridge/wifi_config.py`，至少修改密码和控制令牌：

```python
AP_SSID = "TennisBot"
AP_PASSWORD = "换成至少8位的热点密码"
CONTROL_TOKEN = "换成一段不含空格的随机字符串"
UDP_PORT = 5005
```

然后把同一个 `CONTROL_TOKEN` 写入 `config/navigation.yaml` 的 `driver.token`。两边必须完全一致。

## 第四阶段：选择 UDP 模式并上传完整烧录包

1. 先把 `control_config.py` 设置为 `CONTROL_MODE = "udp"`。
2. 在 Thonny 中按红色停止按钮，确认 Shell 是 `>>>`。
3. 选择 `视图 -> 文件`，上方是电脑文件，下方是 MicroPython 设备文件。
4. 在电脑侧进入 `overview/firmware/esp32_chassis_bridge`。
5. 把以下文件逐个上传到设备根目录 `/`，文件名不能修改：

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

可以右键文件选择“上传到 /”，也可以打开文件后选择 `文件 -> 另存为 -> MicroPython 设备`。

6. 确认设备文件列表中九个文件都存在。
7. 按控制板 RESET，或在 Shell 按 `Ctrl+D` 软重启。
8. Shell 应显示：

```text
READY:TennisBot udp chassis bridge
WIFI:ssid=TennisBot ip=192.168.4.1 udp=5005
```

若出现 `ImportError`，通常是文件漏传；若出现 CAN 初始化错误，检查是否刷入了项目附带固件。

## 第五阶段：电脑连接小车热点

1. Windows 右下角打开 Wi-Fi。
2. 连接 `TennisBot`，输入 `wifi_config.py` 中的密码。
3. Windows 提示“无 Internet”是正常现象，保持连接。
4. USB 线此时可以保留，仅用于供电和查看日志；UDP 控制不走 USB。
5. 在 `overview` 打开 PowerShell：

```powershell
.\.venv\Scripts\Activate.ps1
python scripts/test_chassis_udp.py --host 192.168.4.1 --token 你的控制令牌
```

看到 `UDP PING 成功`，无线链路就已经打通，而且这条命令不会使能电机。

如果超时：确认电脑确实连接 `TennisBot`、令牌一致、ESP32 Shell 没有报错，并暂时关闭 VPN。不要先接电机动力排查网络问题。

## 第六阶段：验证失联保护

1. 暂时仍不接地运行，把四轮架空。
2. 执行一次短动作测试。
3. 动作过程中关闭 PowerShell 或断开电脑 Wi-Fi。
4. ESP32 必须在约 600 ms 内停车并失能，USB Shell 显示：

```text
WARN:WATCHDOG:stopped_and_disabled
```

失联后必须重新执行 ENABLE；导航窗口中就是重新按 `G`。

正式运动前可先连续测 200 个不使能电机的 UDP 往返包：

```powershell
python scripts/diagnose_udp_link.py --host 192.168.4.1 --token TOKEN --count 200
```

建议丢包为 0，`p95` 小于 50 ms，最大值明显低于 600 ms。如果这里稳定而键盘仍失控，优先排查程序状态；如果这里本身大量丢包，再调整电脑网卡、电源和现场 2.4 GHz 干扰。

## 第七阶段：架空轮子测试

将 `TOKEN` 替换为实际令牌，依次执行：

```powershell
python scripts/test_chassis_udp.py --token TOKEN --axis vx --value 0.05 --duration 0.5 --enable-motion
python scripts/test_chassis_udp.py --token TOKEN --axis vy --value 0.05 --duration 0.5 --enable-motion
python scripts/test_chassis_udp.py --token TOKEN --axis omega --value 0.15 --duration 0.5 --enable-motion
```

预期分别为前进、左移、逆时针。每次脚本结束都会发送停车和失能。

## 第八阶段：接入视觉闭环

把 `config/navigation.yaml` 修改为：

```yaml
driver:
  type: udp
  host: 192.168.4.1
  udp_port: 5005
  token: TOKEN
  command_timeout_s: 0.3
```

先执行不带运动权限的干跑：

```powershell
python scripts/run_closed_loop_navigation.py --x 0.60 --y 1.20 --theta 0
```

确认坐标方向和 AprilTag 丢失停车正确后，再执行：

```powershell
python scripts/run_closed_loop_navigation.py --x 0.60 --y 1.20 --theta 0 --enable-motion
```

程序仍以 `SAFE/DISARMED` 启动。确认画面正确后按 `G` 使能；空格紧急停车；`Q` 停车、失能并退出。首次地面速度保持 0.03-0.05 m/s。

## 常见故障

- 看不到 COM：安装 CH341 驱动、换数据线、确认插的是 UART 调试口。
- Thonny 显示端口忙：关闭串口助手、底盘调试软件和正在运行的 PC 控制脚本。
- 烧录无法连接：使用 BOOT + RESET/EN 手动进入下载模式。
- 烧录后没有 `esp32.CAN`：刷错了通用固件，重新刷项目的 `MicroPython1.27.bin`。
- 能连热点但 UDP 超时：令牌错误、VPN/防火墙路由干扰，或 `main.py` 没有正常启动。
- UDP 正常但电机不动：确认电机动力、CAN 接线、急停状态和四个电机 ID。
