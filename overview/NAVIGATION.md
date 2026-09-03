# PC 视觉闭环导航：接线、刷写与逐步测试

## 方案

PC 从 Camo 图像得到 AprilTag 世界位姿，控制器把目标误差转换为车体坐标的 `vx / vy / omega`，再经 USB 串口发送给 ESP32-S3。ESP32 只负责麦克纳姆运动学和 CAN 电机控制，并带 300 ms 指令看门狗。坐标约定与现有底盘代码一致：`+vx` 前进、`+vy` 左移、`+omega` 逆时针。

接近 1 秒的图像延迟对闭环明显偏大：0.08 m/s 时反馈已落后约 8 cm。本实现按配置的 `measurement_latency_s` 做恒速位姿预测，短暂漏 Tag 最多预测 0.25 s，之后立即发零速；这能支持低速验证，但不能代替里程计/IMU 的高频局部闭环。

遥控器作为独立模式保留。在板端 `control_config.py` 中选择 `serial`、`udp` 或 `ps2`；三种模式互斥，避免两套控制源同时覆盖电机命令。

## 0. 安全准备

1. 第一次测试必须把四个轮子架空，机械臂收拢，清空车旁人员和线缆。
2. 保留实体急停和 PS2 遥控器，但 PC 控制期间不要同时发送遥控指令。
3. 先拔电机动力电，只给 ESP32 供电完成串口测试；收到 PING 后再接动力。

## 1. PC 环境

在 `overview` 目录打开 PowerShell：

```powershell
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python scripts/list_serial_ports.py
```

如果没有 COM 口，换一根支持数据的 USB 线，并安装资料中的 CH340/CH341 驱动。记下 ESP32 对应的 COM 号。

## 2. 刷写 ESP32 串口桥

先把 `firmware/esp32_chassis_bridge/control_config.py` 设置为 `CONTROL_MODE = "serial"`，再按照该目录 README 将完整的 9 个文件上传到板子根目录。重启后 Shell 应出现 `READY:TennisBot serial chassis bridge`。

## 3. 只测通信，不转电机

关闭 Thonny（它会占用 COM 口），执行：

```powershell
python scripts/test_chassis_serial.py --port COM12
```

把 `COM12` 换成实际端口。看到 `PING 成功` 才进入下一步。若超时，确认波特率为 115200、Thonny 串口已断开、板子运行的是新 `main.py`。

## 4. 架空轮子核对三个正方向

依次执行，每次只运动 0.5 秒：

```powershell
python scripts/test_chassis_serial.py --port COM12 --axis vx --value 0.05 --duration 0.5 --enable-motion
python scripts/test_chassis_serial.py --port COM12 --axis vy --value 0.05 --duration 0.5 --enable-motion
python scripts/test_chassis_serial.py --port COM12 --axis omega --value 0.15 --duration 0.5 --enable-motion
```

期望分别是前进、左移、逆时针。方向不对时先检查 1–4 号电机位置和接线，不要直接在导航控制器里胡乱反号。每条命令结束、PC 崩溃或串口中断后，ESP32 的 300 ms 看门狗都会停车。

## 5. 地面低速开环

把线速度降到 `0.03`，持续 `0.3` 秒测试前进和横移；确认轮子方向、打滑、线缆拖拽都正常后，最多逐步升到 `0.05`。实体急停必须在手边。

## 6. 视觉闭环干跑（小车不会动）

保持 `config/navigation.yaml` 中 `driver.type: dry_run`：

```powershell
python scripts/run_closed_loop_navigation.py --x 0.60 --y 1.20 --theta 0
```

画面中紫色十字是目标。观察 `Cmd robot`：目标在车头前方时 `vx` 应为正，目标在车体左侧时 `vy` 应为正，目标角度逆时针方向时 `w` 应为正。遮挡 Tag 超过 0.25 s 后必须显示 `pose_lost` 且三项命令归零。此阶段按 `G` 也只是启用 dry-run。

## 7. 实车闭环

把 `config/navigation.yaml` 改为：

```yaml
driver:
  type: serial
  port: COM12
  baudrate: 115200
  command_timeout_s: 0.25
```

首次目标只设在当前车前方 0.20–0.30 m，并保持 `max_linear_mps: 0.05`。启动：

```powershell
python scripts/run_closed_loop_navigation.py --x 0.60 --y 1.20 --theta 0 --enable-motion
```

程序启动后仍为 `SAFE/DISARMED`；确认画面位姿正确再按 `G`。空格立即停车并解除软件使能，`Q` 停车、失能并退出。

## 8. 调参顺序

1. 实测端到端延迟：快速移动 Tag，录像比较真实动作和叠加位姿，修改 `measurement_latency_s`。
2. 先只调 `kp_xy`，从 0.5 开始；能稳定靠近目标后再增大，出现来回振荡就降低。
3. 再调 `kp_theta`，从 1.0 开始。
4. 最后逐步提高 `max_linear_mps`，在只有视觉反馈时建议先不超过 0.10 m/s。
5. 到点抖动时增加 `position_tolerance_m` 到 0.07–0.10；朝向抖动时增大 `heading_tolerance_deg`。

若希望明显提高速度，下一步应在 ESP32/车体侧加入轮速里程计或 IMU 做 50–100 Hz 内环，PC 视觉只以 10–30 Hz 校正漂移；单靠约 1 秒延迟的视觉不适合快速闭环。
