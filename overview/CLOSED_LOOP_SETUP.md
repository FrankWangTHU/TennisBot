# CV + UDP 视觉闭环导航调试手册

## 当前闭环做了什么

主程序 `scripts/run_closed_loop_navigation.py` 每帧执行：

1. Camo/OpenCV 采集图像。
2. AprilTag + Homography 输出世界位姿 `(x, y, theta)`。
3. 按配置的视觉延迟预测当前位姿。
4. 把世界坐标误差旋转到车体坐标，计算 `vx / vy / omega`。
5. 通过 UDP 以车体速度控制 ESP32。
6. 到达目标、AprilTag 丢失、UDP 异常、空格或退出时停车并失能。

程序默认不允许运动。即使命令行带了 `--enable-motion`，也必须连续看到 5 帧 AprilTag 后手动按 `G`。

## 0. 保证相机和小车网络能同时使用

如果 Camo 通过 Wi-Fi 连接 iPhone，而电脑内置 Wi-Fi 又连接 `TennisBot`，Camo 会断开。任选一种：

- iPhone 用 USB 连接 Camo，电脑 Wi-Fi 连接 TennisBot；最简单。
- 内置 Wi-Fi 连接原网络/Camo，USB Wi-Fi 网卡连接 TennisBot。
- 电脑用网线连接互联网，Wi-Fi 连接 TennisBot。

先运行 `run_global_tracking.py`，保持 60 秒，确认 FPS、AprilTag 和世界位姿连续稳定，再进行运动测试。

## 1. 确认 ESP32 和 UDP

ESP32 的 `control_config.py`：

```python
CONTROL_MODE = "udp"
```

不使能电机测试 200 个 UDP 往返包：

```powershell
python scripts/diagnose_udp_link.py --host 192.168.4.1 --token change-me-tennisbot --count 200
```

建议 `lost=0`、`p95<50 ms`。然后用 `teleop_chassis.py` 再确认 W/S/A/D/Z/C 方向正确。

## 2. 配置保守的闭环参数

在 `config/navigation.yaml` 设置：

```yaml
controller:
  kp_xy: 0.7
  kp_theta: 1.2
  max_linear_mps: 0.05
  max_angular_radps: 0.25
  max_linear_accel_mps2: 0.10
  max_angular_accel_radps2: 0.5
  position_tolerance_m: 0.07
  heading_tolerance_deg: 10.0

latency:
  measurement_latency_s: 0.9
  pose_loss_grace_s: 0.25

safety:
  min_pose_frames_to_arm: 5

driver:
  type: udp
  host: 192.168.4.1
  udp_port: 5005
  token: change-me-tennisbot
  command_timeout_s: 0.25
  enable_timeout_s: 2.0
```

令牌必须与 ESP32 的 `wifi_config.py` 一致。

## 3. 选择第一个目标

从 `run_global_tracking.py` 画面读取当前位置 `(x0,y0,theta0)`。第一个目标只移动 0.15-0.20 m，并且必须位于 `0<=x<=1.25`、`0<=y<=2.0` 场地内。

例如当前位置约 `(0.40,0.50)`，先设目标 `(0.55,0.50)`。第一次使用 `--position-only`，完全禁用旋转控制：

```powershell
python scripts/run_closed_loop_navigation.py --x 0.55 --y 0.50 --position-only
```

这是干跑，不会连接真实底盘。检查：

- 紫色 TARGET 在预期位置。
- AprilTag 连续时 `Pose ready` 达到 `5/5`。
- 遮挡 Tag 超过 0.25 秒后显示 `pose_lost`。
- 未按 `G` 时 `Sent` 为零；按 `G` 后可观察模拟速度，但 dry-run 不连接、也不会驱动真实底盘。

## 4. 架空轮子进行第一次闭环使能

四轮架空，使用相同目标：

```powershell
python scripts/run_closed_loop_navigation.py --x 0.55 --y 0.50 --position-only --enable-motion
```

确认 `Pose ready=5/5` 后按 `G`。只观察轮子方向 1-2 秒，立即按空格。由于车轮架空后视觉位置不会变化，控制器会继续请求移动，不能长时间架空运行。

## 5. 地面位置闭环

把车放到场地，清理线缆，实体急停放在手边。继续用 `--position-only` 和 0.15-0.20 m 的近目标：

```powershell
python scripts/run_closed_loop_navigation.py --x 0.55 --y 0.50 --position-only --enable-motion
```

按 `G` 后观察。到达 7 cm 容差并稳定 0.6 秒时，程序自动停车、失能，并显示 `ARRIVED`。

连续做三个方向的小距离目标：世界 +X、世界 +Y、斜向。三次都应能收敛，没有方向反转。

## 6. 单独测试朝向闭环

把目标 x/y 设置为当前车位置，只把 theta 改 15-20 度，不要带 `--position-only`：

```powershell
python scripts/run_closed_loop_navigation.py --x 0.55 --y 0.50 --theta 20 --enable-motion
```

正角度应逆时针。先测试 `20`，再测试 `-20`。不要一开始测试 90 或 180 度。

## 7. 完整位姿闭环

位置和朝向分别稳定后再组合：

```powershell
python scripts/run_closed_loop_navigation.py --x 0.75 --y 0.70 --theta 20 --enable-motion
```

麦克纳姆底盘允许平移和旋转同时进行。若组合后振荡，先降低 `kp_theta`，不要先提高速度。

## 8. 如何调参

每次运行会把 CSV 保存到 `data/navigation/`，包括测量位姿、延迟补偿位姿、目标、误差和实际发送速度。

- 靠近目标很慢：`kp_xy` 每次增加 0.1，最高先试到 1.0。
- 冲过目标并来回摆：把 `kp_xy` 降低 20%，或降低 `max_linear_mps`。
- 转向来回摆：降低 `kp_theta` 或 `max_angular_radps`。
- 起步打滑：降低 `max_linear_accel_mps2`。
- 总是提前停车：`measurement_latency_s` 可能过大。
- 总是冲过目标：视觉延迟可能比配置大；每次只增加 0.1 秒并复测。
- 位姿本身跳动：不要调控制器，先改善 AprilTag 尺寸、照明和相机稳定性。

一次只改一个参数，每个参数至少重复同一目标三次。

## 9. 必须通过的安全验收

1. 未达到连续 5 帧 Tag 时按 G，程序拒绝使能。
2. 运动中遮挡 Tag，约 0.25 秒后停车失能。
3. 运动中断开 TennisBot Wi-Fi，ESP32 最迟约 0.6 秒停车失能。
4. 按空格立即停车并失能。
5. 到达目标后自动停车并失能。
6. 目标超出 1.25 x 2.0 m 场地时程序拒绝启动。

全部通过后，才把 `max_linear_mps` 从 0.05 逐步提高到 0.08；仅依靠约 1 秒延迟的视觉时，暂不建议超过 0.10 m/s。
