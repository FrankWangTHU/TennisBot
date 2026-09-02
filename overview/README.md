# TennisBot V1 全局视觉定位

使用 iPhone + Camo Camera + Windows OpenCV，在固定平面场地中实时输出车顶 AprilTag 的世界位姿 `(x, y, theta)` 和网球的世界坐标。视觉闭环底盘控制和完整的安全调试步骤见 [NAVIGATION.md](NAVIGATION.md)。

## 环境准备

建议使用 Windows 和 64 位 Python 3.10–3.13（本项目已在 Python 3.13 完成自动化测试）。先启动 Camo，确认 iPhone 画面在 Camo 中稳定显示，并关闭可能占用虚拟摄像头的其他软件。

在本目录打开 PowerShell：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

如果 PowerShell 阻止激活脚本，可只对当前窗口执行：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\.venv\Scripts\Activate.ps1
```

## 首次设置与运行

1. 扫描摄像头：

   ```powershell
   python scripts/list_cameras.py
   ```

2. 将 Camo 对应编号写入 `config/camera.yaml` 的 `camera.index`，然后预览：

   ```powershell
   python scripts/preview_camera.py
   ```

3. 在 `config/field.yaml` 填写真实 `width_m`、`height_m`。固定手机的机位、焦距和 Camo crop 后，执行四点标定：

   ```powershell
   python scripts/calibrate_field.py
   ```

   严格按 `(0,0) 左下 → (W,0) 右下 → (W,H) 右上 → (0,H) 左上` 点击。按 `U` 撤销、`R` 重来、`S` 保存。结果写入 `data/calibration/homography.yaml`。

4. 确认打印的 Tag family，并在 `config/perception.yaml` 设置 `apriltag.family`、`robot_tag_id` 和安装方向 `front_edge`。测试像素检测：

   ```powershell
   python scripts/test_apriltag.py
   ```

5. 测试世界位姿与方向：

   ```powershell
   python scripts/test_apriltag.py --world
   ```

6. 可先单独检查网球检测：

   ```powershell
   python scripts/test_ball_detection.py
   ```

7. 启动完整系统：

   ```powershell
   python scripts/run_global_tracking.py
   ```

   程序会单独打开 `Ball Auto Tuning` 状态窗口，不需要拖动任何滑条。在 `Global Tracking` 主画面依次左键点击 2–5 颗真实网球，程序会从球体周围自动学习 HSV、面积和圆度，每次点击后立即应用并保存到 `config/perception.yaml`。右键清空本轮标记，`U` 撤销最后一个标记。

   快捷键：`Q/ESC` 退出，`U` 撤销最后一个自动标注，`S` 保存原图、mask 和检测 JSON，`H` 显示 mask，`W` 按需显示黑色俯视图，`R` 重新加载配置和标定。

## 验收测试

自动化数学与图像单元测试不需要摄像头：

```powershell
python -m pytest -q
```

位置误差测试前，先把 `config/field.yaml` 中的 `verification_points` 改成可实际测量的地面点：

```powershell
python scripts/verify_field_calibration.py
```

依次把球或 marker 放到窗口提示的 GT 点并点击其中心。目标误差小于 `0.05–0.10 m`。再把车分别朝世界坐标 `+X/+Y/-X/-Y` 摆放，运行 `test_apriltag.py --world`，期望角度依次约为 `0°/90°/±180°/-90°`。如果整体固定偏转 90° 或 180°，修改 `front_edge`；如果轴方向反了，重新检查四角点击顺序。

## 坐标约定

- 图像坐标：左上角为原点，`u` 向右，`v` 向下。
- 世界坐标：场地左下角为 `(0,0)`，长边为 `+X`，短边为 `+Y`，单位为米。
- `theta` 内部为弧度、逆时针为正；显示时使用角度。
- 图像中的朝向不会直接当成世界朝向：Tag 中心和前向点都会经过 Homography 后再计算角度。

## 常见问题与限制

- 标定后移动手机、改变焦距、Camo crop 或分辨率，必须重新标定。
- iPhone 自动曝光、阴影和地面反光会改变 HSV，光照变化后应重新调参。
- Tag 太小、模糊或过曝会漏检；优先增大打印尺寸和改善照明。
- 约 1.8 m 高的斜俯视机位可以使用 Homography，不要求摄像头竖直向下；但标定后不能再移动机位。
- 单应性只适合地面平面。球心像素映射会受球半径和斜视影响，V1 的目标是 5–10 cm，不是毫米精度。
- 默认在 `0.5×` 缩小图像上检测，再将坐标映射回原图以提高刷新率；若远处 AprilTag 太小，可把 `performance.processing_scale` 改成 `0.75` 或 `1.0`。
- 球 ID 在当前程序运行期间保持稳定；程序重启后会重新编号。

## 遮挡记忆与夹取判定

完整追踪现在为每颗球分配运行期内稳定的 `track_id`。球被机器人、机械臂或人员遮挡时，画面会用紫色 `MEMORY` 标记保留最后世界坐标；只要机器人仍在该球附近，球不会被删除。机器人离开后，如果原位置连续约 30 帧重新呈现为白色低饱和地面、同时仍检测不到球，才确认球已被夹走并从状态中移除。程序重启后记忆会重新建立。

AprilTag 使用原始分辨率检测并启用专用角点精修，网球仍使用 `0.5×` 图像以维持刷新率。现场建议优先使用一个更大的哑光 Tag，而不是直接贴多个未经标定的 Tag：黑色编码区建议至少 8–10 cm，连同白边约 10–15 cm，并平整安装在机器人最高、最少遮挡的位置。

后续阶段应基于 `GlobalTracker.get_world_state()` 的标准输出实现目标位姿与视觉闭环 `goto_pose()`，本版本尚未包含控制逻辑。
