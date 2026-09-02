# TennisBot V1 — 全局视觉定位原型（Agent 开发说明）

## 0. 任务目标

请为 TennisBot 项目实现第一版“高位视觉定位原型”。

当前硬件与场景：

- 摄像头：iPhone 16 Pro Max
- 视频输入：Camo Camera，将 iPhone 作为 Windows 摄像头使用
- 运行平台：Windows PC
- 视觉框架：Python + OpenCV
- 小车顶部已有 AprilTag
- 场地为实验室平面场地，地面已有明显边界划线
- 场地中散布多个黄色/黄绿色网球
- 本阶段暂时不控制小车，不做 RGBD，不做机械臂，不做路径规划

本阶段只需要完成：

1. 从 Camo Camera 实时读取 iPhone 视频；
2. 检测小车顶部 AprilTag；
3. 识别 AprilTag 的中心位置；
4. 识别 AprilTag 的朝向；
5. 检测所有网球；
6. 得到每个网球的图像中心坐标；
7. 识别或人工标定场地边界；
8. 将 AprilTag 和网球从图像像素坐标转换到场地世界坐标；
9. 实时可视化：
   - 小车 `(x, y, theta)`
   - 所有网球 `(x, y)`
   - 场地边界
   - AprilTag 朝向箭头
10. 保存必要的标定参数，下一次运行时可直接加载。

---

# 1. V1 的最终验收效果

运行：

```bash
python scripts/run_global_tracking.py
```

后弹出实时窗口。

窗口中应至少显示：

- 场地边界；
- AprilTag 四角；
- AprilTag ID；
- AprilTag 中心；
- 小车方向箭头；
- 小车世界坐标：
  ```text
  Robot: x=1.82 m, y=0.73 m, theta=42.5 deg
  ```
- 每颗网球圆心；
- 网球编号；
- 每颗网球的世界坐标：
  ```text
  Ball 0: (0.72, 1.33)
  Ball 1: (2.10, 0.86)
  Ball 2: (4.22, 2.15)
  ```

画面应该持续实时更新。

---

# 2. 当前阶段的系统链路

```text
iPhone 16 Pro Max
        ↓
     Camo Camera
        ↓
Windows Virtual Camera
        ↓
OpenCV VideoCapture
        ↓
   ┌──────────────┐
   │   Frame      │
   └──────┬───────┘
          │
     ┌────┴────┐
     ↓         ↓
AprilTag     Tennis Ball
Detection    Detection
     ↓         ↓
center        centers
heading       (u,v)
     └────┬────┘
          ↓
    Homography
          ↓
     World Frame
          ↓
Robot (x,y,theta)
Balls [(x,y), ...]
          ↓
Visualization / Logging
```

---

# 3. 坐标系定义

整个项目从 V1 开始必须明确坐标系。

## 3.1 图像坐标系 `image`

OpenCV 默认：

```text
origin = 图像左上角

u → 向右
v ↓ 向下
```

像素坐标：

```python
(u, v)
```

## 3.2 场地坐标系 `world`

默认将场地左下角定义为 `(0, 0)`。

```text
+X = 场地长边方向
+Y = 场地短边方向
```

例如 3 m × 5 m Demo：

```text
            +Y
             ↑
             │
(0,3)  ───────────────  (5,3)
       │               │
       │               │
       │               │
(0,0)  ───────────────  (5,0)
             └────────→ +X
```

统一单位：

```text
distance: meter
angle: radian internally
display: degree allowed
```

---

# 4. 摄像头输入

## 4.1 Camo Camera

iPhone 16 Pro Max 通过 Camo Camera 连接 Windows。

程序不要假设固定摄像头 index。

请实现：

```bash
python scripts/list_cameras.py
```

自动扫描：

```python
cv2.VideoCapture(0)
cv2.VideoCapture(1)
...
```

输出例如：

```text
Camera 0: available
Camera 1: unavailable
Camera 2: available
```

然后在配置文件中指定：

```yaml
camera:
  index: 2
```

## 4.2 推荐采集配置

V1 优先：

```text
1920 × 1080
20~30 FPS
```

如果 Camo 能稳定提供 4K，可以后续测试，但第一版优先保证实时性。

建议 iPhone 使用：

```text
1× 主摄
```

暂时避免：

```text
0.5× 超广角
```

因为边缘径向畸变更强。

## 4.3 摄像头固定要求

运行标定以后：

- 不要移动手机；
- 不要改变手机俯仰；
- 不要改变相机焦距；
- 不要改变 Camo crop；
- 不要数字变焦。

否则 Homography 需要重新标定。

---

# 5. AprilTag 检测

优先使用 OpenCV ArUco 模块支持的 AprilTag 字典。

安装：

```bash
pip install opencv-contrib-python numpy pyyaml
```

建议：

```python
cv2.aruco.DICT_APRILTAG_36h11
```

如果实际打印的 Tag family 不是 `36h11`，必须改成对应 family，不要猜。

---

# 6. AprilTag 输出

每一帧至少输出：

```python
AprilTagDetection(
    tag_id: int,
    center_uv: tuple[float, float],
    corners_uv: np.ndarray,
    heading_image_rad: float,
)
```

其中 `center_uv` 为四角平均值。

---

# 7. AprilTag 朝向识别

这是 V1 的重点之一。

AprilTag 检测结果包含有序四角。

定义一个固定的 Tag 本体方向，例如：

```text
Tag 上边方向 = 小车前进方向
```

或者：

```text
Tag 左到右方向 = 小车前进方向
```

必须在代码和文档里明确。

推荐做法：

```python
front_vector = midpoint(top_edge) - center
```

或者根据实际安装方式选择对应边。

图像中先得到：

```python
theta_image
```

然后不要直接把 `theta_image` 当世界角度。

必须：

1. 将 Tag 中心映射到 world；
2. 将 Tag 前方方向上的一个辅助像素点也映射到 world；
3. 在 world frame 下重新计算：

```python
theta_world = atan2(
    front_world_y - center_world_y,
    front_world_x - center_world_x
)
```

这样即使摄像头斜拍，朝向仍然正确。

---

# 8. 小车位姿定义

最终输出：

```python
RobotPose(
    x: float,
    y: float,
    theta: float,
)
```

其中：

```text
x, y: meter
theta: radian
```

显示时可转换为 `theta_deg`。

---

# 9. 网球检测 V1

第一版不要使用 YOLO。

优先使用：

```text
HSV Color Segmentation
+
Morphology
+
Contour Filtering
```

因为实验室环境固定，网球颜色明显，方便 debug。

---

# 10. 网球检测流程

推荐：

```text
BGR Frame
   ↓
HSV
   ↓
Color Threshold
   ↓
Morphological Open / Close
   ↓
findContours
   ↓
Area Filter
   ↓
Circularity Filter
   ↓
center (u,v)
```

圆度：

```text
circularity = 4πA / P²
```

可以作为筛选指标。

---

# 11. HSV 参数不要写死

建立：

```yaml
ball_detection:
  hsv_lower: [H1, S1, V1]
  hsv_upper: [H2, S2, V2]

  min_area: 50
  max_area: 10000

  min_circularity: 0.45
```

同时提供一个调参脚本：

```bash
python scripts/tune_ball_hsv.py
```

使用 OpenCV Trackbar 实时调整：

```text
H min
H max
S min
S max
V min
V max
```

按 `S` 保存到 YAML。

---

# 12. 网球检测输出

统一：

```python
BallDetection(
    center_uv: tuple[float, float],
    radius_px: float,
    area_px: float,
)
```

通过 Homography 后：

```python
BallWorld(
    x: float,
    y: float,
)
```

第一版不需要跨帧永久 ID。

可以每帧按照 X 优先、再 Y 排序后编号。

不要在 V1 过早做 tracking。

---

# 13. 场地边界

用户的实验场地已经有明显划线。

V1 需要支持两种模式。

## Mode A — 手动四点标定（必须先实现）

第一次运行：

```bash
python scripts/calibrate_field.py
```

显示当前 Camo 图像。

用户按顺序点击：

```text
world (0,0)
world (W,0)
world (W,H)
world (0,H)
```

例如：

```text
左下
右下
右上
左上
```

程序计算 `H_image_to_world` 并保存。

配置：

```yaml
field:
  width_m: 5.0
  height_m: 3.0
```

保存：

```text
data/calibration/homography.yaml
```

这是 V1 必须首先跑通的方式。

## Mode B — 自动场地线检测（可选增强）

因为地面已有明显划线，可以尝试：

```text
HSV / grayscale
↓
Canny
↓
HoughLinesP
↓
找到四条主边界
↓
求交点
↓
自动生成场地四角
```

但这一项不是 V1 blocker。

如果不稳定，保留手动标定即可。

不要为了自动识别边界阻塞整个项目。

---

# 14. Homography

实现统一接口：

```python
class FieldTransform:
    def image_to_world(self, uv):
        ...

    def world_to_image(self, xy):
        ...
```

不要在多个脚本里重复写矩阵运算。

---

# 15. 可视化

实时窗口建议至少绘制：

### Field

- 场地边界；
- 四角；
- world coordinate axes。

### AprilTag

- Tag outline；
- Tag ID；
- center；
- front arrow；
- `(x, y, theta)`。

### Tennis Balls

- circle；
- center；
- Ball ID；
- world `(x,y)`。

---

# 16. Bird's-Eye Debug View

非常建议额外做一个：

```text
Top-down world view
```

例如：

```text
3 m
↑
│     ○ Ball 2
│
│             ○ Ball 1
│
│       ↑
│      Robot
│
└────────────────→ 5 m
```

这个视图不要直接用图像，应根据 world 坐标重新绘制。

这样可以快速判断：

```text
Homography 是否正确
AprilTag 方向是否正确
球坐标是否正确
```

---

# 17. 推荐目录结构

```text
TennisBot/
│
├── README.md
├── requirements.txt
│
├── config/
│   ├── camera.yaml
│   ├── field.yaml
│   └── perception.yaml
│
├── tennisbot/
│   ├── __init__.py
│   │
│   ├── camera/
│   │   ├── camera_source.py
│   │   └── camo_camera.py
│   │
│   ├── calibration/
│   │   ├── field_calibration.py
│   │   └── homography.py
│   │
│   ├── perception/
│   │   ├── apriltag_detector.py
│   │   ├── ball_detector.py
│   │   └── models.py
│   │
│   ├── localization/
│   │   └── robot_pose.py
│   │
│   └── visualization/
│       ├── overlay.py
│       └── world_view.py
│
├── scripts/
│   ├── list_cameras.py
│   ├── preview_camera.py
│   ├── calibrate_field.py
│   ├── tune_ball_hsv.py
│   ├── test_apriltag.py
│   ├── test_ball_detection.py
│   └── run_global_tracking.py
│
└── data/
    └── calibration/
```

保持轻量，暂时不要引入 ROS。

---

# 18. 推荐数据模型

使用 dataclass。

例如：

```python
@dataclass
class RobotPose:
    x: float
    y: float
    theta: float


@dataclass
class BallWorld:
    x: float
    y: float
```

---

# 19. 主程序逻辑

`run_global_tracking.py`：

```text
load config
↓
open Camo camera
↓
load homography
↓
while running:
    read frame

    detect AprilTag
    detect tennis balls

    if AprilTag:
        convert center → world
        convert heading → world
        build RobotPose

    for every ball:
        center pixel → world xy

    draw image overlay
    draw bird's-eye world view

    show FPS
    show robot pose
    show ball count
↓
release camera
```

---

# 20. V1 的快捷键

建议：

```text
Q / ESC
退出

S
保存当前帧到 data/debug/

H
显示/隐藏 HSV mask

W
显示/隐藏 world view

R
reload configuration
```

---

# 21. 必须保存 Debug 数据

出现问题时，不要只依赖实时窗口。

按 `S` 保存：

```text
frame_timestamp.jpg
mask_timestamp.png
detections_timestamp.json
```

JSON 示例：

```json
{
  "robot": {
    "x": 1.52,
    "y": 0.82,
    "theta": 0.72
  },
  "balls": [
    [0.52, 1.24],
    [2.82, 0.71]
  ]
}
```

---

# 22. 标定误差验证

标定完成以后，需要一个简单验证脚本：

```bash
python scripts/verify_field_calibration.py
```

用户将网球或 marker 放在几个已知位置，例如：

```text
(1.0, 1.0)
(2.5, 1.5)
(4.0, 2.0)
```

输出：

```text
GT:        (2.500, 1.500)
Estimated: (2.532, 1.472)
Error:     0.043 m
```

V1 初始目标：

```text
position error < 5~10 cm
```

先不用追毫米级。

---

# 23. AprilTag 朝向验证

必须专门验证朝向。

将车依次人工摆成：

```text
0°
90°
180°
270°
```

检查 `theta_world` 方向和符号是否正确。

避免：

```text
上下反了
左右反了
+θ/-θ 反了
Tag 边定义错误
```

这是后续小车闭环控制前必须完成的测试。

---

# 24. 开发顺序

Agent 必须严格按以下顺序开发。

## Step 1 — Camera

完成：

```text
list_cameras.py
preview_camera.py
```

确保 Camo Camera 能稳定读帧。

## Step 2 — AprilTag

完成：

```text
test_apriltag.py
```

只输出：

```text
ID
center pixel
corners
image heading
```

先不要加入球检测。

## Step 3 — Field Calibration

完成：

```text
calibrate_field.py
```

人工点击四角并保存 Homography。

## Step 4 — AprilTag World Pose

完成：

```text
image center
↓
world x,y

image heading
↓
world theta
```

输出：

```text
Robot (x,y,theta)
```

## Step 5 — HSV Ball Detection

完成：

```text
tune_ball_hsv.py
test_ball_detection.py
```

得到所有 `ball centers (u,v)`。

## Step 6 — Ball World Coordinates

通过 Homography：

```text
ball center pixel
↓
world xy
```

## Step 7 — Integration

完成：

```text
run_global_tracking.py
```

同时显示：

```text
Robot pose
Balls
Field
FPS
```

## Step 8 — Calibration Verification

完成位置误差和朝向测试。

---

# 25. V1 暂时不要做的事情

禁止 scope creep。

当前不要实现：

```text
YOLO
DeepSORT
SLAM
EKF
IMU
Wheel odometry
Robot control
PID
TCP robot communication
RGBD
Mechanical arm
Path planning
ROS / ROS2
Gazebo
```

这些属于下一阶段。

---

# 26. 对 Agent 的开发要求

1. 先检查当前仓库已有内容，不要直接覆盖；
2. 代码必须可以在 Windows 运行；
3. 依赖尽量少；
4. Python 版本建议 3.10~3.12；
5. 不要引入 ROS；
6. 不要引入 GPU 依赖；
7. 所有路径使用 `pathlib`；
8. 不要硬编码 Windows 绝对路径；
9. 配置进入 YAML；
10. 坐标和单位必须明确；
11. 不允许把图像 angle 直接当 world angle；
12. 不允许因为自动场地线检测失败而阻塞项目；
13. 先实现人工四点 Homography；
14. 每完成一个 Step 都提供运行命令；
15. 如果 Camo 的 camera index 不确定，必须通过 `list_cameras.py` 查找；
16. 不要写大量无意义测试；
17. 核心数学模块可以写少量单元测试；
18. 每一个模块都必须能单独运行 debug；
19. 出错时打印清晰错误信息；
20. 最终写一份简洁的 README 使用说明。

---

# 27. Agent 完成后需要汇报

最终不要只回复 `Done`。

必须报告：

## Files Created

列出新增文件。

## How to Run

从创建环境到运行 global tracker，逐条给命令。

## Manual Setup

说明：

```text
Camo 设置
camera index
AprilTag family
场地尺寸
四角点击顺序
HSV 调参
```

## Known Limitations

例如：

```text
光照变化
反光
iPhone 自动曝光
AprilTag 过小
场地边缘畸变
```

## Next Step

下一阶段应为：

```text
Robot Pose
→ target pose
→ visual closed-loop goto_pose()
```

但当前不要实现。

---

# 28. V1 Definition of Done

满足以下条件才算完成：

- [ ] Windows OpenCV 可读取 Camo Camera
- [ ] 稳定检测车顶 AprilTag
- [ ] 正确输出 AprilTag 世界 `(x,y)`
- [ ] 正确输出小车朝向 `theta`
- [ ] 可检测场地中多个网球
- [ ] 输出每个网球世界 `(x,y)`
- [ ] 支持手动四点场地 Homography 标定
- [ ] 标定参数可以持久化
- [ ] 实时画面带 overlay
- [ ] 有独立 bird's-eye world view
- [ ] 支持保存 debug frame
- [ ] 位置误差可以量化验证
- [ ] 代码模块化且可扩展

---

# 29. 最终 V1 输出接口

后续 TennisBot 所有模块都应该只依赖这个标准输出，而不是直接读 OpenCV 图像。

目标接口：

```python
world_state = tracker.get_world_state()

print(world_state.robot)
print(world_state.balls)
```

结构：

```python
WorldState(
    robot=RobotPose(
        x=...,
        y=...,
        theta=...
    ),
    balls=[
        BallWorld(x=..., y=...),
        BallWorld(x=..., y=...),
    ]
)
```

这将作为下一阶段：

```text
路径规划
+
小车闭环导航
```

的统一输入。

---

# 30. 一句话项目目标

> 使用 iPhone 16 Pro Max + Camo Camera + Windows OpenCV，在固定实验场地中实时获取小车 AprilTag 的全局二维位姿 `(x,y,theta)` 和所有网球的世界二维坐标 `(x,y)`，构建 TennisBot 后续导航和多球规划所需的第一版全局视觉系统。
