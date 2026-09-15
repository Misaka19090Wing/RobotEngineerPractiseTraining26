# 桥架距离测量

`distance_measure.py` 为桥架测量机器人提供实时几何测量：机器人行驶过程中，
持续给出**前方障碍距离、左右壁距、桥架净宽、累计里程与桥架长度**，并以四种
方式输出（RViz 叠加、ROS2 话题、命令行、JSON/CSV 报告）。

---

## 1. 测量定义

| 测量项 | 话题 | 定义 |
|---|---|---|
| `forward` | `/distance/forward` | 机器人**前方车道内**（`|body_y| ≤ forward_lane_half_width`，`body_x > 0`）最近障碍的距离 |
| `left` | `/distance/left` | `+90°` 扇区（`±side_fov_deg/2`）内最近壁面距离 |
| `right` | `/distance/right` | `−90°` 扇区（`±side_fov_deg/2`）内最近壁面距离 |
| `width` | `/distance/width` | `left + right`，即桥架净宽 |
| `traveled` | `/distance/traveled` | `/odom` 路径积分里程 |
| `tray_length` | `/distance/tray_length` | 点云沿主轴（2D PCA）跨度；无地图时回退为 `traveled` |

所有距离以雷达坐标系 **`radar_link`** 为基准量测，与
`waypoint_navigator._scan_avoidance` 的避障基准保持一致。

### 1.1 为什么前方障碍用「车道门限」而非固定角度扇区

桥架仅约 **0.30 m** 宽，而 `radar_link` 距左壁仅 0.134 m。若用常见的固定角度
扇区（例如 ±15°）：

- 边缘射线（15°）打到左壁的距离 ≈ `0.134 / sin(15°) ≈ 0.52 m`
- 结果会把**侧壁**误判成前方 0.52 m 处的障碍，而不是真正的正前方通道

因此实现改为：把每个命中点投到车体坐标系，只保留落在地板车道内
（`|y| ≤ forward_lane_half_width`，默认 0.10 m，略宽于机器人半宽 0.074 m）且在前方
（`x > 0`）的点，取其最近距离。侧壁点的 `|y|` 为 0.134 / 0.166 m，均被正确排除。

### 1.2 桥架长度如何得到

对累积点云（`/pointcloud_map`）的 XY 投影做 2D PCA：

```
axis  = 0.5 * atan2(2*sxy, sxx - syy)      # 主轴方向
proj  = x*cos(axis) + y*sin(axis)          # 投影
length = max(proj) - min(proj)             # 桥架长度
```

本赛道主廊道沿 X 约 22 m，T 字路口横向臂仅约 3.3 m，因此主轴稳定落在廊道方向。
投影最小/最大点同时作为桥架起止点，以黄色球体标记发布。

---

## 2. 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `report_dir` | `~/distance_reports` | JSON/CSV 保存目录 |
| `publish_rate_hz` | `10.0` | 话题与标记发布频率 |
| `map_rate_hz` | `0.5` | 点云主轴测量频率（纯 Python 解析，降频以省 CPU） |
| `sample_rate_hz` | `2.0` | CSV 采样记录频率 |
| `forward_lane_half_width` | `0.10` | 前方车道半宽 [m] |
| `side_fov_deg` | `30.0` | 左/右壁扇区总张角 |
| `max_map_points` | `40000` | 点云抽稀上限 |
| `max_samples` | `20000` | 内存中保留的采样条数 |
| `min_travel_step` | `1e-4` | 里程积分最小步长，抑制静止漂移 |
| `marker_z_offset` | `0.45` | RViz 文字标记抬高量 |
| `near_distance` | `0.25` | 射线变红阈值 |
| `warn_distance` | `0.60` | 射线变橙阈值 |

可在 launch 时覆盖 `report_dir`：

```bash
ros2 launch autoVehicle gazebo.launch.py report_dir:=/tmp/distance_reports
```

---

## 3. 输出

### 3.1 话题

```
/distance/forward       std_msgs/Float32
/distance/left          std_msgs/Float32
/distance/right         std_msgs/Float32
/distance/width         std_msgs/Float32
/distance/traveled      std_msgs/Float32
/distance/tray_length   std_msgs/Float32
/distance/status        std_msgs/String     单行摘要
/distance_markers       visualization_msgs/MarkerArray
```

无效测量发布 `NaN`（例如尚无扫描数据）。

### 3.2 服务

```
/distance/status   std_srvs/Trigger   返回单行摘要
/distance/reset    std_srvs/Trigger   里程、统计、采样清零
/distance/save     std_srvs/Trigger   写出 JSON + CSV
```

### 3.3 RViz 叠加

- `distance_text` — 跟随机器人的多行文字（里程 / 前方 / 左 / 右 / 净宽 / 桥架长）
- `distance_rays` — 前/左/右量测射线，按距离着色（近红 / 预警橙 / 安全绿）
- `distance_span` — 左右壁之间的净宽连线
- `distance_ends` — 测得的桥架起止点

`config/pointcloud_mapping.rviz` 已包含 **Distance Markers** 显示器，无需手工添加。

### 3.4 命令行

```bash
ros2 run autoVehicle distance_cli.py show      # 逐项打印
ros2 run autoVehicle distance_cli.py status    # 单行摘要
ros2 run autoVehicle distance_cli.py save      # 保存报告
ros2 run autoVehicle distance_cli.py reset     # 清零
```

### 3.5 报告文件

`save` 生成两份文件（`~/distance_reports/`）：

- `distance_<时间戳>.json` — 汇总统计、桥架长度与来源、主轴角度、起止点、位姿
- `distance_<时间戳>.csv` — 逐采样时序
  （`t,x,y,yaw,forward,left,right,width,traveled,tray_length`）

---

## 4. 实测校核

在 `course_test.sdf` 平直段（起点 `x=0.5`，走廊 `y=±0.15`）实测：

| 测量项 | 实测值 | 理论值 | 结论 |
|---|---|---|---|
| 净宽 `width` | **0.3000 m** | 0.15 + 0.15 = 0.300 m | 精确吻合 |
| 左壁 `left` | **0.1343 m** | 0.15 − 0.015705 = 0.134295 m | 精确吻合 |
| 右壁 `right` | **0.1657 m** | 0.15 + 0.015705 = 0.165705 m | 精确吻合 |
| 前方 `forward` | **22.32 m** | 22.88 − 0.563 = 22.317 m | 精确吻合 |
| 桥架长度 `tray_length` | **22.88 m** | 廊道 x ∈ [0, 22.88] | 精确吻合 |
| 里程 `traveled` | 随行驶线性累加 | 0.2 m/s × 时间 | 正常 |

复现方式：

```bash
ros2 launch autoVehicle gazebo.launch.py          # 终端 1
ros2 run autoVehicle distance_cli.py show         # 终端 2
ros2 topic pub --rate 10 --times 50 /cmd_vel \
  geometry_msgs/msg/Twist '{linear: {x: 0.2}}'    # 前进约 5 s
ros2 run autoVehicle distance_cli.py show         # 里程应为数米
ros2 run autoVehicle distance_cli.py save
```

---

## 5. 相关修复

### 5.1 地面扫描生成幻影点，导致桥架长度偏大

**现象**：`tray_length` 报 **27.3 m**，但赛道只有 22.88 m；JSON 中
`tray_start = [-3.94, 0.0157]`，起点落在桥架之外。

**原因**：`synthetic_lidar.py` 地面扫描的边界判断只有上界：

```python
if wx < CORR_END:          # 缺少下界
    if abs(wy) > HW: ok = False
```

后向射线因此在地板范围之外（`wx < 0`）继续生成点，最多延伸到机器人后方 5 m。
这些幻影点累积进 `/pointcloud_map`，把 PCA 主轴跨度撑大到 27.3 m。

**修复**：补上下界，使地面点限制在 `0 ≤ wx < CORR_END`：

```python
if 0.0 <= wx < CORR_END:
    if abs(wy) > HW: ok = False
```

修复后 `tray_length` 为 **22.88 m**，`tray_start = [0.0055, 0.15]`，与赛道吻合；
点云地图也不再包含桥架之外的悬空地面点。

### 5.2 报告目录不可写不再导致节点崩溃

初版在 `__init__` 中直接 `os.makedirs(report_dir)`，目录不可写时整个节点异常退出
（测距功能一并失效）。现改为捕获 `OSError`、打印告警并仅禁用 `/distance/save`，
测量本身继续工作。

---

## 6. 文件清单

| 文件 | 作用 |
|---|---|
| `scripts/distance_measure.py` | 测距主节点 |
| `scripts/distance_cli.py` | 命令行工具 |
| `config/pointcloud_mapping.rviz` | 新增 Distance Markers 显示器 |
| `launch/gazebo.launch.py` | 随仿真启动，可传 `report_dir` |
| `CMakeLists.txt` | 注册两个可执行文件 |
