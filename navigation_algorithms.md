# 项目两种导航算法详解

本项目提供两套导航方式：

1. 自定义航点导航（`waypoint_navigator.py`）
2. Nav2 预建图导航（`nav_map.launch.py`）

两者底层算法不同，本文分别详细解释。

---

## 1. 自定义航点导航

### 1.1 整体流程

```text
用户设置航点
      ↓
waypoint_navigator 接收航点
      ↓
A* 全局路径规划（基于 course_map）
      ↓
生成规划中间点
      ↓
Lookahead 路径跟踪
      ↓
差速运动学 → /cmd_vel
      ↓
/scan 实时避障
```

### 1.2 A* 全局路径规划

#### 是什么

A* 是一种基于栅格地图的最短路径搜索算法，用于从机器人当前位置规划到目标航点的可行路径。

#### 地图来源

- 使用 `maps/course_map.yaml`
- 分辨率：`0.02 m/cell`
- 尺寸：`1186 × 186`
- free 区域用 `254` 表示
- 障碍物用 `0` 表示

#### 预处理

1. **障碍膨胀**
   - 对障碍物向外膨胀 `plan_inflation_cells` 格。
   - 默认 `3` 格，即约 `0.06m`。
   - 防止路径太贴近墙壁。

2. **软代价场**
   - 对障碍物附近生成连续代价。
   - 离墙越近，路径代价越高。
   - 让 A* 偏向走廊/缺口中心，而不是贴着墙走。

#### 搜索过程

A* 使用：

```text
f(n) = g(n) + h(n)
```

其中：

- `g(n)`：从起点到当前节点的实际代价
- `h(n)`：当前节点到目标节点的启发式估计
- 启发式使用欧氏距离：

```text
h(n) = sqrt((x_goal - x_n)^2 + (y_goal - y_n)^2)
```

#### 搜索规则

- 使用 8 邻域：
  - 上下左右代价 `1`
  - 对角代价 `sqrt(2)`
- 只允许进入 free 单元
- 使用优先队列 `heapq` 取最小 `f(n)`
- 记录 `came_from`，找到目标后回溯路径

#### 软代价叠加

实际代价：

```text
g(n+1) = g(n) + step_cost + (soft_cost - 1) × 0.5
```

软代价让 A* 自动选择距离墙壁更远的路径。

#### 路径后处理

- 将栅格路径转换回世界坐标
- 按 `plan_path_spacing`（默认 `0.25m`）稀疏化
- 生成中间航点并插入航点队列

### 1.3 Lookahead 路径跟踪

#### 为什么不用“逐点追点”

如果直接瞄准当前最近中间点：

- 每个中间点都会触发减速、停车、转向
- 小车容易冲过点，又转回来

因此改为 lookahead：

```text
path_lookahead = 0.6m
```

控制目标不是最近点，而是路径前方约 `0.6m` 的点。

#### 经过中间点策略

当满足：

```text
distance(当前位置, 当前点) < goal_tolerance
```

或：

```text
distance(当前位置, 下一点) < distance(当前位置, 当前点)
```

就认为已经“经过”当前点，切换到下一个点，不减速、不停车、不对齐朝向。

### 1.4 差速运动学控制

输入：

- 当前位姿 `(x, y, yaw)`
- 目标点 `(x_goal, y_goal)`

计算：

```text
dx = x_goal - x
dy = y_goal - y

distance = sqrt(dx² + dy²)
desired_yaw = atan2(dy, dx)
yaw_error = normalize(desired_yaw - yaw)
```

期望线速度：

```text
if distance >= approach_distance:
    linear = max_linear_speed
else:
    linear = max(min_linear_speed,
                 max_linear_speed × distance / approach_distance)
```

期望角速度：

```text
angular = clamp(yaw_error × angular_gain,
                -max_angular_speed,
                max_angular_speed)
```

### 1.5 加速度平滑

为了避免忽快忽慢，使用加速度限制：

```text
dt = 1 / control_rate

max_lin_step = linear_accel_limit × dt
max_ang_step = angular_accel_limit × dt

cmd_linear = clamp(desired_linear,
                   cmd_linear - max_lin_step,
                   cmd_linear + max_lin_step)

cmd_angular = clamp(desired_angular,
                    cmd_angular - max_ang_step,
                    cmd_angular + max_ang_step)
```

### 1.6 实时避障

避障使用 `/scan`：

- 将激光点转换到机器人坐标系
- 只保留前方区域
- 只有正前方窄角度内的近距离障碍才停车：

```text
obstacle_stop_angle = 0.5 rad
obstacle_stop_distance = 0.08 m
obstacle_slow_distance = 0.30 m
```

- 侧墙只触发左右避让：

```text
left_risk, right_risk
avoid = avoid_gain × (right_risk - left_risk) / total_risk
```

### 1.7 自定义航点算法总结

```text
A* 全局规划
+ Lookahead 路径跟踪
+ P 控制（角度 + 距离）
+ 加速度平滑
+ 激光侧向避障
```

---

## 2. Nav2 预建图导航

### 2.1 整体流程

```text
RViz 2D Goal Pose
      ↓
Nav2 NavigateToPose 行为树
      ↓
全局规划器 NavfnPlanner
      ↓
全局路径 /plan
      ↓
局部控制器 DWB
      ↓
/cmd_vel
      ↓
Gazebo 运动
```

### 2.2 全局规划：NavfnPlanner

#### 是什么

NavfnPlanner 是 Nav2 默认的全局规划器之一，属于基于栅格代价地图的波前/最短路规划算法。

#### 与 A* 的区别

| 项目 | A* | NavfnPlanner |
|---|---|---|
| 启发式 | 使用启发式加速 | 波前扩散，类似 Dijkstra |
| 搜索方向 | 有目标导向 | 从起点向外扩散 |
| 适用性 | 适合固定小地图 | 适合 Nav2 动态代价地图 |
| 动态障碍 | 不直接支持 | 可结合 costmap |

当前配置：

```yaml
GridBased:
  plugin: nav2_navfn_planner::NavfnPlanner
  use_astar: false
  allow_unknown: true
```

`use_astar: false` 表示使用 NavFn 波前搜索，而不是 A*。

#### 代价地图输入

全局代价地图只使用：

```yaml
plugins:
- static_layer
- inflation_layer
```

`static_layer` 提供 `course_map` 静态地图，`inflation_layer` 负责障碍膨胀。

### 2.3 局部控制：Regulated Pure Pursuit

Nav2 当前使用 `nav2_regulated_pure_pursuit_controller`，即 **RPP（Regulated Pure Pursuit）**。

#### 核心思想

RPP 是纯追踪算法的改进版，在小车前方选择一个 lookahead 目标点，然后控制小车朝该点前进。

相比 DWB，RPP 更适合窄走廊和弯道：

- 不会因为局部代价地图找不到轨迹而停住
- 更贴合全局路径
- 通过曲率自动调节速度
- 通过代价地图自动降低危险区域速度

#### 当前 RPP 参数

```text
desired_linear_vel: 0.5
lookahead_dist: 0.6
min_lookahead_dist: 0.3
max_lookahead_dist: 0.9
rotate_to_heading_angular_vel: 1.5
max_angular_accel: 3.0
use_rotate_to_heading: true
use_collision_detection: false
use_regulated_linear_velocity_scaling: true
use_cost_regulated_linear_velocity_scaling: false
```

#### 速度调节机制

RPP 当前关闭了曲率/代价自适应减速，在直道和走廊中保持匀速；靠近最终目标时仍会按 `approach_velocity_scaling_dist` 减速。

#### 碰撞检测

```text
use_collision_detection: true
max_allowed_time_to_collision_up_to_carrot: 1.0
```

RPP 会预测小车到目标点之间的碰撞风险，提前减速而不是直接卡住。

#### 局部代价地图

```yaml
local_costmap:
  robot_radius: 0.07
  inflation_radius: 0.10
  obstacle_layer:
    observation_sources: scan
```

`obstacle_layer` 使用 `/scan` 实时更新局部障碍。

### 2.4 行为恢复

当控制器长时间无法取得进展时，Nav2 会执行恢复行为：

```text
spin        // 原地旋转
backup      // 后退
wait        // 等待
```

行为树负责决定什么时候执行恢复行为，以及恢复失败后的最终结果。

### 2.5 Nav2 生命周期

Nav2 节点使用生命周期管理：

```text
unconfigured → inactive → active
```

`lifecycle_manager_navigation` 负责自动配置并激活：

```text
map_server
planner_server
controller_server
behavior_server
bt_navigator
waypoint_follower
```

### 2.6 Nav2 算法总结

```text
NavfnPlanner 全局路径规划
+ Regulated Pure Pursuit 局部路径跟踪
+ 多 critic 轨迹评分
+ BehaviorTree 导航流程控制
+ 行为恢复机制
+ 生命周期管理
```

---

## 3. 两套导航对比

| 项目 | 自定义航点导航 | Nav2 导航 |
|---|---|---|
| 全局规划 | 自实现 A* | NavfnPlanner |
| 局部规划 | Lookahead + P 控制 | Regulated Pure Pursuit |
| 地图 | course_map | course_map |
| 避障 | `/scan` 简单避让 | costmap + RPP 碰撞检测 |
| 路径显示 | `/waypoint_path` | `/plan` `/local_plan` |
| 速度控制 | 参数控制 + 加速度限制 | Nav2 参数 + `/speed_limit` |
| 恢复机制 | 无 | spin / backup / wait |
| 适用场景 | 轻量、可控 | 完整、复杂 |

---

## 4. 关键文件

| 文件 | 作用 |
|---|---|
| `scripts/waypoint_navigator.py` | 自定义航点导航 + A* |
| `scripts/generate_course_map.py` | 生成 course_map |
| `config/nav2_params.yaml` | Nav2 参数 |
| `launch/nav_map.launch.py` | Nav2 启动 |
| `config/pointcloud_mapping.rviz` | RViz 路径显示 |
