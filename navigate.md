# 机器人航点导航说明

本文档整理 3D 点云建图、手动航点设置、Nav2 预建图导航及上坡排障的最终方案。

## 1. 功能概览

项目目前提供两套可用的导航方式：

1. **自定义航点导航**：加载 PCD 地图，通过 RViz 2D Goal Pose 或命令行添加航点，按队列输出 `/cmd_vel`。
2. **Nav2 预建图导航**：加载 `course_map.yaml`，使用 Nav2 的 planner/controller 完成路径规划与跟踪。

两套方式可以并存，但不要同时启动任务，避免多个节点同时发布 `/cmd_vel`。

## 2. 相关文件

| 文件 | 作用 |
|---|---|
| `robotProj_ws/autoVehicle/scripts/waypoint_navigator.py` | 自定义航点节点，加载 PCD、维护航点队列、发布 `/cmd_vel` |
| `robotProj_ws/autoVehicle/scripts/waypoint_cli.py` | 命令行添加/启停/保存航点 |
| `robotProj_ws/autoVehicle/scripts/generate_course_map.py` | 按赛道几何生成干净的 Nav2 栅格地图 |
| `robotProj_ws/autoVehicle/scripts/synthetic_lidar.py` | 合成 LiDAR + 点云建图；已优化坡面扫描 |
| `robotProj_ws/autoVehicle/launch/nav_map.launch.py` | Nav2 预建图导航启动文件 |
| `robotProj_ws/autoVehicle/launch/nav_slam.launch.py` | slam_toolbox 建图入口 |
| `robotProj_ws/autoVehicle/config/nav2_params.yaml` | Nav2 窄走廊参数 |
| `robotProj_ws/autoVehicle/maps/course_map.pgm/yaml` | 当前 Nav2 默认地图 |

## 3. 启动方式

### 3.1 自定义航点导航

先启动仿真：

```bash
cd robotProj_ws && source install/setup.bash
ros2 launch autoVehicle gazebo.launch.py
```


仅导航时可以用 `build_map:=false` 关闭点云累积，降低硬件负载：

```bash
ros2 launch autoVehicle gazebo.launch.py build_map:=false
```

RViz 中会显示 PCD 地图，使用 **2D Goal Pose** 在地图上依次点选航点。也可以使用命令行：

```bash
ros2 run autoVehicle waypoint_cli.py add 3.0 0.0 --yaw 0
ros2 run autoVehicle waypoint_cli.py add 22.73 1.2 --yaw 90
ros2 run autoVehicle waypoint_cli.py start
```

常用服务：

```bash
ros2 service call /waypoint/start std_srvs/srv/Trigger
ros2 service call /waypoint/stop std_srvs/srv/Trigger
ros2 service call /waypoint/clear std_srvs/srv/Trigger
ros2 service call /waypoint/status std_srvs/srv/Trigger
ros2 service call /waypoint/reload_map std_srvs/srv/Trigger
```

### 3.2 Nav2 预建图导航

先启动 Gazebo 仿真，再在另一个终端启动 Nav2：

```bash
ros2 launch autoVehicle nav_map.launch.py
```

`nav_map.launch.py` 默认使用：

```text
maps/course_map.yaml
```

也可以指定其他地图：

```bash
ros2 launch autoVehicle nav_map.launch.py map:=/path/to/map.yaml
```

## 3.3 自定义航点路径规划

自定义航点默认启用基于 `course_map` 的 A* 路径规划：

- 添加航点后，会先从机器人当前位置到目标航点规划一条沿走廊的路径。
- A* 带软代价场，会自动偏向走廊/缺口中心，避免贴墙路径。
- 规划出的中间点会自动插入航点队列，避免小车直线冲向墙壁。
- 如果 `course_map` 加载失败，会回退到原来的直线航点跟踪模式。

相关参数：

```text
plan_enabled           是否启用 A*，默认 true
plan_map_file          规划地图，默认 course_map.yaml
plan_inflation_cells   障碍膨胀格数，默认 3（0.06m）
plan_path_spacing      规划路径点间距，默认 0.25m
path_lookahead        路径前瞻距离，默认 0.6m
```

修改规划参数：

```bash
ros2 run autoVehicle waypoint_navigator.py --ros-args   -p plan_enabled:=true   -p plan_path_spacing:=0.25
```

## 3.4 自定义航点速度控制

速度控制已改为“巡航 + 接近减速 + 加速度限制”，不会再因为 `distance × 1.2` 在整段路上忽快忽慢。

相关参数：

```text
max_linear_speed    巡航最高线速度，单位 m/s
max_angular_speed   最高角速度，单位 rad/s
min_linear_speed    接近目标时的最低线速度，单位 m/s
approach_distance   开始减速的距离，单位 m
linear_accel_limit  线加速度限制，单位 m/s²
angular_accel_limit 角加速度限制，单位 rad/s²
```

默认值：

```text
max_linear_speed: 0.8
min_linear_speed: 0.08
approach_distance: 0.6
linear_accel_limit: 0.6
angular_accel_limit: 2.0
```

启动前指定：

```bash
ros2 run autoVehicle waypoint_navigator.py --ros-args   -p max_linear_speed:=0.8   -p min_linear_speed:=0.1   -p approach_distance:=0.8   -p linear_accel_limit:=0.8
```

运行中动态调整：

```bash
ros2 param set /waypoint_navigator max_linear_speed 0.8
ros2 param set /waypoint_navigator approach_distance 0.8
ros2 param set /waypoint_navigator linear_accel_limit 0.8
```

如果通过 `gazebo.launch.py` 启动，直接使用上面的 `ros2 param set` 即可。

避障参数：

```text
obstacle_stop_distance  正前方障碍停车距离，默认 0.08m
obstacle_slow_distance  正前方障碍开始减速距离，默认 0.30m
obstacle_stop_angle     正前方停车判定角，默认 0.5rad（约 28°）
avoid_gain              侧向避让强度，默认 1.2
```

只有正前方窄角度内距离过近才停车；侧墙会触发左右避让，不再直接把小车逼停。

```bash
ros2 param set /waypoint_navigator obstacle_stop_distance 0.06
ros2 param set /waypoint_navigator obstacle_stop_angle 0.6
ros2 param set /waypoint_navigator avoid_gain 1.5
```

## 3.5 Nav2 2D Goal Pose 路径显示与速度控制

RViz 配置已加入 Nav2 路径显示：

- `/plan`：Nav2 全局规划路径
- `/local_plan`：Nav2 局部轨迹
- `/received_global_plan`：Nav2 接收到的全局路径

设置 2D Goal Pose 后，RViz 会自动显示这些路径。

### Nav2 速度控制

Nav2 运行中可动态限制速度：

```bash
# 将最高速度限制为 0.5 m/s
ros2 topic pub /speed_limit nav2_msgs/msg/SpeedLimit "{
  percentage: false,
  speed_limit: 0.5
}" --qos-durability transient_local --qos-reliability reliable -r 1
```

也可以修改 `nav2_params.yaml` 中的控制器参数，例如：

```yaml
FollowPath:
  max_vel_x: 0.5
```

修改后需要重启 Nav2。

当前 Nav2 默认参数：

```text
controller: nav2_regulated_pure_pursuit_controller
desired_linear_vel: 0.5
lookahead_dist: 0.6
min_lookahead_dist: 0.3
max_lookahead_dist: 0.9
rotate_to_heading_angular_vel: 1.5
use_collision_detection: false
use_regulated_linear_velocity_scaling: true
use_cost_regulated_linear_velocity_scaling: false
use_rotate_to_heading: false
robot_radius: 0.07
inflation_radius: 0.10
```

转弯减速参数：

```text
regulated_linear_scaling_min_radius: 0.9   # 转弯半径小于该值开始减速
regulated_linear_scaling_min_speed: 0.1    # 转弯时的最低线速度
```

- 想转弯更慢：调大 `regulated_linear_scaling_min_radius`
- 想转弯最低速度更低：调小 `regulated_linear_scaling_min_speed`

局部控制器已从 DWB 改为 Regulated Pure Pursuit，更适合窄走廊和 Seg3→Seg4 这种拐弯场景，能明显减少贴墙、右墙停住再重规划的现象。


如果想使用“航点队列 + 参数控制 + 路径显示”的完整自定义方案，可以直接使用 `waypoint_navigator.py`，它已支持 A* 路径规划、路径显示和速度参数控制。

## 4. 已修复问题

### 4.1 旧 PGM 地图把走廊中心标记为障碍

现象：

```text
GridBased plugin failed to plan from (0.50, 0.00) to (3.10, -0.00):
"Failed to create plan with tolerance of: 0.100000"
```

原因：

旧 `map_20260807_232605.pgm` 将走廊中心整条标记为 occupied，机器人起点和目标点都落在障碍格内。

解决：

新增 `generate_course_map.py`，按赛道几何生成干净的 `course_map.pgm/yaml`。

```bash
ros2 run autoVehicle generate_course_map.py
```

正常日志应显示：

```text
StaticLayer: Resizing costmap to 1186 X 186 at 0.020000 m/pix
```

### 4.2 窄走廊内 DWB 找不到合法轨迹

现象：

```text
Could not find a legal trajectory: No valid trajectories out of 20!
Collision Ahead - Exiting Spin
```

原因：

`robot_radius`、`inflation_radius` 和 DWB `BaseObstacle` 权重对 0.3 m 走廊过保守。

解决：

在 `nav2_params.yaml` 中调整为：

```text
robot_radius: 0.08
inflation_radius: 0.12
cost_scaling_factor: 5.0
BaseObstacle.scale: 0.02
```

### 4.3 45° 坡道入口无法规划

现象：

```text
Begin navigating from current location (19.90, 0.00) to (21.41, 0.02)
GridBased plugin failed to plan from (19.90, 0.00) to (21.41, 0.02)
```

原因：

`synthetic_lidar.py` 把 45° 坡面交点也写进了 `/scan`，Nav2 的全局 `obstacle_layer` 将坡面入口标记为障碍。

解决：

1. `synthetic_lidar.py` 的 `/scan` 不再包含坡面交点，坡面仍通过地面点阵进入 3D 点云：

```python
r = raycast(lx, ly, lz, world_ang, include_ramp=False)
```

2. 全局代价地图只使用静态 `course_map`，实时 `/scan` 仅用于局部避障：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins:
      - static_layer
      - inflation_layer
```

验证结果：

```text
raycast(x=19.9, 坡面启用):  0.21 m
raycast(x=19.9, 坡面禁用):  2.98 m
侧墙检测:                  0.15 m
```

## 5. 验证记录

- `colcon build --symlink-install --packages-select autoVehicle` 通过。
- PCD 加载正常：`map_20260807_232605.pcd` 可加载并发布为 `/waypoint_map`。
- 自定义航点：完整 Gazebo 中通过服务添加航点后，机器人从 `x=0.50` 移动到 `x≈1.00`，`/cmd_vel` 正常输出。
- Nav2 规划：`ComputePathToPose` 从 `(0.5,0)` 到 `(10.0,0)` 成功返回路径。
- 上坡规划：模拟坡前位置 `(19.9,0.05)` 到 `(21.41,0)`，路径规划成功。
- 窄走廊控制：`navigate_to_pose` 可持续输出 `Passing new path to controller`，无 `No valid trajectories`。

## 6. 建议

- 上坡目标建议设置中间航点或直接使用 Nav2 2D Goal Pose，避免一次跨越太长距离。
- 如果机器人测试后停在坡上，重启 Gazebo 会回到起点 `(0.5, 0)`。
- 旧 PGM 文件仍保留，但 Nav2 默认不再使用。
- 后续如需真实环境导航，可将 `course_map` 替换为真实 SLAM 栅格地图，并保持 `nav2_params.yaml` 的窄走廊参数。
- 赛道改为 20m 后，旧 PCD 只覆盖旧赛道范围；使用自定义航点前需要重新建图：

```bash
ros2 launch autoVehicle gazebo.launch.py
# 另一个终端：遥控或通过 waypoint_cli 沿新赛道走完整条路
ros2 service call /save_map std_srvs/srv/Trigger
ros2 service call /waypoint/reload_map std_srvs/srv/Trigger
```

Nav2 的 `course_map` 已按新赛道几何重新生成，不需要依赖 PCD。
