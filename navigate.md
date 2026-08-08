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

RViz 中会显示 PCD 地图，使用 **2D Goal Pose** 在地图上依次点选航点。也可以使用命令行：

```bash
ros2 run autoVehicle waypoint_cli.py add 3.0 0.0 --yaw 0
ros2 run autoVehicle waypoint_cli.py add 5.94 1.2 --yaw 90
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
StaticLayer: Resizing costmap to 341 X 186 at 0.020000 m/pix
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
Begin navigating from current location (2.90, 0.00) to (4.41, 0.02)
GridBased plugin failed to plan from (2.90, 0.00) to (4.41, 0.02)
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
raycast(x=2.9, 坡面启用):  0.41 m
raycast(x=2.9, 坡面禁用):  3.18 m
侧墙检测:                  0.15 m
```

## 5. 验证记录

- `colcon build --symlink-install --packages-select autoVehicle` 通过。
- PCD 加载正常：`map_20260807_232605.pcd` 可加载并发布为 `/waypoint_map`。
- 自定义航点：完整 Gazebo 中通过服务添加航点后，机器人从 `x=0.50` 移动到 `x≈1.00`，`/cmd_vel` 正常输出。
- Nav2 规划：`ComputePathToPose` 从 `(0.5,0)` 到 `(3.1,0)` 成功返回路径。
- 上坡规划：模拟坡上位置 `(3.3,0.05)` 到 `(4.41,0)`，路径规划成功。
- 窄走廊控制：`navigate_to_pose` 可持续输出 `Passing new path to controller`，无 `No valid trajectories`。

## 6. 建议

- 上坡目标建议设置中间航点或直接使用 Nav2 2D Goal Pose，避免一次跨越太长距离。
- 如果机器人测试后停在坡上，重启 Gazebo 会回到起点 `(0.5, 0)`。
- 旧 PGM 文件仍保留，但 Nav2 默认不再使用。
- 后续如需真实环境导航，可将 `course_map` 替换为真实 SLAM 栅格地图，并保持 `nav2_params.yaml` 的窄走廊参数。
