# 桥架测量机器人设计与调试报告

## 0. 报告说明

本报告综合以下项目文档整理：

- `README.md`：项目总体设计与指标
- `pointCloud.md`：3D 点云建图架构与 Bug 修复历程
- `navigate.md`：航点导航、Nav2 配置与排障
- `testlog.md`：Gazebo 仿真测试日志
- `design_debug_report.md`：当前报告主体

## 1. 项目概述

本项目面向狭窄金属桥架环境的测量作业，设计一款小型桥架测量机器人。机器人采用四轮独立直驱差速转向布局，通过电磁铁吸附在桥架表面行驶，可在平直段、45°斜坡和 T 字路口等场景完成移动、建图和自主导航任务。

当前阶段主要完成 ROS2 仿真环境下的：

- 四轮差速运动控制
- 3D 点云建图与 PCD 地图保存
- 基于 PCD 地图的手动航点导航
- Nav2 预建图导航
- 20m 长直道、45° 坡道和 T 字路口的完整仿真赛道

## 2. 设计要求与技术指标

| 项目 | 指标 |
|---|---|
| 整车尺寸 | ≤180 mm × 180 mm × 95 mm |
| 整车重量 | ≤2 kg |
| 底盘尺寸 | 165 mm × 140 mm |
| 车轮直径 | 30 mm |
| 轮距 | 0.148 m |
| 轴距 | 0.14 m |
| 10 m 内累计误差目标 | ≤0.1 m |
| 最大爬坡角度 | 45° |
| 走廊宽度 | 0.3 m |
| 转弯方式 | 差速原地转向 |

## 3. 总体设计

### 3.1 机械结构

机器人底盘采用碳纤维板与 3D 打印结构件，四轮由 DJI M2006 无刷减速电机直接驱动。车轮采用 30 mm 硅胶轮，底部安装 4 个 KK-P20/15 电磁铁，通过吸附力保证 45° 坡道上的稳定行驶。

### 3.2 电气系统

- 电池：6S LiPo 1300mAh，22.2V
- 电机驱动：SimpleFOC Mini ×4
- 主控：树莓派 4B
- 底层控制板：STM32F407VET6
- 感知：LD06 激光雷达 + MPU6050 IMU
- 供电：24V 直供电机驱动，降压后为树莓派、IMU 和控制板供电

### 3.3 软件系统

软件基于 ROS2 Jazzy、Gazebo Harmonic 和 RViz2，核心节点包括：

| 节点 | 功能 |
|---|---|
| `diff_drive_controller` | 订阅 `/cmd_vel`，输出四轮关节速度 |
| `pose_tf_broadcaster` | 发布 `odom → base_footprint` 动态 TF |
| `synthetic_lidar` | 合成 LiDAR 扫描、点云累积、地图发布和 PCD 保存 |
| `waypoint_navigator` | PCD 加载、航点队列、差速巡航和激光避障 |
| `waypoint_cli` | 命令行添加/启停/保存航点 |
| `generate_course_map` | 按赛道几何生成 Nav2 栅格地图 |
| Nav2 节点 | map_server、planner、controller、bt_navigator、waypoint_follower |

### 3.4 数据流与 TF 树

```text
teleop → /cmd_vel → diff_drive_controller → ros_gz_bridge → Gazebo joint
                                                              ↓
                                                     OdometryPublisher
                                                              ↓
                                                     /odom
                                                              ↓
                                                  pose_tf_broadcaster
                                                              ↓
                                                  odom → base_footprint
                                                              ↓
                                          synthetic_lidar / waypoint_navigator / Nav2
```

完整 TF 树：

```text
odom → base_footprint → base_link → (left_front_wheel, right_rear_wheel, radar_link, ...)
```

## 4. 仿真赛道

`course_test.sdf` 的赛道参数如下：

| 区域 | X 范围 (m) | Z 范围 (m) | 备注 |
|---|---|---|---|
| 平直段 Seg1 | 0 ~ 20 | 0 | 20m 长直道 |
| 45° 斜坡 Seg2 | 20 ~ 21.13 | 0 → 1.13 | 坡面 |
| 干扰段 Seg3 | 21.13 ~ 22.58 | 1.13 | 地面含凸起 |
| T 字路口 | 约 22.73 | 1.13 | Y 方向展开 ±1.6m |

## 5. 运动控制调试过程

### 5.1 URDF 导出问题

- SolidWorks 第一次重新导出时，joint origin 均为 `(0,0,0)`。
- 修复方案：保留旧版 joint origin，使用新版 mass/inertia。
- 第二次导出后 joint origin 正确，但存在 git merge 冲突标记。
- 修复方案：手动清理冲突，保留新版 mass/inertia/visual 与旧版 joint 数据。

### 5.2 碰撞几何迭代

| 方案 | 结果 | 结论 |
|---|---|---|
| STL mesh 碰撞 | 前进后退正常，无法转向 | STL 接触面不稳定，小尺寸摩擦不足 |
| 圆柱碰撞 | 前进平滑，转向失败 | 圆柱沿轴方向摩擦力不足 |
| 球体碰撞 | 前进基本正常，转向失败 | 接触面过小，低转速摩擦不足 |
| 方盒碰撞 | 转向有改善但前进抽搐 | 盒子旋转时边角插地 |
| 圆柱 + 小方盒 | 转向仍不可靠 | 接触间歇性 |
| 最终球体 + DART 摩擦 | 转向正常 | 球体旋转不变，各向摩擦均匀 |

### 5.3 摩擦参数调试

- ODE `<mu>` 参数下无法可靠转向。
- DART 引擎需要显式添加 `<primary_friction>` / `<secondary_friction>`。
- 摩擦系数从 1.0 提高到 100.0 后转向稳定。
- 最终配置：DART primary=100，secondary=100。

### 5.4 控制器方案

- Gazebo Harmonic 的 joint 默认不接受外部 `cmd_vel`。
- 移除 DiffDrive 插件后，`ros_gz_bridge` 能通信但轮子不转。
- 为四个轮子 joint 添加 `gz-sim-joint-controller-system` 后，轮子正常响应。
- 最终控制链路：

```text
/cmd_vel → diff_drive_controller.py → ros_gz_bridge → Gazebo joint cmd_vel
                                                              ↓
                                                   JointController 插件
                                                              ↓
                                                         轮子转动
```

## 6. 3D 点云建图

### 6.1 建图方案

由于虚拟机无法稳定运行 Gazebo 的 GPU/URDF LiDAR 传感器，`synthetic_lidar.py` 使用数学射线投射模拟 360 条 2D 激光射线，并生成地面点阵。

### 6.2 扫描参数

| 项目 | 参数 |
|---|---|
| 扫描频率 | 10Hz |
| 墙面射线 | 360 条 |
| 地面点阵 | 36 方位 × 10 距离步 |
| 单帧点数 | 约 400~500 点 |
| 累积速率 | 约 4000~5000 点/秒 |
| 地图发布 | `/pointcloud_map`，每 0.5s |
| 自动保存间隔 | 每 500K 点 |

### 6.3 PCD 地图

PCD 使用 ASCII 格式，字段为：

```text
FIELDS x y z intensity
```

保存位置：

```text
~/pointcloud_maps/map_YYYYMMDD_HHMMSS.pcd
```

### 6.4 建图 Bug 修复清单

1. TF 树方向错误：修正为 `base_footprint → base_link`。
2. 缺少 `joint_state_publisher`：补齐后轮子 TF 正常。
3. Header 类型错误：`rclpy.time.Time().to_msg()` 返回 Time，需构造 Header。
4. `gpu_lidar` 依赖 OGRE2：VMware 无 3D 加速，切换为 CPU `lidar`。
5. LiDAR 桥接配置反复迭代：最终移除物理 LiDAR 桥接，使用数学射线投射。
6. `raycast()` 缩进 Bug：修复 `UnboundLocalError`。
7. DDS 跨进程通信失败：扫描与 mapper 合并为单节点。
8. TF 时间戳不匹配：改用 `self.get_clock().now()`。
9. 斜坡面穿透：新增 45° 坡面交点检测。
10. 坐标系未变换：射线方向先做旋转矩阵变换再 raycast。
11. 墙面线状显示：增加地面/10cm/20cm 三层垂直线填充。
12. 地面点越界：添加 `abs(wy) > HW` 边界检查。
13. 30m 垃圾点：未命中射线改为 `nan`。

## 7. 自主导航

### 7.1 自定义航点导航

`waypoint_navigator.py` 支持：

- 自动加载 `~/pointcloud_maps` 中最新 PCD
- RViz 2D Goal Pose 或 `/waypoint/add` 添加航点
- 航点队列、Marker 和 Path 可视化
- 基于 TF/`/odom` 的位姿估计
- 基于 `/scan` 的前方减速、停止和左右避障

常用接口：

- `/waypoint/add`
- `/waypoint/start`
- `/waypoint/stop`
- `/waypoint/clear`
- `/waypoint/save`
- `/waypoint/load`
- `/waypoint/status`
- `/waypoint/reload_map`

### 7.2 Nav2 预建图导航

`nav_map.launch.py` 启动 Nav2 核心节点，默认加载 `maps/course_map.yaml`。

特点：

- 未启动 `collision_monitor`，避免参数解析问题。
- 全局代价地图只使用静态 `course_map`。
- 局部代价地图使用 `/scan` 进行动态避障。
- `map → odom` 使用 identity 静态 TF。

### 7.3 Nav2 关键参数

```text
robot_radius: 0.08
inflation_radius: 0.12
cost_scaling_factor: 5.0
BaseObstacle.scale: 0.02
```

## 8. 导航调试问题记录

### 8.1 旧 PGM 地图把走廊中心标记为障碍

现象：

```text
GridBased plugin failed to plan from (0.50, 0.00) to (3.10, -0.00):
"Failed to create plan with tolerance of: 0.100000"
```

原因：

旧 PGM 地图投影噪声导致走廊中心整条被标记为 occupied。

解决：

新增 `generate_course_map.py`，按赛道几何生成干净地图：

```text
course_map.pgm/yaml
1186 x 186 @ 0.02 m/cell
```

### 8.2 窄走廊 DWB 找不到合法轨迹

现象：

```text
Could not find a legal trajectory: No valid trajectories out of 20!
Collision Ahead - Exiting Spin
```

原因：

`robot_radius`、`inflation_radius` 和 DWB `BaseObstacle` 权重对 0.3m 走廊过保守。

解决：

调整为：

```text
robot_radius: 0.08
inflation_radius: 0.12
cost_scaling_factor: 5.0
BaseObstacle.scale: 0.02
```

### 8.3 45° 坡面被误判为 2D 障碍

现象：

```text
Begin navigating from current location (19.90, 0.00) to (21.41, 0.02)
GridBased plugin failed to plan from (19.90, 0.00) to (21.41, 0.02)
```

原因：

`synthetic_lidar.py` 把坡面交点写进 `/scan`，Nav2 全局 `obstacle_layer` 将坡面入口标记为 occupied。

解决：

1. `/scan` 不再包含坡面交点，坡面仍通过地面点阵进入 3D 点云。
2. Nav2 全局代价地图只使用静态 `course_map`。

验证：

```text
raycast(x=19.9, 坡面启用): 0.21m
raycast(x=19.9, 坡面禁用): 2.98m
侧墙检测:                  0.15m
```

### 8.4 赛道延长到 20m

将 Seg1 从 3.2m 改为 20m，同步更新：

- `course_test.sdf`
- `course_test.world`
- `synthetic_lidar.py` 扫描常量
- `generate_course_map.py`
- `course_map.pgm/yaml`
- RViz 初始视角
- README、pointCloud、navigate 文档

## 9. 测试结果

| 测试项 | 结果 |
|---|---|
| 编译构建 | `colcon build` 通过 |
| 遥控前进/后退 | 正常 |
| 遥控左右转 | 正常 |
| 原地旋转 | 有小量漂移，符合差速模型预期 |
| 3D 点云建图 | 正常累积并发布 `/pointcloud_map` |
| PCD 保存 | ASCII PCD 保存成功 |
| 自定义航点 | 机器人从 x=0.50 自动移动，`/cmd_vel` 正常输出 |
| Nav2 规划 | `(0.5,0) → (10,0)` 成功返回路径 |
| 上坡规划 | 坡前位置到坡上位置规划成功 |
| 窄走廊控制 | 无 `No valid trajectories` |
| course_map | 1186 × 186，起点/直道/坡道/路口均为 free |

## 10. 遗留问题与改进方向

### 遗留问题

- 旧 PCD 地图仍为旧赛道数据，自定义航点使用前需要重新建图。
- 尚未完成 IMU + 编码器传感器融合。
- 尚未完成真机 SLAM 和真机导航验证。
- Gazebo 仿真依赖数学射线投射，真实 LiDAR 噪声模型未覆盖。
- 原地旋转仍有位置漂移，属于差速模型常见现象。

### 改进方向

- 接入 LD06 真机 LiDAR，替换 `synthetic_lidar` 的数据源。
- 使用 MPU6050 与编码器里程计做数据融合。
- 将 `course_map` 替换为真实 SLAM 栅格地图。
- 增加自动巡线建图脚本，一键完成 20m 赛道的 PCD 重建。
- 在真机环境中验证电磁铁吸附、爬坡和窄道转向。

## 11. 结论

项目已在 ROS2 + Gazebo 仿真环境中完成桥架测量机器人的运动控制、3D 点云建图、PCD 保存、自定义航点导航和 Nav2 预建图导航。针对 DDS 通信、TF 时间戳、栅格地图噪声、窄走廊控制、坡道误判和运动学控制等问题，均已完成定位和修复，并通过仿真验证。

## 附录 A：主要文件

```text
robotProj_ws/autoVehicle/
├── config/
│   ├── nav2_params.yaml
│   └── pointcloud_mapping.rviz
├── launch/
│   ├── gazebo.launch.py
│   ├── nav_map.launch.py
│   └── nav_slam.launch.py
├── maps/
│   └── course_map.pgm/yaml
├── scripts/
│   ├── synthetic_lidar.py
│   ├── waypoint_navigator.py
│   ├── waypoint_cli.py
│   └── generate_course_map.py
├── urdf/
│   └── autoVehicle.urdf
└── world/
    ├── course_test.sdf
    └── course_test.world
```

## 附录 B：常用命令

```bash
# 启动仿真
cd robotProj_ws && source install/setup.bash
ros2 launch autoVehicle gazebo.launch.py

# 启动 Nav2
ros2 launch autoVehicle nav_map.launch.py

# 添加航点并启动
ros2 run autoVehicle waypoint_cli.py add 10.0 0.0 --yaw 0
ros2 run autoVehicle waypoint_cli.py start

# 保存 PCD
ros2 service call /save_map std_srvs/srv/Trigger

# 重新生成 course_map
ros2 run autoVehicle generate_course_map.py
```

## 附录 C：关键日志特征

- 建图正常：`[SCAN #n] wall_hits=... floor=... map=...`
- Nav2 地图正常：`StaticLayer: Resizing costmap to 1186 X 186`
- 规划失败：`GridBased plugin failed to plan ... Failed to create plan`
- 控制失败：`Could not find a legal trajectory: No valid trajectories`
- 坡道误判：`Collision Ahead - Exiting Spin / DriveOnHeading`
