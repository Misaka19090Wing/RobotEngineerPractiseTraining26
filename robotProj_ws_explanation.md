# robotProj_ws 项目详解

本文档逐部分解释 `robotProj_ws` 项目：每个文件是什么、为什么存在、怎么实现、以及如何运行。

## 1. 项目总览

`robotProj_ws` 是一个 ROS2 ament 工作空间，内部包含一个 ROS2 功能包：

```text
robotProj_ws/
├── autoVehicle/       # ROS2 功能包源码
├── build/             # colcon 编译中间产物
├── install/           # colcon 安装结果，运行入口
├── log/               # colcon 编译日志
├── src/               # 可选的源码软链接目录
└── maps/              # 早期地图文件目录
```

### 是什么
- 工作空间：ROS2 编译、安装、运行的组织方式。
- 功能包：`autoVehicle`，包含机器人描述、仿真世界、建图、导航等所有代码。

### 为什么
ROS2 需要通过 `colcon` 编译和安装包，才能用 `ros2 run` / `ros2 launch` 启动节点。使用 `--symlink-install` 可以让 Python 脚本直接链接源码，修改后无需重复安装。

### 怎么做

```bash
cd robotProj_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select autoVehicle
source install/setup.bash
```

---

## 2. autoVehicle 功能包目录

```text
autoVehicle/
├── CMakeLists.txt
├── package.xml
├── config/
├── launch/
├── maps/
├── meshes/
├── resource/
├── scripts/
├── urdf/
└── world/
```

### 2.1 package.xml

**是什么**：ROS2 包的元信息，包括名称、依赖、许可证。

**为什么**：`ros2 launch`、`colcon build` 和依赖管理都靠它识别包和运行依赖。

**怎么做**：声明 `ament_cmake` 构建类型，以及 `robot_state_publisher`、`rviz2`、`ros_gz_bridge`、`nav_msgs`、`visualization_msgs`、`rcl_interfaces`、`tf2_ros` 等运行依赖。

### 2.2 CMakeLists.txt

**是什么**：构建和安装规则。

**为什么**：需要把 `config`、`launch`、`maps`、`meshes`、`urdf`、`world`、`scripts` 安装到 `share/autoVehicle`，并把 Python 脚本安装为可执行程序。

**怎么做**：

```cmake
install(DIRECTORY config launch maps meshes urdf world scripts
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/diff_drive_controller.py
  scripts/synthetic_lidar.py
  scripts/waypoint_navigator.py
  ...
  DESTINATION lib/${PROJECT_NAME})
```

---

## 3. 机器人模型：urdf 与 meshes

### 3.1 urdf/autoVehicle.urdf

**是什么**：机器人 URDF 描述文件，定义底盘、四个轮子、雷达、IMU、关节和传感器。

**为什么**：`robot_state_publisher` 需要 URDF 才能发布机器人内部 TF；Gazebo 需要 collision、inertial 和 friction 才能做物理仿真。

**怎么做**：

- `base_link`：底盘，尺寸约 165×140×57.5mm。
- `left_front_joint` / `left_rear_joint` / `right_front_joint` / `right_rear_joint`：四个连续旋转关节。
- 四个轮子：左右对称，Y 方向间距约 148mm。
- `radar_link`：雷达安装位置，通过 `radar_joint` 固定在底盘上。
- Gazebo 插件：
  - `JointController`：允许外部通过 joint velocity topic 控制轮子。
  - `OdometryPublisher`：发布地面真实里程计 `/model/autoVehicle/odometry`。
  - CPU `lidar` 传感器：定义 360 线 2D 激光扫描。
  - IMU 传感器：200Hz，含高斯噪声。

### 3.2 meshes/

**是什么**：SolidWorks 导出的 STL 三维模型。

**为什么**：用于 RViz 和 Gazebo 的可视化，比简单几何体更接近真实外形。

**怎么做**：URDF `<visual>` 中通过 `package://autoVehicle/meshes/xxx.STL` 引用。

### 3.3 为什么最终轮子碰撞用球体

调试结论：

- STL、圆柱、方盒在 15mm 小轮子上转向不稳定。
- 球体碰撞各向摩擦均匀，配合 DART 摩擦参数最稳定。
- 最终轮子碰撞使用球体 `r=0.02m`，DART 摩擦 `primary=100 / secondary=100`。

---

## 4. 世界文件：world/

### 4.1 test_flat.sdf / test_flat.world

**是什么**：200m×200m 白色平坦地面测试世界。

**为什么**：用于最基础的遥控、碰撞、摩擦和控制器调试，排除赛道复杂度。

**怎么做**：通过 Gazebo 地面平面 + DART/ODE 摩擦标签实现。

### 4.2 course_test.sdf / course_test.world

**是什么**：完整测试赛道，模拟桥架环境。

**为什么**：验证机器人在长直道、45°坡道、干扰段和 T 字路口的运动与导航能力。

**怎么做**：

| 区域 | X 范围 (m) | 说明 |
|---|---|---|
| Seg1 平直段 | 0 ~ 20 | 20m 长直道，宽 0.3m |
| Seg2 45° 坡 | 20 ~ 21.13 | 高度从 0 到 1.13m |
| Seg3 干扰段 | 21.13 ~ 22.58 | 地面带多个小凸起 |
| T 字路口 | 约 22.73 | Y 方向展开 ±1.6m |

每段都由静态 `test_course` 模型中的多个 link 组成：地面、左右墙、坡面、路口墙等。

---

## 5. Launch 启动文件

### 5.1 gazebo.launch.py（主启动）

**是什么**：一键启动完整仿真系统。

**为什么**：把所有节点、桥接、TF、RViz 放在一个 launch 里，避免手动逐个启动。

**启动内容**：

1. Gazebo Harmonic，默认加载 `course_test.sdf`。
2. `robot_state_publisher`：发布 URDF 静态 TF。
3. `joint_state_publisher`：发布关节状态。
4. `static_transform_publisher` ×2：`base_footprint → base_link` 和备用 `odom → base_footprint`。
5. `ros_gz_bridge`：
   - `/cmd_vel` 双向桥接
   - `/model/autoVehicle/odometry → /odom`
   - IMU 桥接
   - 四个轮子 joint cmd_vel 桥接
6. `diff_drive_controller`：把 `/cmd_vel` 转成轮速。
7. `pose_tf_broadcaster`：发布动态 `odom → base_footprint`。
8. `synthetic_lidar`：合成雷达 + 点云建图 + PCD 保存。
9. `waypoint_navigator`：PCD 航点导航。
10. RViz2。

**为什么使用 TimerAction**：Gazebo 和机器人 spawn 需要时间，节点延迟启动可以避免 TF/话题还没就绪就启动。

### 5.2 display.launch.py

**是什么**：纯模型显示 launch，不启动仿真。

**为什么**：单独查看 URDF、关节和 RViz 模型。

**怎么做**：启动 `robot_state_publisher`、`joint_state_publisher_gui` 和 RViz。

### 5.3 nav_map.launch.py

**是什么**：Nav2 预建图导航启动文件。

**为什么**：在 `course_map` 基础上运行 Nav2，支持 2D Goal Pose 和路径规划。

**怎么做**：

- 启动 `map_server`，加载 `maps/course_map.yaml`。
- 发布 `map → odom` identity TF。
- 启动 `planner_server`、`controller_server`、`behavior_server`、`bt_navigator`、`waypoint_follower`。
- 通过 `lifecycle_manager_navigation` 自动激活。
- 不启动 `collision_monitor`，避免之前出现的参数解析错误。

### 5.4 nav_slam.launch.py

**是什么**：SLAM 建图入口，使用 `slam_toolbox`。

**为什么**：如果需要从 `/scan` 实时生成栅格地图，可以单独运行它。

**怎么做**：

```bash
ros2 launch autoVehicle nav_slam.launch.py
```

---

## 6. 核心 Python 脚本

### 6.1 diff_drive_controller.py

**是什么**：四轮差速控制器。

**为什么**：Gazebo 的默认 DiffDrive 插件在小尺寸机器人上转向不稳定，因此改为自定义运动学控制器。

**怎么做**：

```text
v_left  = (linear - angular * wheel_separation / 2) / wheel_radius
v_right = (linear + angular * wheel_separation / 2) / wheel_radius
```

订阅 `/cmd_vel`，分别发布到四个轮子的 joint cmd_vel topic。

### 6.2 pose_tf_broadcaster.py

**是什么**：`odom → base_footprint` TF 广播器。

**为什么**：Gazebo 只提供里程计数据，不直接提供 TF，需要自己把机器人位姿发布到 TF 树。

**怎么做**：

- 订阅 `/odom` 和 `/model/autoVehicle/pose`。
- 收到数据后发布动态 TF。
- 没收到数据时发布 identity TF 兜底，保证 `odom` 帧一直存在。

### 6.3 synthetic_lidar.py（核心建图节点）

**是什么**：合成 LiDAR + 点云累积 + 地图发布 + PCD 保存的集成节点。

**为什么**：

- VMware 虚拟机没有 3D 加速，Gazebo 的 GPU/URDF LiDAR 不发布数据。
- DDS 跨进程通信在虚拟机中不稳定，所以扫描和建图合并到单节点。

**怎么做**：

- 通过 TF 获取 `radar_link` 在 `odom` 下的位姿。
- 生成 360 条射线，与赛道墙壁、路口、坡面做数学求交。
- 每条命中射线生成墙点，并补充地面/10cm/20cm 三层墙面填充。
- 生成 36 方位 × 10 距离步的地面点阵。
- 累积到 `map_pts`，发布 `/pointcloud_map`。
- 通过 `/save_map` 服务保存 ASCII PCD。

**为什么 `/scan` 要排除坡面**：

坡面是地面，不是 2D 障碍。如果不排除，Nav2 会把坡道入口当成墙，导致 `Goal failed`。

### 6.4 pointcloud_mapper.py（旧版）

**是什么**：独立的点云 mapper 节点。

**为什么**：早期方案，后来因为 DDS 跨进程通信不稳定，功能被合并进 `synthetic_lidar.py`。文件保留但通常不再启动。

### 6.5 waypoint_navigator.py

**是什么**：自定义航点导航节点。

**为什么**：让用户基于已保存的 PCD 地图手动设立航点，机器人按航点队列自主行驶。

**怎么做**：

1. 自动加载 `~/pointcloud_maps` 中最新的 PCD。
2. 发布 `/waypoint_map`，供 RViz 显示。
3. 支持 `/goal_pose` 或 `/waypoint/add` 添加航点。
4. 通过 TF/`/odom` 获取机器人位姿。
5. 按航点队列输出 `/cmd_vel`。
6. 使用 `/scan` 做前方障碍减速、停止和左右避障。

服务列表：

```text
/waypoint/add
/waypoint/start
/waypoint/stop
/waypoint/clear
/waypoint/save
/waypoint/load
/waypoint/status
/waypoint/reload_map
```

### 6.6 waypoint_cli.py

**是什么**：航点命令行工具。

**为什么**：RViz 之外还需要一种可靠的手动添加方式；通过服务调用避免 DDS 发布延迟。

**怎么做**：

```bash
ros2 run autoVehicle waypoint_cli.py add 10.0 0.0 --yaw 0
ros2 run autoVehicle waypoint_cli.py start
ros2 run autoVehicle waypoint_cli.py status
```

### 6.7 generate_course_map.py

**是什么**：生成 Nav2 使用的 `course_map.pgm/yaml`。

**为什么**：旧 PGM 地图把走廊中心标成障碍，导致 Nav2 无法规划；按已知赛道几何生成地图最稳定。

**怎么做**：

- 以 0.02m 分辨率遍历地图。
- 直道内 `|y|≤0.15` 或路口内 `|y|≤1.6` 标为 free。
- 其余区域标为 occupied。
- 输出二进制 PGM 和 YAML。

---

## 7. 配置文件

### 7.1 config/nav2_params.yaml

**是什么**：Nav2 节点参数。

**为什么**：0.3m 窄走廊需要小半径、小膨胀和低 DWB 障碍权重，否则找不到合法轨迹。

**关键参数**：

```text
robot_radius: 0.08
inflation_radius: 0.12
cost_scaling_factor: 5.0
BaseObstacle.scale: 0.02
```

全局代价地图只使用静态 `course_map`，局部代价地图使用 `/scan` 避障。

### 7.2 config/pointcloud_mapping.rviz

**是什么**：RViz2 配置。

**为什么**：预设点云、航点、路径、TF、机器人模型和 2D Goal Pose 工具。

**怎么做**：

- 显示 `/pointcloud_map`、`/lidar_points`、`/waypoint_map`。
- 显示 `/waypoint_markers` 和 `/waypoint_path`。
- 固定坐标系为 `odom`。
- 加入 `nav2_rviz_plugins/GoalTool`。

### 7.3 config/bridge_lidar.yaml

**是什么**：LiDAR/里程计桥接 YAML 备用配置。

**为什么**：早期尝试用 `ros_gz_bridge` 桥接 Gazebo 传感器；后来因 Gazebo 传感器不工作改为合成扫描，文件保留备用。

### 7.4 config/diag.sh

**是什么**：运行时诊断脚本。

**为什么**：快速检查话题、TF、Gazebo topic 和桥接节点是否正常。

**怎么做**：

```bash
./autoVehicle/config/diag.sh
```

---

## 8. 地图文件：maps/

### 8.1 course_map.pgm / course_map.yaml

**是什么**：Nav2 使用的干净栅格地图。

**为什么**：旧 PGM 地图存在走廊中心 occupied 问题，无法规划；course_map 由赛道几何重新生成。

**参数**：

```text
resolution: 0.02
origin: [-0.5, -2.0, 0.0]
1186 x 186
```

### 8.2 map_20260807_232605.pgm/yaml

**是什么**：早期保存的旧地图。

**为什么保留**：作为调试对比材料；Nav2 默认不再使用。

---

## 9. 整体数据流

```text
teleop / waypoint_navigator / Nav2
                 │
                 ▼
            /cmd_vel
                 │
                 ▼
      diff_drive_controller.py
                 │
                 ▼
       ros_gz_bridge (joint cmd_vel)
                 │
                 ▼
      Gazebo JointController → 轮子转动
                 │
                 ▼
       OdometryPublisher → /odom
                 │
                 ▼
      pose_tf_broadcaster → odom → base_footprint
                 │
                 ▼
  synthetic_lidar / waypoint_navigator / Nav2
```

## 10. 常见问题结论

### 为什么机器人转向不稳？
小尺寸轮子摩擦不足，需要球体碰撞 + DART 高摩擦 + 自定义差速控制器。

### 为什么不用 Gazebo LiDAR？
VMware 无 3D 加速，URDF sensor 不发布数据，改用数学射线投射。

### 为什么 Nav2 找不到路径？
旧 PGM 地图中心被标为 occupied，换成 `course_map` 后解决。

### 为什么 Nav2 没有合法轨迹？
窄走廊参数过保守，调小 `robot_radius`、`inflation_radius` 和 `BaseObstacle.scale` 后解决。

### 为什么上坡会 Goal failed？
坡面被当作 2D 障碍，`/scan` 排除坡面且全局代价地图只使用静态地图后解决。

---

## 11. 常用命令汇总

```bash
# 编译
cd robotProj_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select autoVehicle
source install/setup.bash

# 启动完整仿真
ros2 launch autoVehicle gazebo.launch.py

# 启动 Nav2
ros2 launch autoVehicle nav_map.launch.py

# 启动 SLAM 建图
ros2 launch autoVehicle nav_slam.launch.py

# 航点命令行
ros2 run autoVehicle waypoint_cli.py add 10.0 0.0 --yaw 0
ros2 run autoVehicle waypoint_cli.py start

# 保存 PCD
ros2 service call /save_map std_srvs/srv/Trigger

# 重新生成 course_map
ros2 run autoVehicle generate_course_map.py

# 诊断
./autoVehicle/config/diag.sh
```
