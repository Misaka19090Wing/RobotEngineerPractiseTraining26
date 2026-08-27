# 3D 点云地图构建与保存

基于 ROS2 Jazzy + Gazebo Harmonic + RViz2 的遥控控制 3D 点云地图构建系统。

**当前状态：全部功能正常，可遥控建图 + 自动/手动保存 PCD。**

---

## 1. 架构概述

```
teleop_twist_keyboard  →  /cmd_vel  →  diff_drive_controller.py  →  Gazebo (四轮差速)
                                                                         ↓
                                                                OdometryPublisher (URDF)
                                                                         ↓
                                                              /odom (ros_gz_bridge)
                                                                         ↓
                                                        pose_tf_broadcaster.py
                                                                         ↓
                                                            odom → base_footprint TF
                                                                         ↓
synthetic_lidar.py ← TF(radar_link→odom) ←── robot_state_publisher ←── URDF
       │
       ├── /scan (LaserScan, 10Hz) ──────────→ RViz2 (Live LiDAR)
       ├── /lidar_points (PointCloud2, 10Hz) ──→ RViz2 (Live LiDAR 3D)
       ├── /pointcloud_map (PointCloud2, 2Hz) ─→ RViz2 (PointCloud Map)
       └── /save_map (Trigger) ──────────────────→ ~/pointcloud_maps/*.pcd
```

**synthetic_lidar.py 是唯一核心节点**——集成了 LiDAR 扫描、TF 变换、点云累积、地图发布、PCD 保存。单节点架构消除了 DDS 跨进程通信问题。

完整 TF 树：

```
odom → base_footprint → base_link → (wheels, radar_link)
  ^          ^               ^
pose_tf    static TF    robot_state_publisher
broadcaster            + joint_state_publisher
```

---

## 2. 文件结构

```
robotProj_ws/autoVehicle/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── bridge_lidar.yaml           # LiDAR 桥接 YAML 配置（备用）
│   ├── diag.sh                     # 运行诊断脚本
│   ├── joint_names_autoVehicle.yaml
│   └── pointcloud_mapping.rviz     # RViz2 配置
├── launch/
│   ├── display.launch.py
│   └── gazebo.launch.py            # 主启动文件
├── meshes/                         # STL 模型文件
├── resource/
├── scripts/
│   ├── diff_drive_controller.py   # 四轮差速控制器
│   ├── pointcloud_mapper.py       # 独立点云累积节点（已被集成替代）
│   ├── pose_tf_broadcaster.py     # odom TF 广播器
│   └── synthetic_lidar.py         # ★ 集成 LiDAR + 点云建图（核心）
├── urdf/
│   └── autoVehicle.urdf           # 机器人 URDF（含 LiDAR + IMU 传感器）
└── world/
    ├── course_test.sdf             # 赛道世界
    └── test_flat.sdf               # 平坦测试世界
```

---

## 3. 核心组件

### 3.1 synthetic_lidar.py（集成 LiDAR + 建图）

由于 Gazebo Harmonic 在 VMware 环境下无法正确加载 URDF-based `<gazebo><sensor>` 元素（传感器不发布数据），且 ROS 2 DDS 跨进程通信在虚拟环境中不稳定，最终采用**单节点集成方案**。

**扫描功能**：

| 组件       | 参数                  | 说明                                                      |
| ---------- | --------------------- | --------------------------------------------------------- |
| 墙壁检测   | 720 条 2D 射线（可配置）| 对赛道几何做数学投射，包含坐标系变换（radar_link→world） |
| 斜坡面检测 | 基于 LiDAR 高度`oz` | 45° 斜坡交点 = RAMP_X0 + oz                              |
| 垂直线填充 | 每墙点生成 3 层       | 地平面/10cm/20cm（墙顶），构建完整墙面                    |
| 墙面插值     | 相邻墙点间 0.05m 插值 | 减少突出墙/窄墙被 1° 角分辨率漏扫                         |
| 地面扫描   | 36 方位 × 10 距离步  | 0.5m~5m 环形地面点阵，验证走廊/路口边界                   |

**建图功能**：

- TF 变换：`radar_link` → `odom` 坐标系，旋转+平移
- 点云累积：直接在进程内累积所有历史点云
- 地图发布：每 0.5 秒发布 `/pointcloud_map`（下采样至 200K 上限）
- 自动保存：每 500K 点触发一次 PCD 文件保存

### 3.2 pose_tf_broadcaster.py

订阅 `/odom`（Gazebo 里程计）和 `/model/autoVehicle/pose`（Gazebo 模型位姿），发布动态 `odom → base_footprint` TF 变换。包含 10Hz identity 回退定时器，确保 `odom` 帧始终存在。收到里程计数据后打印 `[ODOM RECEIVED]` 并切换到动态 TF。

### 3.3 diff_drive_controller.py

订阅 `/cmd_vel`，计算四轮差速目标转速，通过 `ros_gz_bridge` 发布到 Gazebo 关节速度话题。

### 3.4 URDF 传感器

- **CPU `lidar`**（ray 类型）：2D 激光扫描器，720 射线（可配置），10Hz，`visualize=true`
- **IMU**（imu 类型）：200Hz，附着于 `base_link`，含高斯噪声模型
- **OdometryPublisher**：Gazebo 系统插件，发布里程计到 `/model/autoVehicle/odometry`

---

## 4. 赛道几何模型

硬编码了 `course_test.sdf` 的完整几何参数：

| 区域           | X 范围         | Z 范围     | 墙壁                                                        |
| -------------- | -------------- | ---------- | ----------------------------------------------------------- |
| 平直段 Seg1    | [0, 20]        | Z=0        | Y = ±0.15                                                  |
| 45° 斜坡 Seg2 | [20, 21.13]    | Z: 0→1.13 | Y = ±0.15；坡面会阻挡水平射线                              |
| 干扰段 Seg3    | [21.13, 22.58] | Z=1.13     | Y = ±0.15                                                  |
| T 字路口       | X≈22.73       | Z=1.13     | 左墙 X=22.58 (带缺口\|Y\|<0.2)，右墙 X=22.88，端墙 Y=±1.65 |

**墙壁检测函数 `raycast(ox, oy, oz, angle)`** 检查 5 种交点（取最近）：

1. 走廊侧墙（Y=±0.15, X<22.58）
2. T 字路口左墙（X=22.58, 带入口缺口）
3. T 字路口右墙（X=22.88）
4. T 字路口端墙（Y=±1.65）
5. **45° 斜坡面**（Z = X-20，当 LiDAR 高度在坡面范围内时检测）

**坐标系变换**：每条射线方向从 `radar_link` 坐标系通过旋转矩阵变换到世界坐标系后再做射线投射，确保机器人旋转时墙壁检测方向正确。

---

## 5. 启动方式

```bash
cd ~/RobotEngineerPractiseTraining26/robotProj_ws
source install/setup.bash
ros2 launch autoVehicle gazebo.launch.py
```

遥控（另一个终端）：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

诊断（运行时）：

```bash
~/RobotEngineerPractiseTraining26/robotProj_ws/autoVehicle/config/diag.sh
```

保存地图（手动触发）：

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

地图文件位置：`~/pointcloud_maps/map_YYYYMMDD_HHMMSS.pcd`（ASCII PCD v0.7 格式）

---

## 6. 已知问题与解决方案

| 问题                          | 原因                                             | 解决方案                                  |
| ----------------------------- | ------------------------------------------------ | ----------------------------------------- |
| Gazebo LiDAR 传感器不发布数据 | VMware 无 3D 加速 / URDF sensor 不被 Gazebo 加载 | 使用数学射线投射替代物理传感器            |
| ROS 2 DDS 跨进程通信失败      | 虚拟环境 DDS 发现不稳定                          | 单节点集成扫描+建图                       |
| 点云形成"圆环" / Y 方向偏移   | TF 查询使用`rclpy.time.Time()`(时间0)          | 改用`self.get_clock().now()`            |
| 地面点超出墙壁                | 地面网格未验证走廊边界                           | 添加边界检查`abs(wy) > HW`              |
| 30m 垃圾点（极值噪声）        | 未命中射线设为 RNG_MAX                           | 改为`float('nan')`，被 mapper 自动跳过  |
| 射线穿透 45° 斜坡            | raycast 只检测垂直墙壁                           | 新增斜坡面交点检测                        |
| 机器人旋转后点云偏移          | 射线角度在 radar_link 帧但 raycast 假设世界帧    | 射线方向先做旋转矩阵变换再传入 raycast    |
| 墙面只显示一条线              | 2D LiDAR 物理限制                                | 垂直线填充：每墙点生成地面/10cm/20cm 三层 |
| IMU 传感器 SDF 警告           | URDF→SDF 转换中 noise 属性命名差异              | 无功能影响，可忽略                        |

---

## 7. 运行效果

- 扫描速率：10Hz（720 射线墙壁 + 36×10 环形地面）
- 单帧点数：约 400~500 点（含垂直填充）
- 累积速率：约 4,000~5,000 点/秒
- 自动保存间隔：每 500K 点触发一次
- 机器人可遥控行驶通过平直段→45°斜坡→干扰段→T 字路口
- 点云地图实时反映墙壁表面（3D）、地面高程（含坡面）、走廊轮廓

---

## 8. 关键 Bug 修复历程

按时间顺序：

1. **TF 树方向错误**：`base_link → base_footprint` 反了，修正为 `base_footprint → base_link`
2. **缺少 joint_state_publisher**：`robot_state_publisher` 需要 joint_states 才能发布轮子 TF
3. **Header 类型错误**：`rclpy.time.Time().to_msg()` 返回 `Time` 不是 `Header` → `AttributeError`
4. **`gpu_lidar` 需要 OGRE2 渲染引擎**：VMware 无 3D 加速时不可用，切换到 CPU `lidar`
5. **桥接配置多次迭代**：命令行参数→YAML→命令行，最终因 Gazebo 传感器不工作而移除所有 LiDAR 桥接
6. **`raycast()` 缩进 Bug**：`ix`/`iy` 变量在条件外被引用 → `UnboundLocalError`
7. **DDS 跨进程通信失败**：synthetic_lidar 发布消息但 pointcloud_mapper 永远收不到 → 合并为单节点
8. **TF 时间戳不匹配**：`rclpy.time.Time()`→`self.get_clock().now()` 解决偏移
9. **斜坡面穿透**：raycast 只检测垂直墙壁 → 新增 45° 坡面交点检测
10. **坐标系未变换**：射线角度在 radar_link 帧传入 raycast 但函数假设世界帧 → 加入旋转矩阵变换
11. **墙面线状显示**：2D LiDAR 只有一层 → 垂直线填充（地/中/顶）

## 9. 航点自主导航（基于已保存 PCD）

新增 `waypoint_navigator.py`，把建好的 PCD 地图直接作为航点巡航的上下文：

1. 节点自动加载 `~/pointcloud_maps` 中最新的 `.pcd`（也可用 `map_file` 参数指定），发布为 `/waypoint_map`，供 RViz 显示。
2. 使用 RViz 的 **2D Goal Pose** 工具点选航点，节点收到 `/goal_pose` 后加入队列；命令行通过 `/waypoint/add` 服务添加航点。
3. `waypoint/start` 启动后，节点订阅 TF/`/odom` 获取位姿，按差速模型输出 `/cmd_vel`，并用 `/scan` 做前方障碍减速/停止与左右避让。

命令行示例：

```bash
ros2 run autoVehicle waypoint_cli.py add 3.0 0.0 --yaw 0
ros2 run autoVehicle waypoint_cli.py add 22.73 1.2 --yaw 90
ros2 run autoVehicle waypoint_cli.py start
```

相关接口：

- `/waypoint_map`、`/waypoint_markers`、`/waypoint_path`
- `/waypoint/add`、`/waypoint/start`、`/waypoint/stop`、`/waypoint/clear`
- `/waypoint/save`、`/waypoint/load`、`/waypoint/status`
- `/waypoint/reload_map` — 保存新 PCD 后无需重启，直接刷新地图

如果使用 Nav2 预建图导航，可在 Gazebo 启动后运行：

```bash
ros2 launch autoVehicle nav_map.launch.py
```

该启动文件已包含 `map_server`、Nav2 核心节点（不含 `collision_monitor`）和 `map→odom`
静态 TF，默认地图为 `maps/course_map.yaml`。

> 旧版 `map_20260807_232605.pgm` 曾把走廊中心整条标记为 occupied，导致 Nav2 始终
> “Failed to create plan”。现已新增 `generate_course_map.py`，按赛道几何生成干净的
> `course_map.pgm/yaml`，可执行 `ros2 run autoVehicle generate_course_map.py` 重新生成。

`nav2_params.yaml` 也已针对 0.3 m 窄走廊调整：`robot_radius=0.08`、
`inflation_radius=0.12`、DWB `BaseObstacle.scale=0.02`，避免控制器在两侧墙壁之间
找不到合法轨迹。

上坡导航：`synthetic_lidar.py` 的 `/scan` 不再包含 45° 坡面交点，坡面仍通过地面点阵
进入 3D 点云；Nav2 全局代价地图只使用静态 `course_map`，实时 `/scan` 仅用于局部避障，
避免坡道入口被误判为障碍导致 `Goal failed`。
