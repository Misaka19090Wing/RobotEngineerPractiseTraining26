# 桥架测量机器人设计方案

## 项目状态

- [X] **仿真环境搭建完成** — ROS2 Jazzy + Gazebo Harmonic + RViz2
- [X] **遥控驾驶** — teleop_twist_keyboard + 四轮差速控制器
- [X] **3D 点云建图** — synthetic_lidar.py 集成扫描+累积+保存
- [X] **PCD 地图保存** — 自动/手动保存 ASCII PCD 文件
- [X] **航点自主导航** — 基于已保存 PCD 地图手动设立航点
- [X] **Nav2 预建图导航** — `nav_map.launch.py` 可选启动
- [ ] 传感器融合（IMU + 里程计）
- [ ] SLAM / 完整导航栈

> **详细文档**：仿真实现与建图见 [pointCloud.md](pointCloud.md)，导航与排障见 [navigate.md](navigate.md)。

---

## 1. 总体概述

机器人采用**四轮独立直驱差速转向**布局，整车尺寸 ≤180 mm × 180 mm × 95 mm，总重 ≤2 kg，具备原地转向能力。通过**电磁铁吸附**在金属桥架表面行驶，可通行平直段、45°斜坡及直角弯。
感知系统采用**高分辨率光电编码器 + 小型激光雷达 + IMU**，融合轮式里程计与激光里程计，实现 10 m 内累计误差 ≤0.1 m（≤1%）。
软件基于 ROS2，集成点云处理、SLAM、路径规划及自主导航，支持遥控驾驶与航点自主巡航，并实时显示环境点云和分段距离。

---

## 2. 运动学模型

### 2.1 尺寸参数

底盘参数：$165*140*12(mm^3)$
底盘带电机距离地面最小距离：$3mm$
车轮直径：$30 mm$，半径 $r=0.015 m$
轮距 $B=0.148m$，轴距 $L=0.14 m$（近似正方形布局）
机器人参考点：底盘几何中心

### 2.2 差速转向方程

四轮电机采用左前/左后同步、右前/右后同步的控制方式。
令左侧车轮转速为 $ω_L$，右侧车轮转速为 $ω_R$（单位 rad/s），则中心线速度 $v$ 与角速度 $ω_z$ 为：

$$
\begin{align}
v&=\frac{r}{2}(ω_L+ω_R) \\
ω_z&=\frac{r}{2d}(ω_R−ω_L),d=\frac{\sqrt{B^2+L^2}}{2}≈0.099m
\end{align}
$$

**原地转向**：令 $ω_L=−ω_R$，此时 $v=0$，转弯半径 $R=0$，可在狭窄桥架内绕中心旋转。

### 2.3 ROS2 节点

编写 `diff_drive_controller` 节点，订阅 `/cmd_vel`，计算四轮目标转速，并通过底层控制板驱动电机；同时根据编码器反馈发布 `/odom`（轮式里程计）和 TF 变换。

---

## 3. 机械结构设计

### 3.1 整体布局

- 底板采用碳纤维板，外形 165 mm × 140 mm，保证轮缘不超出 180 mm 边界。
- 车轮：4 个直径 30 mm 硅胶轮胎，直接套装在无刷电机外转子。
- 上层安装控制板、激光雷达，电池和电磁铁，下层安装电机。
- 整车高度：车轮半径 15 mm + 底盘间隙 + 上层器件，总高 ≤90 mm（加装 LD06 雷达后 ≤95 mm）。

### 3.2 主要器件选型与重量估算

| 部件           | 型号                                 | 数量 | 单重(g) | 总重(g)            |
| -------------- | ------------------------------------ | ---- | ------- | ------------------ |
| 无刷减速电机   | DJI M2006（24V，36:1，4096线编码器） | 4    | 90      | 360                |
| 电机驱动       | SimpleFOC Mini（宽压版，支持24V）    | 4    | 12      | 48                 |
| 激光雷达       | 思岚 LD06（10Hz）                    | 1    | 120     | 120                |
| IMU            | MPU6050 模块                         | 1    | 5       | 5                  |
| 主控板         | 树莓派 4B（4GB）                     | 1    | 45      | 45                 |
| 底层控制板     | STM32F407VET6                        | 1    | 30      | 30                 |
| 电池           | 6S LiPo 1300mAh（22.2V）             | 1    | 210     | 210                |
| 电磁铁         | KK-P20/15（额定吸力 20N）            | 4    | 55      | 220                |
| 车轮           | 30mm 硅胶轮                          | 4    | 10      | 40                 |
| 底板及结构件   | 碳纤维板+3D打印件                    | 1    | 120     | 120                |
| 其他           | 线材、降压模块、螺丝                 | —   | 50      | 50                 |
| **总计** |                                      |      |         | **≈1258 g** |

总重约 1.26 kg，远小于 2 kg 指标，保留充足负载余量。

### 3.3 爬坡与电磁铁吸附力计算

- 最大设计总质量 $m=2 kg$，重力 $G=19.6 N$
- 45° 斜坡下滑力：$F_{down}=mgsin⁡45\degree≈13.86 N$
- 硅胶轮与钢面摩擦系数 $μ=0.7$，所需总正压力：

  $N_{total}≥\frac{F_{down}}{μ}=\frac{13.86}{0.7}≈19.8 N$
- 重力法向分力：$N_g=mgcos⁡45\degree≈13.86 N$
- 需电磁铁额外吸附力：$F_{mag\_sum}≥19.8−13.86=5.94 N$
- 单个电磁铁所需吸力：≥1.5 N，所选 KK-P20/15（额定 20N）留有 13 倍安全余量，可克服气隙、灰尘等因素。

**电机扭矩校核**：M2006 额定输出扭矩 0.18 N·m（减速后），爬坡时每轮所需扭矩约 0.075 N·m，远小于电机能力，满足需求。

### 3.4 转弯能力

差速模型可近似实现原地转向，能顺利通过桥架中的 90° 直角弯。

---

## 4. 电气系统设计

### 4.1 供电架构

- **电池**：6S LiPo 22.2V（1300mAh，35C），直接为电机驱动供电。
- **系统供电**：电池经 DC-DC 降压模块（24V→5V/3A）供给树莓派、LD06；5V 再经 LDO 转为 3.3V 供 STM32 和 MPU6050。
- **保护**：输入端串联保险丝，电池使用 XT30 接口，硬件低电压报警。

### 4.2 电机驱动与编码器

- 四路 SimpleFOC Mini 驱动板，支持 24V 输入，通过 CAN/串口与 STM32 通信。
- M2006 自带 4096 线输出轴编码器，信号直接接入驱动板，实现 FOC 速度/位置闭环，同时回传至 STM32 用于里程计计算。

### 4.3 感知与通讯接口

- **LD06 激光雷达**：UART 转 USB 连接树莓派，10 Hz 扫描，ROS2 驱动 `sllidar_ros2`。
- **MPU6050 IMU**：I2C 连接 STM32，200 Hz 读取角速度和加速度，经低通滤波后发送给树莓派。
- **摄像头**：树莓派 CSI 接口广角摄像头，用于遥控场景视野。
- **STM32–树莓派**：USB 虚拟串口，200 Hz 频率交换电机状态与控制指令。

---

## 5. 软件系统（ROS2 仿真实现）

### 5.1 当前已实现节点

| 节点                            | 可执行文件                   | 功能                                                                    |
| ------------------------------- | ---------------------------- | ----------------------------------------------------------------------- |
| **diff_drive_controller** | `diff_drive_controller.py` | 订阅`/cmd_vel`，计算四轮转速，通过 `ros_gz_bridge` 驱动 Gazebo 关节 |
| **pose_tf_broadcaster**   | `pose_tf_broadcaster.py`   | 订阅`/odom`，发布动态 `odom→base_footprint` TF                     |
| **synthetic_lidar** ★    | `synthetic_lidar.py`       | **集成 LiDAR 扫描 + 点云累积 + 地图发布 + PCD 保存**              |
| **waypoint_navigator** ★ | `waypoint_navigator.py`    | **加载 PCD 地图、手动航点队列、差速巡航 + 激光避障**              |
| `waypoint_cli`                | `waypoint_cli.py`          | 命令行添加/启停/保存航点                                                |
| `generate_course_map`         | `generate_course_map.py`   | 按赛道几何生成 Nav2 使用的`course_map.pgm/yaml`                       |
| `robot_state_publisher`       | ROS2 标准                    | 从 URDF 发布`base_link→child_links` TF                               |
| `joint_state_publisher`       | ROS2 标准                    | 发布零位关节状态，驱动轮子 TF                                           |
| `ros_gz_bridge` ×8           | ROS2 标准                    | cmd_vel、odometry、IMU、关节速度 桥接                                   |

> ★ `synthetic_lidar.py` 是整个点云建图系统的唯一核心节点。详见架构图和 [pointCloud.md](pointCloud.md)。

### 5.2 数据流架构

```
teleop → /cmd_vel → diff_drive_controller → ros_gz_bridge → Gazebo joints → 底盘运动
                                                                       ↓
                                                               OdometryPublisher
                                                                       ↓
                                                     ros_gz_bridge → /odom
                                                                       ↓
                                                           pose_tf_broadcaster
                                                                       ↓
                                                               odom TF 树
                                                                       ↓
synthetic_lidar.py ← TF(radar_link→odom) ← robot_state_publisher ← URDF
       │
       ├── /scan            (10Hz) → RViz2 Live LiDAR
       ├── /lidar_points     (10Hz) → RViz2 Live 3D Points
       ├── /pointcloud_map  (2Hz)  → RViz2 PointCloud Map
       └── /save_map        (手动)  → ~/pointcloud_maps/*.pcd

waypoint_navigator.py ← PCD(~/pointcloud_maps/*.pcd) + /odom or TF + /scan
       │
       ├── /waypoint_map     → RViz2 PCD Map
       ├── /waypoint_markers → RViz2 航点队列
       ├── /waypoint_path    → RViz2 剩余路径
       ├── /goal_pose        ← RViz2 2D Goal Pose
       └── /cmd_vel          → diff_drive_controller → Gazebo

nav_map.launch.py ← course_map + /scan + TF
       │
       ├── map_server       → /map → global_costmap
       ├── planner_server   → 路径规划
       ├── controller_server → /cmd_vel → diff_drive_controller
       └── /goal_pose       ← RViz2 2D Goal Pose
```

完整 TF 树：

```
odom → base_footprint → base_link → (left_front_wheel, right_rear_wheel, radar_link, ...)
```

### 5.3 仿真环境

- **ROS2**: Jazzy
- **Gazebo**: Harmonic (gz-sim 8.11)
- **物理引擎**: DART
- **控制**: teleop_twist_keyboard
- **车型**: 四轮差速转向（左前/左后同步，右前/右后同步）
- **赛道**: `course_test.sdf`（平直段→45°斜坡→干扰段→T字路口）

### 5.4 URDF 传感器

- **激光雷达**：CPU `lidar`（ray 类型），720 射线（可配置），10Hz，安装于 `radar_link`
- **IMU**：`imu` 类型，200Hz，安装于 `base_link`
- **里程计**：`OdometryPublisher` 系统插件，发布地面真实位姿

---

## 6. 模拟桥架环境与测试

### 6.1 赛道几何

| 区域      | X 范围 (m)     | Z 范围 (m) | 墙壁                   |
| --------- | -------------- | ---------- | ---------------------- |
| 平直段    | [0, 20]        | 0          | Y = ±0.15             |
| 45° 斜坡 | [20, 21.13]    | 0 → 1.13  | Y = ±0.15             |
| 干扰段    | [21.13, 22.58] | 1.13       | Y = ±0.15，地面含凸起 |
| T 字路口  | ~22.73         | 1.13       | Y 向展开 ±1.6m        |

### 6.2 启动方式

```bash
cd robotProj_ws && source install/setup.bash
ros2 launch autoVehicle gazebo.launch.py
```

可选：基于已保存 PGM/YAML 地图启动 Nav2（需要先运行上面的 Gazebo 仿真）：

```bash
ros2 launch autoVehicle nav_map.launch.py
```

Nav2 默认使用 `maps/course_map.yaml`。若需要重新生成这张干净栅格图：

```bash
ros2 run autoVehicle generate_course_map.py
```

`nav2_params.yaml` 已按窄走廊调整 DWB 与代价地图参数，减少 `No valid trajectories`
和 `Collision Ahead` 误报。

坡道处理：`synthetic_lidar.py` 不再把 45° 坡面作为 `/scan` 的 2D 障碍；全局代价地图
只使用静态 `course_map`，动态避障交给局部代价地图，避免上坡入口被实时扫描误判为墙。

注意：赛道改为 20m 后，旧的 PCD 地图只覆盖旧赛道范围。使用自定义航点前需要重新建图：

```bash
# 重启 Gazebo 后，遥控或通过 waypoint_cli 沿新赛道走完整条路
ros2 service call /save_map std_srvs/srv/Trigger
ros2 service call /waypoint/reload_map std_srvs/srv/Trigger
```

Nav2 使用 `course_map`，不受旧 PCD 影响。

遥控（另一终端）：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

保存地图：

```bash
ros2 service call /save_map std_srvs/srv/Trigger
# 文件：~/pointcloud_maps/map_YYYYMMDD_HHMMSS.pcd
```

### 6.3 航点自主导航

启动后 RViz 中会显示 `/waypoint_map`（自动加载 `~/pointcloud_maps` 中最新的 PCD）。使用 **2D Goal Pose** 工具在地图上依次点选航点，然后：

```bash
ros2 service call /waypoint/start std_srvs/srv/Trigger
```

也可以完全用命令行手动设立航点：

```bash
ros2 run autoVehicle waypoint_cli.py add 3.0 0.0 --yaw 0
ros2 run autoVehicle waypoint_cli.py add 22.73 1.2 --yaw 90
ros2 run autoVehicle waypoint_cli.py start
ros2 run autoVehicle waypoint_cli.py stop
ros2 run autoVehicle waypoint_cli.py status
```

可用服务：

- `/waypoint/add` — 通过 `rcl_interfaces/SetParameters` 添加 `x y z yaw_deg`
- `/waypoint/start`、`/waypoint/stop`、`/waypoint/clear` — 启停与清空
- `/waypoint/save`、`/waypoint/load` — 保存/加载 `~/waypoint_files/waypoints.json`
- `/waypoint/status` — 查询当前状态、目标与里程
- `/waypoint/reload_map` — 重新加载 `~/pointcloud_maps` 中最新的 PCD

若需要使用 Nav2 的预建图导航，可直接运行 `ros2 launch autoVehicle nav_map.launch.py`；
`nav_slam.launch.py` 仍保留为 slam_toolbox 建图入口。

### 6.4 Nav2 2D Goal Pose 导航

启动 Gazebo 和 Nav2 后，在 RViz 中选择 **2D Goal Pose** 工具，在 `course_map` 上点击目标位置并拖出朝向。Nav2 会依次执行：

1. 全局路径规划（使用静态 `course_map`）
2. 局部轨迹跟踪（使用 `/scan` 做动态避障）
3. 到达目标后输出 `Goal succeeded`

当前 `nav_map.launch.py` 未启动 `collision_monitor`，避免之前出现的参数解析错误；窄走廊和坡道参数已写入 `nav2_params.yaml`。

### 6.5 测试结果

- 扫描速率：10Hz（720 射线 + 墙面插值 + 环形地面扫描）
- 单帧点数：~400-500（含墙壁垂直填充 + 地面点阵）
- 累积速率：~4,000-5,000 点/秒
- 自动保存：每 ~50 帧触发一次
- 机器人成功遥控通过平直段→斜坡→干扰段→T字路口
- 自定义航点：服务添加航点后机器人可自动行驶，`/cmd_vel` 正常输出
- Nav2 规划：`ComputePathToPose` 在平直段和坡上均能成功返回路径
- 点云实时显示墙壁表面（3D）、地面高程（含坡面）、走廊轮廓

---

## 7. 关键技术指标满足情况

- **尺寸**：170×170×~90 mm，≤180×180×95 mm ✓
- **重量**：约 1.26 kg，≤2 kg ✓
- **爬坡能力**：电磁铁吸附安全余量充足，45° 斜坡可靠通过 ✓
- **遥控建图**：仿真环境已实现 3D 点云建图与 PCD 保存 ✓
- **里程精度**：仿真使用地面真实里程计，无漂移
- **感知与导航**：仿真 LiDAR 10Hz，PCD 航点巡航与 Nav2 预建图导航已实现 ✓

---

## 8. 相关文档

- [pointCloud.md](pointCloud.md) — 点云建图详细文档（架构、核心组件、赛道模型、Bug 修复历程）
- [navigate.md](navigate.md) — 航点导航、Nav2 配置与上坡排障整理
- [testlog.md](testlog.md) — Gazebo 仿真测试日志（摩擦调试、控制器选择等历史记录）
