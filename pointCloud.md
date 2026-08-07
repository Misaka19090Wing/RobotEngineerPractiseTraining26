# 3D 点云地图构建与保存

基于 ROS2 Jazzy + Gazebo Harmonic + RViz2 的遥控控制 3D 点云地图构建系统。

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

由于 Gazebo Harmonic 在 VMware 环境下无法正确加载 URDF-based `<gazebo><sensor>` 元素（传感器不发布数据），且 ROS 2 DDS 跨进程通信在虚拟环境中不稳定，最终采用**单节点集成方案**：

- **墙壁检测**：360 条 2D 射线对赛道几何模型做数学投射，检测走廊墙壁（Y=±0.15）、T 字路口墙壁、端墙
- **地面扫描**：24 方位 × 8 距离步（0.5m~4m）的环形地面点阵，经验证仅保留走廊/路口内的点
- **TF 变换**：通过 `tf2_ros` 将扫描点从 `radar_link` 坐标系变换到 `odom` 坐标系
- **点云累积**：直接在进程内累积所有历史点云到 `map_points` 列表
- **地图发布**：每 0.5 秒发布 `/pointcloud_map`（下采样至 200K 上限）
- **自动保存**：每 500K 点触发一次 PCD 文件保存（`~/pointcloud_maps/map_*.pcd`）

### 3.2 pose_tf_broadcaster.py

订阅 `/odom`（Gazebo 里程计）和 `/model/autoVehicle/pose`（Gazebo 模型位姿），发布动态 `odom → base_footprint` TF 变换。包含 10Hz identity 回退定时器，确保 `odom` 帧始终存在。

### 3.3 diff_drive_controller.py

订阅 `/cmd_vel`（teleop），计算四轮差速目标转速，发布到 Gazebo 关节速度话题。

### 3.4 URDF 传感器

- **CPU `lidar`**（ray 类型）：2D 激光扫描器，360 射线，10Hz
- **IMU**（imu 类型）：200Hz，附着于 `base_link`
- **OdometryPublisher**：Gazebo 系统插件，发布里程计到 `/model/autoVehicle/odometry`

---

## 4. 赛道几何模型

代码中硬编码了 `course_test.sdf` 的几何参数：

| 区域 | X 范围 | Z 范围 | 墙壁 Y |
|---|---|---|---|
| 平直段 Seg1 | [0, 3.2] | Z=0 | ±0.15 |
| 45° 斜坡 Seg2 | [3.2, 4.33] | Z: 0→1.13 | ±0.15 |
| 干扰段 Seg3 | [4.33, 5.78] | Z=1.13 | ±0.15 |
| T 字路口 | X≈5.93 | Z=1.13 | 左墙 X=5.78 (带缺口 Y±0.2)，右墙 X=6.08，端墙 Y=±1.65 |

墙壁检测函数 `raycast()` 对四个墙壁集合做射线-线段求交，取最近命中。

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

---

## 6. 已知问题与解决方案

| 问题 | 原因 | 解决方案 |
|---|---|---|
| Gazebo LiDAR 传感器不发布数据 | VMware 无 3D 加速 / URDF sensor 不被 Gazebo 加载 | 使用数学射线投射替代物理传感器 |
| ROS 2 DDS 跨进程通信失败 | 虚拟环境 DDS 发现不稳定 | 单节点集成扫描+建图，消除跨进程依赖 |
| 点云形成"圆环" | TF 使用 `rclpy.time.Time()`(时间0) 而非当前时钟 | 改用 `self.get_clock().now()` |
| Y 方向偏移 | TF 时间戳不匹配 | 同上修复 |
| 地面点超出墙壁 | 地面网格未验证走廊边界 | 添加 `abs(wy) > HW` 边界检查 |
| 无回波射线产生 30m 垃圾点 | 未命中射线设为 RNG_MAX | 改为 `float('nan')`（mapper 跳过） |
| IMU 传感器 SDF 警告 | URDF→SDF 转换中 noise 属性命名差异 | 无功能影响，可忽略 |

---

## 7. 运行效果

- 扫描速率：10Hz（360 射线墙壁 + 环形地面）
- 点云增长率：约 350 点/帧（墙壁+地面）
- 500 帧后累积约 18 万点
- PCD 自动保存间隔：每 50 帧
- 机器人从起点(0.5, 0, 0.06) 可遥控行驶通过平直段→45°斜坡→干扰段→T 字路口
- 点云地图实时反映走廊墙壁轮廓和地面高程变化

---

## 8. 关键 Bug 修复历程

1. **TF 树方向错误**：`base_link → base_footprint` 反了，修正为 `base_footprint → base_link`
2. **缺少 joint_state_publisher**：`robot_state_publisher` 需要 joint_states 推送才能发布轮子 TF
3. **Header 类型错误**：`rclpy.time.Time().to_msg()` 返回 `Time` 不是 `Header`，改为 `std_msgs.msg.Header()`
4. **`gpu_lidar` 需要 OGRE2 渲染引擎**：在 VMware 中不可用，切换到 CPU `lidar` 传感器
5. **`raycast()` 缩进 Bug**：`ix`/`iy` 变量在条件语句外被引用，导致 `UnboundLocalError`
6. **桥接配置多次迭代**：从命令行参数到 YAML 再到命令行参数，最终因为 Gazebo 传感器不工作而移除所有 LiDAR 桥接
