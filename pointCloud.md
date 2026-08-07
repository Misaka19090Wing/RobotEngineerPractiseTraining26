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

| 组件 | 参数 | 说明 |
|---|---|---|
| 墙壁检测 | 360 条 2D 射线 | 对赛道几何做数学投射，包含坐标系变换（radar_link→world） |
| 斜坡面检测 | 基于 LiDAR 高度 `oz` | 45° 斜坡交点 = RAMP_X0 + oz |
| 垂直线填充 | 每墙点生成 3 层 | 地平面/10cm/20cm（墙顶），构建完整墙面 |
| 地面扫描 | 36 方位 × 10 距离步 | 0.5m~5m 环形地面点阵，验证走廊/路口边界 |

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

- **CPU `lidar`**（ray 类型）：2D 激光扫描器，360 射线，10Hz，`visualize=true`
- **IMU**（imu 类型）：200Hz，附着于 `base_link`，含高斯噪声模型
- **OdometryPublisher**：Gazebo 系统插件，发布里程计到 `/model/autoVehicle/odometry`

---

## 4. 赛道几何模型

硬编码了 `course_test.sdf` 的完整几何参数：

| 区域 | X 范围 | Z 范围 | 墙壁 |
|---|---|---|---|
| 平直段 Seg1 | [0, 3.2] | Z=0 | Y = ±0.15 |
| 45° 斜坡 Seg2 | [3.2, 4.33] | Z: 0→1.13 | Y = ±0.15；坡面会阻挡水平射线 |
| 干扰段 Seg3 | [4.33, 5.78] | Z=1.13 | Y = ±0.15 |
| T 字路口 | X≈5.93 | Z=1.13 | 左墙 X=5.78 (带缺口 \|Y\|<0.2)，右墙 X=6.08，端墙 Y=±1.65 |

**墙壁检测函数 `raycast(ox, oy, oz, angle)`** 检查 5 种交点（取最近）：
1. 走廊侧墙（Y=±0.15, X<5.78）
2. T 字路口左墙（X=5.78, 带入口缺口）
3. T 字路口右墙（X=6.08）
4. T 字路口端墙（Y=±1.65）
5. **45° 斜坡面**（Z = X-3.2，当 LiDAR 高度在坡面范围内时检测）

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

| 问题 | 原因 | 解决方案 |
|---|---|---|
| Gazebo LiDAR 传感器不发布数据 | VMware 无 3D 加速 / URDF sensor 不被 Gazebo 加载 | 使用数学射线投射替代物理传感器 |
| ROS 2 DDS 跨进程通信失败 | 虚拟环境 DDS 发现不稳定 | 单节点集成扫描+建图 |
| 点云形成"圆环" / Y 方向偏移 | TF 查询使用 `rclpy.time.Time()`(时间0) | 改用 `self.get_clock().now()` |
| 地面点超出墙壁 | 地面网格未验证走廊边界 | 添加边界检查 `abs(wy) > HW` |
| 30m 垃圾点（极值噪声） | 未命中射线设为 RNG_MAX | 改为 `float('nan')`，被 mapper 自动跳过 |
| 射线穿透 45° 斜坡 | raycast 只检测垂直墙壁 | 新增斜坡面交点检测 |
| 机器人旋转后点云偏移 | 射线角度在 radar_link 帧但 raycast 假设世界帧 | 射线方向先做旋转矩阵变换再传入 raycast |
| 墙面只显示一条线 | 2D LiDAR 物理限制 | 垂直线填充：每墙点生成地面/10cm/20cm 三层 |
| IMU 传感器 SDF 警告 | URDF→SDF 转换中 noise 属性命名差异 | 无功能影响，可忽略 |

---

## 7. 运行效果

- 扫描速率：10Hz（360 射线墙壁 + 36×10 环形地面）
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
