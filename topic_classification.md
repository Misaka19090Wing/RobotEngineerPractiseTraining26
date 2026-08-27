# ROS2 话题分类说明

这份文档根据当前项目的 `ros2 topic list` 输出，按功能对话题进行分类。

> 注意：`ros2 topic list` 列出的是**话题**，不是节点。节点需要通过 `ros2 node list` 查看。

## 1. 传感器 / 感知

| 话题 | 作用 | 来源 |
|---|---|---|
| `/scan` | 2D 激光扫描 | `synthetic_lidar` |
| `/lidar_points` | 地面点阵点云 | `synthetic_lidar` |
| `/pointcloud_map` | 累积 3D 点云地图 | `synthetic_lidar` |
| `/imu` | IMU 数据 | Gazebo IMU bridge |
| `/cost_cloud` | 代价点云（障碍物投影） | Nav2 costmap |

## 2. 里程计 / 位姿 / TF

| 话题 | 作用 | 来源 |
|---|---|---|
| `/odom` | 机器人里程计 | Gazebo → `ros_gz_bridge` |
| `/model/autoVehicle/pose` | Gazebo 模型位姿 | Gazebo |
| `/tf` | 动态 TF | `pose_tf_broadcaster` |
| `/tf_static` | 静态 TF | `robot_state_publisher` 等 |
| `/joint_states` | 关节状态 | `joint_state_publisher` |
| `/robot_description` | URDF 描述 | `robot_state_publisher` |

## 3. 运动控制

| 话题 | 作用 | 来源/去向 |
|---|---|---|
| `/cmd_vel` | 速度指令 | 控制器/导航 → `diff_drive_controller` |
| `/model/autoVehicle/joint/*/cmd_vel` | 四个轮子关节速度 | `diff_drive_controller` → Gazebo |
| `/speed_limit` | 速度限制 | Nav2 velocity smoother |
| `/controller_selector` | 局部控制器选择 | Nav2 BT |
| `/planner_selector` | 全局规划器选择 | Nav2 BT |

## 4. 建图 / 地图

| 话题 | 作用 | 来源 |
|---|---|---|
| `/map` | Nav2 静态栅格地图 | `map_server` |
| `/waypoint_map` | PCD 点云地图显示 | `waypoint_navigator` |
| `/pointcloud_map` | 实时累积点云地图 | `synthetic_lidar` |

## 5. 全局代价地图

| 话题 | 作用 |
|---|---|
| `/global_costmap/costmap` | 全局代价地图 |
| `/global_costmap/costmap_raw` | 未膨胀代价地图 |
| `/global_costmap/static_layer` | 静态地图层 |
| `/global_costmap/static_layer_raw` | 静态层原始数据 |
| `/global_costmap/footprint` | 机器人足迹 |
| `/global_costmap/published_footprint` | 发布足迹 |
| `/global_costmap/global_costmap/transition_event` | lifecycle 状态事件 |

## 6. 局部代价地图

| 话题 | 作用 |
|---|---|
| `/local_costmap/costmap` | 局部代价地图 |
| `/local_costmap/costmap_raw` | 未膨胀局部代价地图 |
| `/local_costmap/obstacle_layer` | `/scan` 实时障碍层 |
| `/local_costmap/obstacle_layer_raw` | 障碍层原始数据 |
| `/local_costmap/footprint` | 机器人足迹 |
| `/local_costmap/published_footprint` | 发布足迹 |
| `/local_costmap/local_costmap/transition_event` | lifecycle 状态事件 |

## 7. 路径规划

| 话题 | 作用 |
|---|---|
| `/plan` | Nav2 全局规划结果 |
| `/received_global_plan` | 接收到的全局路径 |
| `/transformed_global_plan` | 转换坐标系后的全局路径 |
| `/local_plan` | 局部轨迹 |
| `/goal_pose` | RViz 2D Goal Pose 目标点 |
| `/waypoint_path` | 自定义航点剩余路径 |
| `/waypoint_markers` | 自定义航点 Marker 可视化 |

## 8. 行为树 / 调试

| 话题 | 作用 |
|---|---|
| `/behavior_tree_log` | 行为树执行日志 |
| `/evaluation` | 行为树节点评估 |
| `/marker` | Nav2 可视化 Marker |
| `/diagnostics` | 诊断信息 |
| `/bond` | Nav2 生命周期连接状态 |
| `/rosout` | ROS2 日志 |
| `/parameter_events` | 参数变更事件 |

## 9. Lifecycle 状态事件

这些话题属于 Nav2 生命周期管理，表示各节点是否处于：

```text
unconfigured / inactive / active
```

```text
/behavior_server/transition_event
/bt_navigator/transition_event
/controller_server/transition_event
/planner_server/transition_event
/map_server/transition_event
/waypoint_follower/transition_event
```

## 一句话总结

```text
感知话题：scan / lidar_points / pointcloud_map / imu
定位话题：odom / pose / tf / tf_static
控制话题：cmd_vel / joint cmd_vel
地图话题：map / pointcloud_map / waypoint_map
Nav2 内部：global_costmap / local_costmap / plan / local_plan / transition_event
可视化话题：waypoint_markers / waypoint_path / marker / cost_cloud
```
