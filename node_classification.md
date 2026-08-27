# ROS2 节点分类说明

根据当前项目的 `ros2 node list` 输出，按功能对节点进行分类。

## 1. Gazebo / ROS 桥接节点

| 节点 | 作用 |
|---|---|
| `/b_cmd_vel` | 桥接 `/cmd_vel` |
| `/b_odom` | 桥接 Gazebo 里程计到 `/odom` |
| `/b_imu` | 桥接 Gazebo IMU 到 `/imu` |
| `/b_left_front_joint` | 桥接左前轮关节速度 |
| `/b_left_rear_joint` | 桥接左后轮关节速度 |
| `/b_right_front_joint` | 桥接右前轮关节速度 |
| `/b_right_rear_joint` | 桥接右后轮关节速度 |

## 2. 机器人模型 / TF

| 节点 | 作用 |
|---|---|
| `/robot_state_publisher` | 发布 URDF 描述和内部静态 TF |
| `/joint_state_publisher` | 发布关节状态 `/joint_states` |
| `/pose_tf_broadcaster` | 发布动态 `odom → base_footprint` |
| `/tf_footprint_to_base` | 静态 TF `base_footprint → base_link` |
| `/tf_map_odom` | 静态 TF `map → odom` |
| `/tf_odom_fallback` | 备用静态 TF `odom → base_footprint` |

## 3. 感知 / 建图

| 节点 | 作用 |
|---|---|
| `/synthetic_lidar` | 合成 LiDAR、点云累积、地图发布、PCD 保存 |

## 4. 运动控制 / 遥控

| 节点 | 作用 |
|---|---|
| `/diff_drive_controller` | 订阅 `/cmd_vel`，计算四轮速度并发布到 Gazebo |
| `/teleop_twist_keyboard` | 键盘遥控，发布 `/cmd_vel` |

## 5. 自定义航点导航

| 节点 | 作用 |
|---|---|
| `/waypoint_navigator` | PCD 地图加载、A* 路径规划、航点队列、速度控制、避障 |

## 6. Nav2 核心节点

| 节点 | 作用 |
|---|---|
| `/map_server` | 加载并发布静态栅格地图 `/map` |
| `/planner_server` | 全局路径规划（NavfnPlanner） |
| `/controller_server` | 局部路径跟踪（DWB） |
| `/behavior_server` | 行为恢复：spin / backup / wait |
| `/bt_navigator` | 行为树导航，执行整个导航任务 |
| `/waypoint_follower` | Nav2 航点任务执行 |
| `/lifecycle_manager_navigation` | 管理 Nav2 节点生命周期 |

## 7. Nav2 代价地图节点

| 节点 | 作用 |
|---|---|
| `/global_costmap/global_costmap` | 全局代价地图 |
| `/local_costmap/local_costmap` | 局部代价地图 |

## 8. Nav2 行为树内部节点

| 节点 | 作用 |
|---|---|
| `/bt_navigator_navigate_to_pose_rclcpp_node` | `navigate_to_pose` 行为树内部线程节点 |
| `/bt_navigator_navigate_through_poses_rclcpp_node` | `navigate_through_poses` 行为树内部线程节点 |

## 9. 可视化

| 节点 | 作用 |
|---|---|
| `/rviz2` | RViz2 可视化界面 |

## 10. 内部监听节点

| 节点 | 作用 |
|---|---|
| `/transform_listener_impl_*` | TF2 内部 TransformListener 实现节点，一般无需关注 |

## 一句话总结

```text
桥接：/b_*
模型/TF：robot_state_publisher / pose_tf_broadcaster / static TF
感知建图：synthetic_lidar
控制：diff_drive_controller / teleop_twist_keyboard
自定义航点：waypoint_navigator
Nav2：map_server / planner_server / controller_server / behavior_server / bt_navigator / waypoint_follower
代价地图：global_costmap / local_costmap
生命周期：lifecycle_manager_navigation
可视化：rviz2
```
