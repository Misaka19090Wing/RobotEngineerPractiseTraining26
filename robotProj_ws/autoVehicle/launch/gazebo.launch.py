#!/usr/bin/env python3
"""
Launch Gazebo Harmonic + autoVehicle robot + LiDAR point cloud mapping + RViz2.

TF tree:
  odom → base_footprint → base_link → (wheels, radar_link)
          ^                ^             ^
     broadcaster +    static TF     robot_state_publisher
     Gazebo Odometry        + joint_state_publisher
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('autoVehicle')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_arg = DeclareLaunchArgument('world', default_value='course_test.sdf')
    world_name = LaunchConfiguration('world')

    # ── Gazebo ──────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [os.path.join(pkg_share, 'world'), '/', world_name, ' -r'],
        }.items(),
    )

    # ── Robot description ───────────────────────────────────────────
    urdf_file = os.path.join(pkg_share, 'urdf', 'autoVehicle.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # ── Joint state publisher ───────────────────────────────────────
    # Required! robot_state_publisher needs joint_states to publish
    # transforms for continuous joints (four wheels).
    joint_state_pub = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher', output='screen',
    )

    # ── TF: base_footprint → base_link (static identity) ────────────
    tf_footprint_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_footprint_to_base',
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
    )

    # ── TF: odom → base_footprint (static identity fallback) ────────
    #  Ensures the odom frame always exists, even before odometry arrives.
    #  The pose_tf_broadcaster will override this once live data flows.
    tf_odom_fallback = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_odom_fallback',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
    )

    # ── Spawn robot into Gazebo ─────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        name='spawn_autoVehicle', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'autoVehicle',
                   '-x', '0.5', '-y', '0.0', '-z', '0.06'],
    )

    # ── Bridges ────────────────────────────────────────────────────
    # cmd_vel (teleop → Gazebo) — bidirectional
    bridge_cmd_vel = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='bridge_cmd_vel', output='screen',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
    )

    # Odometry bridge (Gazebo → ROS /odom)
    bridge_odom = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='bridge_odom', output='screen',
        arguments=['/model/autoVehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[('/model/autoVehicle/odometry', '/odom')],
    )

    # LiDAR LaserScan bridge (Gazebo → ROS /scan)
    # LiDAR PointCloud2 bridge (Gazebo → ROS /lidar_points)
    # Diagnostic: list ROS topics 10s after startup
    ros_topic_diag = ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],
        output='screen',
    )

    # Joint velocity bridges (ROS → Gazebo)
    joints = ['left_front_joint', 'left_rear_joint',
              'right_front_joint', 'right_rear_joint']
    bridge_joints = []
    for j in joints:
        b = Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'bridge_{j}_vel', output='screen',
            arguments=[
                f'/model/autoVehicle/joint/{j}/cmd_vel'
                '@std_msgs/msg/Float64]gz.msgs.Double'
            ],
        )
        bridge_joints.append(b)

    # ── Custom nodes ──────────────────────────────────────────────
    controller = Node(
        package='autoVehicle', executable='diff_drive_controller.py',
        name='diff_drive_controller', output='screen',
    )

    # Broadcasts odom→base_footprint TF from /odom (or identity fallback)
    pose_tf = Node(
        package='autoVehicle', executable='pose_tf_broadcaster.py',
        name='pose_tf_broadcaster', output='screen',
    )

    # Synthetic LiDAR — bypasses Gazebo sensor (VMware has no 3D)
    # 3D point cloud mapper: accumulate, publish /pointcloud_map, save PCD

    # Synthetic LiDAR — bypasses Gazebo URDF sensor limitation
    synthetic_lidar = Node(
        package='autoVehicle', executable='synthetic_lidar.py',
        name='synthetic_lidar', output='screen',
        parameters=[{'num_samples': 360, 'rate_hz': 10.0}],
    )

    pointcloud_mapper = Node(
        package='autoVehicle', executable='pointcloud_mapper.py',
        name='pointcloud_mapper', output='screen',
        parameters=[{
            'downsample_input': 1,
            'publish_rate_hz': 2.0,
            'save_dir': os.path.expanduser('~/pointcloud_maps'),
        }],
    )

    # ── RViz2 ──────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_share, 'config', 'pointcloud_mapping.rviz')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen', arguments=['-d', rviz_config],
    )

    # ── Timing ────────────────────────────────────────────────────
    delayed_spawn    = TimerAction(period=3.0, actions=[spawn_robot])
    delayed_pose_tf  = TimerAction(period=4.0, actions=[pose_tf])
    delayed_synth   = TimerAction(period=5.0, actions=[synthetic_lidar])
    delayed_ctrl     = TimerAction(period=5.0, actions=[controller])
    delayed_mapper   = TimerAction(period=6.0, actions=[pointcloud_mapper])

    return LaunchDescription([
        world_arg,
        gz_sim,
        robot_state_pub,
        joint_state_pub,
        tf_footprint_to_base,
        tf_odom_fallback,
        bridge_cmd_vel,
        bridge_odom,
        *bridge_joints,
        delayed_spawn,
        delayed_pose_tf,
        delayed_synth,
        delayed_ctrl,
        delayed_mapper,
        TimerAction(period=10.0, actions=[ros_topic_diag]),
        rviz,
    ])
