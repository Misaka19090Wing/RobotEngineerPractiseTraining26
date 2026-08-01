#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('autoVehicle')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_arg = DeclareLaunchArgument('world', default_value='test_flat.sdf')
    world_name = LaunchConfiguration('world')

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [os.path.join(pkg_share, 'world'), '/', world_name, ' -r']}.items(),
    )

    # Robot description
    urdf_file = os.path.join(pkg_share, 'urdf', 'autoVehicle.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # Spawn
    spawn_robot = Node(
        package='ros_gz_sim', executable='create', name='spawn_autoVehicle',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'autoVehicle',
                   '-x', '0.5', '-y', '0.0', '-z', '0.06'],
    )

    # Bridges
    bridge_cmd_vel = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='bridge_cmd_vel',
        output='screen', arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
    )

    bridge_odom = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='bridge_odom',
        output='screen', arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
    )

    # Joint velocity bridges (ROS → Gazebo)
    joints = ['left_front_joint', 'left_rear_joint', 'right_front_joint', 'right_rear_joint']
    bridge_joints = []
    for j in joints:
        b = Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'bridge_{j}_vel',
            output='screen',
            arguments=[f'/model/autoVehicle/joint/{j}/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double'],
        )
        bridge_joints.append(b)

    # Custom diff drive controller
    controller = Node(
        package='autoVehicle',
        executable='diff_drive_controller.py',
        name='diff_drive_controller',
        output='screen',
    )

    tf_base = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_footprint_base',
        arguments=['0','0','0','0','0','0','base_link','base_footprint'],
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])
    delayed_controller = TimerAction(period=5.0, actions=[controller])

    return LaunchDescription([
        world_arg, gz_sim, robot_state_pub, tf_base,
        bridge_cmd_vel, bridge_odom,
        *bridge_joints,
        delayed_spawn, delayed_controller,
    ])
