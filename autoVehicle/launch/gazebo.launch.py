#!/usr/bin/env python3
"""
Launch Gazebo Harmonic (gz-sim) with the test course world,
spawn autoVehicle via robot_description topic, and bridge
cmd_vel / odom / TF.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autoVehicle')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # world file
    world_file = os.path.join(pkg_share, 'world', 'course_test.sdf')

    # ----- Gazebo Harmonic -----
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_file + ' -r'}.items(),
    )

    # ----- robot_description -----
    urdf_file = os.path.join(pkg_share, 'urdf', 'autoVehicle.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # ----- spawn robot -----
    # ros_gz_sim create reads from /robot_description topic
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_autoVehicle',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'autoVehicle',
            '-x', '0.5',
            '-y', '0.0',
            '-z', '0.15',
        ],
    )

    # ----- bridges: cmd_vel, odom, joint_states -----
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_vel',
        output='screen',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        output='screen',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
    )

    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_joint_states',
        output='screen',
        arguments=['/world/course_test/model/autoVehicle/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'],
    )

    # TF: base_link → base_footprint
    tf_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # spawn after Gazebo is running
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    return LaunchDescription([
        gz_sim,
        robot_state_pub,
        tf_base_footprint,
        bridge_cmd_vel,
        bridge_odom,
        bridge_joint_states,
        delayed_spawn,
    ])
