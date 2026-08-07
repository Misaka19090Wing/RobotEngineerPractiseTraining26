#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('autoVehicle')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    world_name = LaunchConfiguration('world')

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [os.path.join(pkg_share, 'world'), '/',
                                       LaunchConfiguration('world', default='course_test.sdf'),
                                       ' -r']}.items(),
    )

    # Robot description
    with open(os.path.join(pkg_share, 'urdf', 'autoVehicle.urdf')) as f:
        robot_desc = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    joint_state_pub = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher', output='screen',
    )

    tf_ftb = Node(package='tf2_ros', executable='static_transform_publisher',
        name='tf_footprint_to_base',
        arguments=['--frame-id','base_footprint','--child-frame-id','base_link'])

    tf_ofb = Node(package='tf2_ros', executable='static_transform_publisher',
        name='tf_odom_fallback',
        arguments=['--frame-id','odom','--child-frame-id','base_footprint'])

    spawn = TimerAction(period=3.0, actions=[Node(
        package='ros_gz_sim', executable='create', name='spawn_autoVehicle',
        output='screen', arguments=['-topic','robot_description','-name','autoVehicle',
            '-x','0.5','-y','0.0','-z','0.06'])])

    # Bridges
    bridge_cmd_vel = Node(package='ros_gz_bridge', executable='parameter_bridge',
        name='b_cmd_vel', output='screen',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'])

    bridge_odom = Node(package='ros_gz_bridge', executable='parameter_bridge',
        name='b_odom', output='screen',
        arguments=['/model/autoVehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[('/model/autoVehicle/odometry','/odom')])

    bridge_imu = Node(package='ros_gz_bridge', executable='parameter_bridge',
        name='b_imu', output='screen',
        arguments=['/model/autoVehicle/link/base_link/sensor/imu/imu'
                   '@sensor_msgs/msg/Imu'
                   '[gz.msgs.IMU'],
        remappings=[('/model/autoVehicle/link/base_link/sensor/imu/imu','/imu')])

    bridge_joints = []
    for j in ['left_front_joint','left_rear_joint','right_front_joint','right_rear_joint']:
        bridge_joints.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
            name=f'b_{j}', output='screen',
            arguments=[f'/model/autoVehicle/joint/{j}/cmd_vel'
                       '@std_msgs/msg/Float64]gz.msgs.Double']))

    # Custom nodes
    controller = TimerAction(period=5.0, actions=[Node(
        package='autoVehicle', executable='diff_drive_controller.py',
        name='diff_drive_controller', output='screen')])

    pose_tf = TimerAction(period=4.0, actions=[Node(
        package='autoVehicle', executable='pose_tf_broadcaster.py',
        name='pose_tf_broadcaster', output='screen')])

    synth_lidar = TimerAction(period=5.0, actions=[Node(
        package='autoVehicle', executable='synthetic_lidar.py',
        name='synthetic_lidar', output='screen',
        parameters=[{'num_samples':360, 'rate_hz':10.0}])])

    mapper = TimerAction(period=6.0, actions=[Node(
        package='autoVehicle', executable='pointcloud_mapper.py',
        name='pointcloud_mapper', output='screen',
        parameters=[{'downsample_input':1, 'publish_rate_hz':2.0,
                     'save_dir':os.path.expanduser('~/pointcloud_maps')}])])

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'pointcloud_mapping.rviz')])

    ros_diag = TimerAction(period=10.0, actions=[ExecuteProcess(
        cmd=['ros2','topic','list'], output='screen')])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='course_test.sdf'),
        gz_sim, robot_state_pub, joint_state_pub, tf_ftb, tf_ofb,
        bridge_cmd_vel, bridge_odom, bridge_imu, *bridge_joints,
        spawn, controller, pose_tf, synth_lidar, rviz, ros_diag,
    ])
