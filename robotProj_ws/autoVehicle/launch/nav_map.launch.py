#!/usr/bin/env python3
"""
Navigation with pre-built map.
Loads the map, publishes map->odom static TF (ground truth),
and starts Nav2 nodes WITHOUT collision_monitor.

Usage:
  ros2 launch autoVehicle nav_map.launch.py map:=/path/to/map.yaml
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autoVehicle')
    params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = LaunchConfiguration('map')

    default_map = os.path.join(
        pkg_share, 'maps', 'course_map.yaml')

    tf_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_map_odom',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[params, {'yaml_filename': map_file}],
    )

    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen', parameters=[params])

    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen', parameters=[params])

    behavior = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen', parameters=[params])

    bt_nav = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen', parameters=[params])

    waypoint = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen', parameters=[params])

    lifecycle = TimerAction(period=3.0, actions=[Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': False, 'autostart': True,
            'node_names': [
                'map_server', 'planner_server', 'controller_server',
                'behavior_server', 'bt_navigator', 'waypoint_follower',
            ],
        }],
    )])

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        tf_map_odom, map_server,
        planner, controller, behavior, bt_nav, waypoint,
        lifecycle,
    ])
