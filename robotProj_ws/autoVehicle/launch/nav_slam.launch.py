#!/usr/bin/env python3
"""SLAM-only launch - builds an occupancy grid map via slam_toolbox."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autoVehicle')

    slam_toolbox = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[{
            'mode': 'mapping',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'resolution': 0.02,
            'max_laser_range': 30.0,
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            'enable_interactive_mode': True,
        }],
    )

    return LaunchDescription([slam_toolbox])
