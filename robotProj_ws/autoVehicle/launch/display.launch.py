#!/usr/bin/env python3
"""
Launch RViz2 with joint_state_publisher_gui for manual joint control.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autoVehicle')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(pkg_share, 'urdf', 'autoVehicle.urdf'),
        description='Path to the URDF model file',
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Use joint_state_publisher_gui',
    )

    urdf_path = LaunchConfiguration('model')

    # read URDF at launch time
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_path}],
    )

    joint_state_pub = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        gui_arg,
        robot_state_pub,
        joint_state_pub,
        rviz,
    ])
