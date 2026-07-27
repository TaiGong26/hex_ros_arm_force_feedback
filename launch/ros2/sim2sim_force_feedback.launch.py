#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace  # 修改这里：PushRosNamespace（Ros 不是 ROS）


def generate_launch_description():
    sim_pkg_path = FindPackageShare('hex_ros_sim_archer_y6')
    keyboard_pkg_path = FindPackageShare('hex_ros_teleop_keyboard')
    force_feedback_pkg_path = FindPackageShare('hex_ros_demo_arm_force_feedback')

    # args
    viewer_arg = DeclareLaunchArgument(
        name='viewer',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to turn on mujoco viewer')
    rviz_arg = DeclareLaunchArgument(name='rviz',
                                     default_value='false',
                                     choices=['true', 'false'],
                                     description='Flag to turn on rviz')

    # sim environment (mujoco + rviz, no built-in test ctrl)
    sim_master_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_pkg_path, "sim_archer_y6.launch.py"])),
        launch_arguments={
            'viewer': LaunchConfiguration('viewer'),
            'rviz': LaunchConfiguration('rviz'),
            'test': 'false',
        }.items(),
    )
    
    sim_slave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_pkg_path, "sim_archer_y6.launch.py"])),
        launch_arguments={
            'viewer': LaunchConfiguration('viewer'),
            'rviz': LaunchConfiguration('rviz'),
            'test': 'false',
        }.items(),
    )
    
    master_group = GroupAction([
        PushRosNamespace('master'),  
        sim_master_launch,
    ])
    
    slave_group = GroupAction([
        PushRosNamespace('slave'),  
        sim_slave_launch,
    ])

    # keyboard teleop
    keyboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [keyboard_pkg_path, "teleop_keyboard.launch.py"])), )

    # force_feedback control node
    force_feedback_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [force_feedback_pkg_path, "arm_force_feedback.launch.py"])), )

    return LaunchDescription([
        viewer_arg,
        rviz_arg,
        master_group,
        slave_group,
        keyboard_launch,
        force_feedback_launch,
    ])