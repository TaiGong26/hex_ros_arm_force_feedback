#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    force_feedback_pkg_path = FindPackageShare('hex_ros_arm_force_feedback')
    urdf_pkg_path = FindPackageShare('hex_ros_urdf_archer_y6')

    # arm_force_feedback node
    force_feedback_param_path = PathJoinSubstitution(
        [force_feedback_pkg_path, "config", "ros2", "params.yaml"])
    urdf_file_path = PathJoinSubstitution(
        [urdf_pkg_path, "urdf", "gr100_comp.urdf"])

    arm_force_feedback_node = Node(
        package='hex_ros_arm_force_feedback',
        executable='arm_force_feedback',
        name='arm_force_feedback',
        output="screen",
        emulate_tty=True,
        parameters=[
            force_feedback_param_path,
            {
                "model_urdf": ParameterValue(urdf_file_path, value_type=str),
                "use_sim_time": True,
            },
        ],
        remappings=[
            ('master/manip_state', 'master/manip_state'),
            ('master/manip_ctrl', 'master/manip_ctrl'),
            ('slave/manip_state', 'slave/manip_state'),
            ('slave/manip_ctrl', 'slave/manip_ctrl'),
            ('teleop_keyboard_state', 'teleop_keyboard_state'),
        ],
    )

    return LaunchDescription([
        arm_force_feedback_node,
    ])
