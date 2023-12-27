#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  ekf_option = LaunchConfiguration("ekf_option")
  ekf_option_arg = DeclareLaunchArgument(
    'ekf_option',
    default_value="False"
  )
  tetra_node = Node(
    package="tetra",
    executable="tetra",
    name="tetra",
    output="screen",
    parameters=[
      {"ekf_option":ekf_option}
    ]
  )
  return LaunchDescription([
    ekf_option_arg,
    tetra_node
  ])