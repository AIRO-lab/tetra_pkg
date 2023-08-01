#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  tf_prefix = LaunchConfiguration("tf_prefix")
  ekf_option = LaunchConfiguration("ekf_option")
  tf_prefix_arg = DeclareLaunchArgument(
    'tf_prefix',
    default_value="none"
  )
  ekf_option_arg = DeclareLaunchArgument(
    'ekf_option',
    default_value="False"
  )
  return LaunchDescription([
    tf_prefix_arg,
    ekf_option_arg,
    Node(
      package="joy",
      executable="joy_node",
      name="joy_node"
    ),
    Node(
      package="tetra",
      executable="tetra",
      name="tetra",
      output="screen",
      emulate_tty=True,
      parameters=[
        {"tf_prefix":tf_prefix,
        "ekf_option":ekf_option}
      ]
    )
  ])