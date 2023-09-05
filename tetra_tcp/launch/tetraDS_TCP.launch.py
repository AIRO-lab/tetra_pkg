#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  port = LaunchConfiguration("port")
  port_arg = DeclareLaunchArgument(
    'port',
    default_value=5100
  )
  return LaunchDescription([
    port_arg,
    Node(
      package="tetra_tcp",
      executable="tetra_tcp",
      name="tetra_tcp",
      output="screen",
      emulate_tty=True,
      parameters=[
        {"port":port}
      ]
    )
  ])