#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  joystick_node = Node(
    package="joy",
    executable="joy_node",
    name="joy_node",
    output="screen",
    parameters=[
      {"dev":"dev/input/js0"}
    ]
  )
  joystick_2_cmd = Node(
    package="tetra_joystick",
    executable="joystick",
    name="joystick",
    output="screen",
  )
  return LaunchDescription([
    joystick_node,
    joystick_2_cmd
  ])