#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.actions import Node
def generate_launch_description():
  tf_prefix = LaunchConfiguration("tf_prefix")
  tf_prefix_arg = DeclareLaunchArgument(
    'tf_prefix',
    default_value=os.getenv('ROS_NAMESPACE')
  )

  rs_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'rs_params.yaml'
  )
  usb_cam_node = Node(
    package="realsense2_camera",
    executable="realsense2_camera_node",
    namespace=tf_prefix,
    name="realsense2_camera",
    output="screen",
    arguments=['--ros-args', '--log-level', 'info'],
    emulate_tty=True,
    parameters=[{"tf_prefix": tf_prefix},
                rs_parameter]
  )
  
  return LaunchDescription([
      tf_prefix_arg,
      usb_cam_node
    ])