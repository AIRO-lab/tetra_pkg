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
  depthimage_to_laserscan1_node = Node(
    package="depthimage_to_laserscan",
    executable="depthimage_to_laserscan_node",
    name="depthimage_to_laserscan1",
    output="screen",
    remappings=[("depth","/"+os.getenv('ROS_NAMESPACE')+"/camera1/aligned_depth_to_color/image_raw"),
                ("scan","pcl_1")],
    parameters=[{"output_frame": os.getenv('ROS_NAMESPACE')+"/camera1_link"},
                {"range_min": 0.01},
                {"range_max": 2.5}]
  )
  
  depthimage_to_laserscan2_node = Node(
    package="depthimage_to_laserscan",
    executable="depthimage_to_laserscan_node",
    name="depthimage_to_laserscan2",
    output="screen",
    remappings=[("depth","/"+os.getenv('ROS_NAMESPACE')+"/camera2/aligned_depth_to_color/image_raw"),
                ("scan","pcl_2")],
    parameters=[{"output_frame": os.getenv('ROS_NAMESPACE')+"/camera2_link"},
                {"range_min": 0.01},
                {"range_max": 2.5}]
  )
  
  return LaunchDescription([
    depthimage_to_laserscan1_node,
    depthimage_to_laserscan2_node
	])