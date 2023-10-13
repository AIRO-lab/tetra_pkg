#!/usr/bin/env python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Bishop Pearson

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  tetra_2dnav_prefix = get_package_share_directory('tetra_2dnav')
  cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(tetra_2dnav_prefix, 'param'))
  configuration_basename = LaunchConfiguration('configuration_basename', default='test.lua')
  rviz_config_dir = os.path.join(get_package_share_directory('tetra_2dnav'), 'rviz', 'demo_2d.rviz')

  resolution = LaunchConfiguration('resolution', default='0.05')
  publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
  
  ar_track_alvar_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'ar_track_alvar_parameter.yaml'
  )

  return LaunchDescription([
    Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='link1_broadcaster',
    arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
    output='screen'
    ),
    DeclareLaunchArgument(
      'cartographer_config_dir',
      default_value=cartographer_config_dir,
      description='Full path to config file to load'),
    DeclareLaunchArgument(
      'configuration_basename',
      default_value=configuration_basename,
      description='Name of lua file for cartographer'),
    DeclareLaunchArgument(
      'use_sim_time',
      default_value='false',
      description='Use simulation (Gazebo) clock if true'),

    Node(
      package='cartographer_ros',
      executable='cartographer_node',
      name='cartographer_node',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time}],
      arguments=['-configuration_directory', cartographer_config_dir,
                  '-configuration_basename', configuration_basename],
      remappings=[
        ('echoes', 'scan'),
        ('/odom', '/odometry/filtered')
      ]
    ),

    DeclareLaunchArgument(
      'resolution',
      default_value=resolution,
      description='Resolution of a grid cell in the published occupancy grid'),

    DeclareLaunchArgument(
      'publish_period_sec',
      default_value=publish_period_sec,
      description='OccupancyGrid publishing period'),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                        'publish_period_sec': publish_period_sec}.items(),
    ),
    Node(
      package='tetra_landmark',
      executable='tetra_landmark_save',
      name='tetra_landmark_save',
      output='screen'
    ),
    Node(
    package="ar_track_alvar",
      executable="individual_markers_no_kinect",
      name="ar_track_alvar2",
      output="screen",
      parameters=[{"output_frame": "odom"},
                  ar_track_alvar_parameter],
      remappings=[
        # ('camera_image', 'usb_cam/image_raw'),
        # ('camera_info', 'usb_cam/camera_info'),
        ('ar_pose_marker', 'map_to_marker_pose')
      ]
    ),
    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config_dir],
      parameters=[{'use_sim_time': use_sim_time}],
      output='screen'
    ),
    
  ])