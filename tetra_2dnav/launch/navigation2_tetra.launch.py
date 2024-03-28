#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch.actions import GroupAction


def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  map_name = LaunchConfiguration('map_name', default='machining_center')
  map_dir = LaunchConfiguration(
    'map',
    default=os.path.join(
      get_package_share_directory('tetra_2dnav'),
      'maps', 'mc.yaml')
    )

  param_file_name = 'tetra_nav.yaml'
  param_dir = LaunchConfiguration(
    'params_file',
    default=os.path.join(
      get_package_share_directory('tetra_2dnav'),
      'param',
      param_file_name))

  nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
  
  tetra_landmark_node = Node(
    package="tetra_landmark",
    executable="tetra_landmark",
    name="tetra_landmark",
    output="screen",
  )

  rviz_config_dir = os.path.join(
    get_package_share_directory('nav2_bringup'),
    'rviz',
    'nav2_default_view.rviz')
  rviz2_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_dir],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
  )
  nav_include = GroupAction(
    actions=[
        SetRemap(src='/odom',dst='/odometry/filtered'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file_dir + '/bringup_launch.py'),
            launch_arguments={
              'map': map_dir,
              'use_sim_time': use_sim_time,
              'params_file': param_dir}.items(),
        )
    ]
  )
  
  laser_node = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'map_name',
      default_value='MCmap',
      description='Full path to map file to load'),
    
    DeclareLaunchArgument(
      'map',
      default_value=[os.path.join(
      get_package_share_directory('tetra_2dnav'),
      'maps'), '/', map_name, '.yaml'],
      description='Full path to map file to load'),

    DeclareLaunchArgument(
      'params_file',
      default_value=param_dir,
      description='Full path to param file to load'),

    DeclareLaunchArgument(
      'use_sim_time',
      default_value='false',
      description='Use simulation (Gazebo) clock if true'),
    laser_node,
    nav_include,
    tetra_landmark_node,
    rviz2_node
  ])