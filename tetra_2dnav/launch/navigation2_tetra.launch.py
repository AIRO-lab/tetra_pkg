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
  map_name = LaunchConfiguration("map_name")
  map_name_arg = DeclareLaunchArgument(
    'map_name',
    default_value='machining_center'
  )
  use_namespace = LaunchConfiguration("use_namespace")
  use_namespace_arg = DeclareLaunchArgument(
    'use_namespace',
    default_value='True'
  )
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  map_dir = LaunchConfiguration(
    'map',
    default=os.path.join(
      get_package_share_directory('tetra_2dnav'),
      'maps',
      str(map_name)+'.yaml'))

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
    executable="tetra_landmark_load",
    name="tetra_landmark_load",
    output="screen",
  )

  rviz_config_dir = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'rviz',
    'view_navigation.rviz')
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
        SetRemap(src='scan',dst='/scan'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file_dir + '/bringup_launch.py'),
            launch_arguments={
              'map': map_dir,
              'use_namespace': use_namespace,
              'use_sim_time': use_sim_time,
              'params_file': param_dir}.items(),
        )
    ]
  )
  return LaunchDescription([
    use_namespace_arg,
    map_name_arg,
    DeclareLaunchArgument(
      'map',
      default_value=map_dir,
      description='Full path to map file to load'),

    DeclareLaunchArgument(
      'params_file',
      default_value=param_dir,
      description='Full path to param file to load'),

    DeclareLaunchArgument(
      'use_sim_time',
      default_value='false',
      description='Use simulation (Gazebo) clock if true'),

    nav_include,
    tetra_landmark_node,
    rviz2_node
  ])