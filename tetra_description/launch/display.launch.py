#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.descriptions import ParameterValue
import xacro

def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  use_gui = LaunchConfiguration('use_gui', default='true')
  urdf_file_name = 'tetraDS.xacro'
  rviz_file_name = 'rviz.rviz'
  
  urdf = os.path.join(
    get_package_share_directory('tetra_description'),
    'urdf',
    urdf_file_name
  )
  
  rviz = os.path.join(
    get_package_share_directory('tetra_description'),
    'rviz',
    rviz_file_name
  )
  
  # with open(urdf, 'r') as infp:
  #   robot_desc = infp.read()
  
  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value='false',
    ),
    DeclareLaunchArgument(
      'use_gui',
      default_value='true',
    ),
    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time,
                   'robot_description': xacro.process_file(urdf).toxml()
                   }],
      arguments=[urdf]
    ),
    Node(
      package='joint_state_publisher',
      executable='joint_state_publisher',
      name='joint_state_publisher',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time}],
      arguments=[urdf]
    )
  ])