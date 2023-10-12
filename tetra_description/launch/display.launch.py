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
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  urdf_file_name = 'tetraDS.xacro'
  
  urdf = os.path.join(
    get_package_share_directory('tetra_description'),
    'urdf',
    urdf_file_name
  )
  
  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value='false',
    ),
    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time,
                   'robot_description': xacro.process_file(urdf).toprettyxml(indent='  ')
                   }]
    )
  ])