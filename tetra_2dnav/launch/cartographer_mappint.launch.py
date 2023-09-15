#!/usr/bin/env python3

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
  tf_prefix = LaunchConfiguration("tf_prefix")
  tf_prefix_arg = DeclareLaunchArgument(
    'tf_prefix',
    default_value=os.getenv('ROS_NAMESPACE')
  )
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  resoltion = LaunchConfiguration('resolution', default='0.05')
  tetra_2dnav_perfix = get_package_share_directory('tetra_2dnav')
  cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  tetra_2dnav_perfix, 'config'))
  configuration_basename = LaunchConfiguration('configuration_basename',
                                                  default='backpack_2d.lua')
  publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
  rviz_config_dir = os.path.join(get_package_share_directory('tetra_2dnav'),
                                  'rviz', 'demo_2d.rviz')
  ar_track_alvar_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'ar_track_alvar_parameter.yaml'
  )
  return LaunchDescription([
    DeclareLaunchArgument(
      'cartographer_config_dir',
      default_value=cartographer_config_dir,
      description='Full path to config file to load'
    ),
    DeclareLaunchArgument(
      'configuration_basename',
      default_value=configuration_basename,
      description='Name of lua file for cartographer'
    ),
    DeclareLaunchArgument(
      'use_sime_time',
      default_value='false',
      description='Use simulation (Gazebo) clock if true'
    ),
    DeclareLaunchArgument(
      'resolution',
      default_value=resoltion,
      description='Resolution of a grid cell in the published occupancy grid'
    ),
    Node(
      package='cartographer_ros',
      executable='cartographer_node',
      name='cartographer_node',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time}],
      arguments=['-configuration_directory', cartographer_config_dir,
                  '-configuration_basename',configuration_basename],
      remappings=[
        ('echoes', 'scan'),
        (tf_prefix+'/odom', tf_prefix+'/odometry/filtered')
      ]
    ),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time, 'resolution': resoltion,
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
      name="ar_track_alvar",
      output="screen",
      parameters=[{"output_frame": "/map"},
                  ar_track_alvar_parameter],
      remappings=[
        ('camera_image', 'usb_cam/image_raw'),
        ('camera_info', 'usb_cam/camera_info'),
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