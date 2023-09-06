#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
	luna_lidar_parameter = LaunchConfiguration(
		'luna_lidar_parameter',
		default=os.path.join(
			get_package_share_directory('luna_bringup'),
			'param',
			'luna_lidar.yaml'
		)
	)

	md_parameters = LaunchConfiguration(
		'md_parameters',
		default=os.path.join(
			get_package_share_directory('md_ros2'),
			'param',
			'md.yaml'
		)
	)

	use_sim_time = LaunchConfiguration('use_sim_time', default='false')

	md_ros2_dir = LaunchConfiguration(
		'md_ros2_dir',
		default=os.path.join(
			get_package_share_directory('md_ros2'),
			'launch'
		)
	)

	luna_description_dir = LaunchConfiguration(
		'luna_description_dir',
		default=os.path.join(
			get_package_share_directory('luna_description'),
			'launch'
		)
	)

	return LaunchDescription([
		DeclareLaunchArgument(
			'luna_lidar_parameter',
			default_value=luna_lidar_parameter
		),

		DeclareLaunchArgument(
			'md_parameters',
			default_value=md_parameters
		),

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/luna_lidar_G2.launch.py']),
			launch_arguments={'luna_lidar_parameter': luna_lidar_parameter}.items()
		),

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([md_ros2_dir, '/md.launch.py']),
			launch_arguments={'md_parameters': md_parameters}.items()
		),

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([luna_description_dir, '/luna_description.launch.py']),
			launch_arguments={'use_sim_time': use_sim_time}.items()
		)
	])