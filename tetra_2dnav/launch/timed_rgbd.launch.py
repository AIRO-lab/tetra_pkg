#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

	timed_roslaunch_parameters = LaunchConfiguration(
		'timed_roslaunch_parameters',
		default=os.path.join(
			get_package_share_directory('timed_roslaunch'),
			'param',
			'timed_roslaunch.yaml'
		)
	)

	timed_roslaunch_dir = LaunchConfiguration(
		'timed_roslaunch_dir',
		default=os.path.join(
			get_package_share_directory('timed_roslaunch'),
			'launch'
		)
	)

	return LaunchDescription([
		DeclareLaunchArgument(
			'timed_roslaunch_parameters',
			default_value=timed_roslaunch_parameters
		),

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([timed_roslaunch_dir, '/timed_roslaunch.launch.py']),
			launch_arguments={'timed_roslaunch_parameters': timed_roslaunch_parameters}.items()
		)
	])