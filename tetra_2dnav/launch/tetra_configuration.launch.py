#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
  ekf_localization_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'ekf_localization.yaml'
  )
  ekf_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[ekf_localization_parameter],
  )
  
  use_sim_time = LaunchConfiguration("use_sim_time")
  use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value="false")
  urdf_file = os.path.join(get_package_share_directory('tetra_description'),
                                                  'urdf',
                                                  'tetra.xacro')
  robot_description = xacro.process_file(urdf_file).toxml()
  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    parameters=[{
      'use_sim_time': use_sim_time,
      'robot_description': robot_description}]
  )

  joy_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'joy_params.yaml'
  )
  joy_node = Node(
      package='joy',
      executable='joy_node',
      parameters=[joy_parameter]
  )
  
  m_bSingle_TF_option = LaunchConfiguration("m_bSingle_TF_option")
  m_bSingle_TF_option_arg = DeclareLaunchArgument(
    'm_bSingle_TF_option',
    default_value='False'
  )
  iahrs_driver_node = Node(
      package='iahrs_driver',
      executable='iahrs_driver',
      parameters=[{'m_bSingle_TF_option': m_bSingle_TF_option}]
  )
  
  ekf_option = LaunchConfiguration("m_bEKF_option")
  ekf_option_arg = DeclareLaunchArgument(
    'm_bEKF_option',
    default_value='True'
  )
  tetra_node = Node(
    package="tetra",
    executable="tetra",
    name="tetra",
    output="screen",
    parameters=[
      {"m_bEKF_option": ekf_option}
    ]
  )
  
  conveyor_option = LaunchConfiguration("m_bConveyor_option")
  conveyor_option_arg = DeclareLaunchArgument(
    'm_bConveyor_option',
    default_value='False'
  )
  ultrasonic_option = LaunchConfiguration("m_bUltrasonic_option")
  ultrasonic_option_arg = DeclareLaunchArgument(
    'm_bUltrasonic_option',
    default_value='False'
  )
  tetra_interface_node = Node(
    package="tetra_interface",
    executable="tetra_interface",
    name="tetra_interface",
    output="screen",
    parameters=[
      {"m_bConveyor_option": conveyor_option},
      {"m_bUltrasonic_option": ultrasonic_option}
    ]
  )
  
  tetra_service_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'tetra_service.yaml'
  )
  tetra_service_node = Node(
    package="tetra_service",
    executable="tetra_service",
    name="tetra_service",
    output="screen",
    emulate_tty=True,
    parameters=[
      tetra_service_parameter
    ]
  )
  
  ar_track_alvar_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'ar_track_alvar_parameter.yaml'
  )
  ar_track_alvar_node = Node(
    package="ar_track_alvar",
    executable="individual_markers_no_kinect",
    name="ar_track_alvar",
    output="screen",
    parameters=[{"output_frame": "camera"},
                ar_track_alvar_parameter],
    # remappings=[
    #   ('camera_image', 'usb_cam/image_raw'),
    #   ('camera_info', 'usb_cam/camera_info')
    # ]
  )
  
  # tf2_web_republisher_node = Node(
  #   package="tf2_web_republisher_py",
  #   executable="tf2_web_republisher",
  #   name="tf2_web_republisher",
  #   output="screen"
  # )
  
  # rosbridge_server_dir = os.path.join(get_package_share_directory('rosbridge_server'), 'launch')
  
  # port = LaunchConfiguration("port")
  # port_arg = DeclareLaunchArgument(
  #   'port',
  #   default_value='5100'
  # )
  # tetra_tcp_node = Node(
  #   package="tetra_tcp",
  #   executable="tetra_tcp",
  #   name="tetra_tcp",
  #   output="screen",
  #   parameters=[port]
  # )
  
  return LaunchDescription([
    m_bSingle_TF_option_arg,
    ekf_option_arg,
    conveyor_option_arg,
    ultrasonic_option_arg,
    use_sim_time_arg,
    # port_arg,
    tetra_node,
    ekf_localization_node,
    tetra_interface_node,
    iahrs_driver_node,
    joy_node,
    robot_state_publisher_node,
    tetra_service_node,
    ar_track_alvar_node,
    
    # USB Camera
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('usb_cam'), '/launch/camera.launch.py']),
		),
			
		# sick_tim_571
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('sick_scan2'), '/launch/sick_tim_5xx.launch.py']),
		),
		
		# cygbot 2D lidar
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('cyglidar_d1_ros2'), '/launch/cyglidar.launch.py']),
		),
		
		# realsense D435F
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
		),
    # IncludeLaunchDescription(XMLLaunchDescriptionSource([rosbridge_server_dir, '/rosbridge_websocket_launch.xml'])),
    # tf2_web_republisher_node,
    # tetra_tcp_node
	])