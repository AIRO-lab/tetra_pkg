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
  tf_prefix = LaunchConfiguration("tf_prefix")
  tf_prefix_arg = DeclareLaunchArgument(
    'tf_prefix',
    default_value=os.getenv('ROS_NAMESPACE')
  )
  ekf_localization_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'ekf_localization.yaml'
  )
  ekf_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[{'tf_prefix': tf_prefix},
                ekf_localization_parameter],
  )
  
  urdf_file = 'xacro --inorder %s' % os.path.join(get_package_share_directory('tetra_description'),
                                                  'urdf',
                                                  'tetraDS.xacro')
  robot_description = Command(urdf_file)
  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    parameters=[{'robot_description': robot_description}],
  )
  joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    parameters=[{'robot_description': robot_description}],
    arguments=['-urdf_file', urdf_file]
  )
  
  sick_tim_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'sick_tim.yaml'
  )
  sick_tim_node = Node(
      package='sick_tim',
      executable='sick_tim551_2050001',
      parameters=[{'tf_prefix': tf_prefix},
                  {'frame_id': tf_prefix + "/laser"},
                  sick_tim_parameter]
  )

  joy_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'joy_params.yaml'
  )
  joy_node = Node(
      package='joy_node',
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
      parameters=[{'tf_prefix': tf_prefix,
                   'm_bSingle_TF_option': m_bSingle_TF_option}]
  )
  
  ekf_option = LaunchConfiguration("ekf_option")
  ekf_option_arg = DeclareLaunchArgument(
    'ekf_option',
    default_value='True'
  )
  tetra_node = Node(
    package="tetra",
    executable="tetra",
    name="tetra",
    output="screen",
    emulate_tty=True,
    parameters=[
      {"tf_prefix": tf_prefix,
      "ekf_option": ekf_option}
    ]
  )
  
  conveyor_option = LaunchConfiguration("conveyor_option")
  conveyor_option_arg = DeclareLaunchArgument(
    'conveyor_option',
    default_value='False'
  )
  tetra_interface_node = Node(
    package="tetra_interface",
    executable="tetra_interface",
    name="tetra_interface",
    output="screen",
    emulate_tty=True,
    parameters=[
      {"tf_prefix": tf_prefix,
      "conveyor_option": conveyor_option}
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
      {"tf_prefix": tf_prefix},
      tetra_service_parameter
    ]
  )
  
  usb_cam_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'usb_cam_params.yaml'
  )
  usb_cam_node = Node(
    package="usb_cam",
    executable="usb_cam_node",
    name="usb_cam",
    output="screen",
    respawn=True,
    parameters=[{"frame_id": tf_prefix + "/usb_cam"},
                usb_cam_parameter]
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
    parameters=[{"output_frame": tf_prefix + "/usb_cam"},
                ar_track_alvar_parameter],
    remappings=[
      ('camera_image', 'usb_cam/image_raw'),
      ('camera_info', 'usb_cam/camera_info')
    ]
  )
  
  realsense_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'examples', 'pointcloud')
  
  cyglidar_parameter = os.path.join(
    get_package_share_directory('tetra_2dnav'),
    'param',
    'cyglidar_parameter.yaml'
  )
  cyglidar_d1_ros2_node = Node(
    package="cyglidar_d1_ros2",
    executable="cyglidar_d1_publisher",
    name="line_laser",
    output="screen",
    parameters=[{"frame_id": tf_prefix + "/laser_link2"},
                cyglidar_parameter]
  )
  
  tf2_web_republisher_node = Node(
    package="tf2_web_republisher_py",
    executable="tf2_web_republisher",
    name="tf2_web_republisher",
    output="screen"
  )
  
  rosbridge_server_dir = os.path.join(get_package_share_directory('rosbridge_server'), 'launch')
  
  port = LaunchConfiguration("port")
  port_arg = DeclareLaunchArgument(
    'port',
    default_value='5100'
  )
  tetra_tcp_node = Node(
    package="tetra_tcp",
    executable="tetra_tcp",
    name="tetra_tcp",
    output="screen",
    parameters=[port]
  )
  
  return LaunchDescription([
		tf_prefix_arg,
    m_bSingle_TF_option_arg,
    ekf_option_arg,
    conveyor_option_arg,
    port_arg,
    ekf_localization_node,
    robot_state_publisher_node,
    joint_state_publisher_node,
    sick_tim_node,
    joy_node,
    iahrs_driver_node,
    tetra_node,
    tetra_interface_node,
    tetra_service_node,
    usb_cam_node,
    ar_track_alvar_node,
    IncludeLaunchDescription(PythonLaunchDescriptionSource([realsense_dir, '/rs_pointcloud_launch.py'])),
    IncludeLaunchDescription(PythonLaunchDescriptionSource([rosbridge_server_dir, '/rosbridge_websocket_launch.xml'])),
    cyglidar_d1_ros2_node,
    tf2_web_republisher_node,
    tetra_tcp_node
	])