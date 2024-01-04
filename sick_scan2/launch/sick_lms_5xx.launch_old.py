# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import os
import io
from yaml import load, dump
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import launch_ros.actions
from launch_ros.substitutions import ExecutableInPackage
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

import sys


def generate_launch_description():
    #executable = ExecutableInPackage(package='tf2_ros', executable='static_transform_publisher')
    exec_sick_scan2 = ExecutableInPackage(package='sick_scan2', executable='sick_generic_caller')
    exec_sick_scan2_gdb = ExecutableInPackage(package='sick_scan2', executable='gdb')

    ############################################################################################
    #
    # modify your settings here:
    #
        # TODO(wjwwood): Use a substitution to find share directory once this is implemented in launch
    urdf = os.path.join(get_package_share_directory('sick_scan2'),
                         'launch', 'single_rrbot.urdf')


    yamlFileName = os.path.join(get_package_share_directory('sick_scan2'),
                        'launch', 'sick_lms_5xx.yaml')
    with open(yamlFileName, 'r') as stream:
        try:
            yamlConf=yaml.load(stream)
            print(yaml.safe_load(stream))
        except yaml.YAMLError as exc:
            print(exc)
    print ("##########################################################################")
    print ("# ROS2 Driver for SICK SCANNER (SET YOUR SETTING ACCORDING TO YOUR SETUP!")
    print ("##########################################################################")
    print ("Your Settings:")
    #hostname = '192.168.0.151'
    #port = "2112";
    #name = "sick_lms_5xx"
    #frameId = "laser"
    print(yaml.dump(yamlConf))
    #print ("hostname [IP-address of scanner   ]: " + hostname)
    #print ("port     [IP-port    of scanner   ]: " + port)
    #print ("name     [to identify scanner type]: " + name)
    print ("##########################################################################")
    print ("# STARTING NODE")
    print ("##########################################################################")



    # Yaml-File reserved for future use ...
    return LaunchDescription([
        #LogInfo(msg=[
        #    'Try to start world <-> laser'
        #]),

        #launch.actions.ExecuteProcess(
        #cmd=[executable, '0','0','0','0','0','0','world','laser'], output='screen'),
        #
        # YAML-File support seems to be buggy. ros2 param crashes ...
         launch_ros.actions.Node(
                        name = 'sick_lms_5xx',
                        package='sick_scan2',
            		    executable='sick_generic_caller',
                        parameters=[yamlFileName],
                        output='screen')
        # __hostname:=192.168.0.151 __frame_id:=laser __port:=2112 --ros-args --remap __name:=sick_lms_5xx
        #launch.actions.ExecuteProcess(
        # 	cmd=[exec_sick_scan2, '__hostname:='+hostname ," __port:=" + port , " __frame_id:=" + frameId , "--ros-args --remap __name:=" + name], output='screen')
        # launch.actions.ExecuteProcess(
     	# 	cmd=[exec_sick_scan2_gdb, '--args sick_generic_caller __hostname:='+hostname+ " __port:=" + port + " __frame_id:=" + frameId + " --ros-args --remap __name:=" + name], output='screen')

    ])

#
# construction place ...
#

if __name__== "__main__":
    if (len(sys.argv) > 1):
        yamlStemFileName = sys.argv[1] + '.yaml'
    else:
        yamlStemFileName = 'sick_lms_5xx.yaml'
    print("Hello Tester!")

    yamlFileName = os.path.join(get_package_share_directory('sick_scan2'),
                                'launch', yamlStemFileName)

    with open(yamlFileName) as fp:
        my_configuration = yaml.load(fp)
        print(my_configuration)
        paramSet = my_configuration['sick_generic_caller']['ros__parameters']

        for i in range(len(paramSet)):
            print(paramSet[i])

    file = open(yamlFileName, 'r')
    print (file.readlines())
