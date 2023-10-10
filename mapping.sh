#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/.bashrc
export ROS_NAMESPACE=TE2317003

pkill -9 -f nav2_controller
sleep 1

ros2 launch tetra_2dnav cartographer_mapping.launch