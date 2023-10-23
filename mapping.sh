#!/bin/bash

source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/.bashrc

pkill -9 -f nav2_controller
sleep 1

ros2 launch tetra_2dnav cartographer_mapping.launch.py