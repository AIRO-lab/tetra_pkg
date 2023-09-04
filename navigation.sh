#!/bin/bash
map_name=$1
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/.bashrc
echo $map_name

pkill -9 -f nav2_controller
sleep 1

ros2 launch tetra_2dnav nav2_tetra.launch.py map_name:=${map_name}