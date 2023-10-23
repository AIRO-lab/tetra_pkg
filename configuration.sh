#!/bin/bash
map_name=$1
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/.bashrc

ros2 launch tetra_2dnav tetra_configuration.launch.py
