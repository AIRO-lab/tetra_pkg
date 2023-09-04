#!/bin/bash
map_name=$1
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd /home/tetra/ros2_ws/src/tetra_pkg/tetra_2dnav/maps/
ros2 run nav2_map_server map_saver_cli --occ 55 --free 45 -f ${map_name}
