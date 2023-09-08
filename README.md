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

ar_track_alvar
iahrs_driver

tetra_2dnav
config tetra_nav parameter editting....

//TODO move_base edit
tetra_service
tetra_TCP

sudo apt install python3-pip
sudo apt install ros-foxy-robot-localization
sudo apt install ros-foxy-joint-state-publisher
sudo apt install ros-foxy-xacro
sudo apt install ros-foxy-pcl-conversions
sudo apt install ros-foxy-rosbridge-*

pip3 install scikit-learn
pip3 install transformations

cd ~/ros2_ws/src/
git clone -b humble https://github.com/uos/sick_tim.git
git clone -b ros2 https://github.com/ros-drivers/usb_cam.git
git clone https://github.com/Wisc-HCI/tf2_web_republisher_py.git

cd ~/ros2_ws/src/
sudo apt install ros-foxy-librealsense2*
git clone -b ros2-development https://github.com/IntelRealSense/realsense-ros.git
cd ~/ros2_ws/
sudo apt-get install python3-rosdep -y
sudo rosdep init
rosdep update

cd ~/ros2_ws/src/
git clone -b ROS2-v0.3.0 https://github.com/CygLiDAR-ROS/cyglidar_d1.git
cd ~/ros2_ws/
colcon build
cd ~/ros2_ws/src/cyglidar_d1/scripts/
chmod +x create_udev_rules.sh
./create_udev_rules.sh