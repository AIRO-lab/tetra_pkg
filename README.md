tetra_service
  clear_costmap_client = nodes->create_client<nav2_msgs::srv::ClearEntireCostmap>("move_base/clear_costmaps");

//TODO move_base edit
tetra_TCP // turtlebot action complie
//move_base result edit --> service (server tcp, client service)

sudo apt install python3-pip
sudo apt install ros-foxy-robot-localization
sudo apt install ros-foxy-joint-state-publisher
sudo apt install ros-foxy-xacro
sudo apt install ros-foxy-joy
sudo apt install ros-foxy-pcl-conversions
sudo apt install ros-foxy-pcl-ros
sudo apt install ros-foxy-rosbridge-*
sudo apt install ros-foxy-cartographer*
sudo apt install ros-foxy-camera-info-manager
sudo apt install ros-foxy-nav2-costmap-2d
sudo apt install ros-foxy-usb-cam
sudo apt install ros-foxy-dwb-critics
sudo apt install libqglviewer-dev-qt5
sudo apt install libusb-dev
sudo apt install libusb-1.0-0-dev
sudo apt-get install v4l-utils

pip3 install pyquaternion
pip3 install pydantic==1.10.9
pip3 install scikit-learn
pip3 install transformations
cd ~/ros2_ws/src/
git clone -b humble https://github.com/uos/sick_tim.git
git clone -b ros2 https://github.com/ros-drivers/usb_cam.git
git clone https://github.com/Wisc-HCI/tf2_web_republisher_py.git
git clone -b foxy-devel https://github.com/rst-tu-dortmund/teb_local_planner.git
git clone -b foxy-devel https://github.com/ros-perception/depthimage_to_laserscan.git
git clone https://github.com/RoverRobotics-forks/serial-ros2.git

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

cd ~/
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 20201223_git
git branch 20201223_git
git switch 20201223_git
mkdir build
cd build
cmake ../
make -j4
sudo make install -j4