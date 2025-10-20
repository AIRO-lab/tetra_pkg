# Hyulim TETRA_ROS2(humble)


## Install Dependencies
```bash
  sudo apt install python3-pip -y

  sudo apt install ros-humble-robot-localization -y
  sudo apt install ros-humble-spatio-temporal-voxel-slayer -y
  sudo apt install ros-humble-xacro -y
  sudo apt install ros-humble-joy -y
  sudo apt install ros-humble-pcl-conversions -y
  sudo apt install ros-humble-pcl-ros -y
  sudo apt install ros-humble-rosbridge-* -y
  sudo apt install ros-humble-cartographer* -y
  sudo apt install ros-humble-camera-info-manager -y
  sudo apt install ros-humble-nav2-* -y
  sudo apt install ros-humble-dwb-critics -y

  sudo apt install libqglviewer-dev-qt5 -y
  sudo apt install libusb-dev -y
  sudo apt install libusb-1.0-0-dev -y
  sudo apt install v4l-utils -y

  pip3 install pyquaternion
  pip3 install pydantic==1.10.9
  pip3 install scikit-learn
  pip3 install transformations
```


## Install Dependencies Packages
```bash
  cd ~/ros2_ws/src/
  git clone https://github.com/Wisc-HCI/tf2_web_republisher_py.git
  git clone -b humble-devel https://github.com/rst-tu-dortmund/teb_local_planner.git
  git clone -b humble-devel https://github.com/ros-perception/depthimage_to_laserscan.git
  git clone https://github.com/RoverRobotics-forks/serial-ros2.git
```


## Install Realsense Package
```bash
  cd ~/ros2_ws/src/
  sudo apt-get install python3-rosdep -y
  sudo apt install ros-humble-librealsense2*
  cd ~/ros2_ws/
  sudo rosdep init
  rosdep update
```


## Setting cyglidar
```bash
  cd ~/ros2_ws/src/tetra_pkg/cyglidar_d1/scripts/
  chmod +x create_udev_rules.sh
  ./create_udev_rules.sh
```


## Setting g2o(for teb_local_planner)
```bash
  cd ~/
  git clone -b 20201223_git https://github.com/RainerKuemmerle/g2o.git
  cd g2o
  git checkout 20201223_git
  git branch 20201223_git
  git switch 20201223_git
  mkdir build
  cd build
  cmake ../
  make -j4
  sudo make install -j4
```
