# Hyulim TETRA_ROS2(Foxy)


## Install Dependencies
```bash
  sudo apt install python3-pip -y

  sudo apt install ros-foxy-robot-localization -y
  sudo apt install ros-foxy-spatio-temporal-voxel-slayer -y
  sudo apt install ros-foxy-xacro -y
  sudo apt install ros-foxy-joy -y
  sudo apt install ros-foxy-pcl-conversions -y
  sudo apt install ros-foxy-pcl-ros -y
  sudo apt install ros-foxy-rosbridge-* -y
  sudo apt install ros-foxy-cartographer* -y
  sudo apt install ros-foxy-camera-info-manager -y
  sudo apt install ros-foxy-nav2-* -y
  sudo apt install ros-foxy-dwb-critics -y

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
  git clone -b foxy-devel https://github.com/rst-tu-dortmund/teb_local_planner.git
  git clone -b foxy-devel https://github.com/ros-perception/depthimage_to_laserscan.git
  git clone https://github.com/RoverRobotics-forks/serial-ros2.git
```


## Install Realsense Package
```bash
  cd ~/ros2_ws/src/
  sudo apt-get install python3-rosdep -y
  sudo apt install ros-foxy-librealsense2*
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