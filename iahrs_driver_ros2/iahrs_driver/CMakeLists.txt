cmake_minimum_required(VERSION 3.5)
project(iahrs_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(iahrs_msgs REQUIRED)

include_directories(include)

add_executable(iahrs_driver src/iahrs_driver.cpp)
ament_target_dependencies(iahrs_driver rclcpp tf2_ros tf2_geometry_msgs tf2 geometry_msgs sensor_msgs std_msgs iahrs_msgs)

install(TARGETS
iahrs_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
