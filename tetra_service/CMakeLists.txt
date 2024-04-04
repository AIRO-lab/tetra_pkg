cmake_minimum_required(VERSION 3.8)
project(tetra_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tetra_msgs REQUIRED)
find_package(ar_track_alvar_msgs REQUIRED)

add_executable(tetra_service src/tetra_service.cpp)
ament_target_dependencies(tetra_service rclcpp rclcpp_action nav2_msgs robot_localization tf2 tf2_ros geometry_msgs std_srvs std_msgs visualization_msgs sensor_msgs rcl_interfaces tetra_msgs ar_track_alvar_msgs)

install(TARGETS
  tetra_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
