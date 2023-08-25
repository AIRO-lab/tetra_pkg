cmake_minimum_required(VERSION 3.8)
project(tetra_landmark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tetra_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(ar_track_alvar_msgs REQUIRED)

add_executable(tetra_landmark_save src/tetraDS_landmark_save.cpp)
ament_target_dependencies(tetra_landmark_save rclcpp std_msgs std_srvs geometry_msgs visualization_msgs sensor_msgs tetra_msgs message_filters ar_track_alvar_msgs)
add_executable(tetra_landmark_load src/tetraDS_landmark_load.cpp)
ament_target_dependencies(tetra_landmark_load rclcpp std_msgs std_srvs geometry_msgs visualization_msgs message_filters ar_track_alvar_msgs)

install(TARGETS
  tetra_landmark_save
  tetra_landmark_load
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
