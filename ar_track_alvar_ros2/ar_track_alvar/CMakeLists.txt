cmake_minimum_required(VERSION 3.8)
project(ar_track_alvar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ar_track_alvar_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(individual_markers_no_kinect src/IndividualMarkersNoKinect.cpp)
ament_target_dependencies(individual_markers_no_kinect rclcpp tf2_geometry_msgs resource_retriever tf2 tf2_ros geometry_msgs std_msgs visualization_msgs sensor_msgs ar_track_alvar_msgs pcl_conversions image_transport cv_bridge pcl_ros Eigen3 OpenCV)

install(TARGETS
  individual_markers_no_kinect
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
