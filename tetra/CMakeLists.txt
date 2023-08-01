cmake_minimum_required(VERSION 3.8)
project(tetra)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tetra_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(tetra include/rs232.c include/rs232.h include/rs232_common.c include/rs232_common.h src/drive_module.c src/drive_module.h src/dssp_rs232_drive_module.c src/dssp_rs232_drive_module.h src/tetraDS.cpp)
ament_target_dependencies(tetra rclcpp tf2_ros tf2 nav_msgs geometry_msgs std_msgs sensor_msgs tetra_msgs tf2_geometry_msgs)

install(TARGETS
  tetra
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
