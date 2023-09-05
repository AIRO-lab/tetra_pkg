cmake_minimum_required(VERSION 3.5)
project(tetra_tcp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tetra_msgs REQUIRED)

add_executable(tetra_tcp src/tetraDS_TCP.cpp)
ament_target_dependencies(tetra_tcp rclcpp sensor_msgs tetra_msgs nav_msgs)

install(TARGETS
  tetra_tcp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
