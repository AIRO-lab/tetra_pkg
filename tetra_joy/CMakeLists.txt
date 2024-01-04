cmake_minimum_required(VERSION 3.5)
project(tetra_joy)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(tetra_joy src/tetra_joy_node.cpp)
ament_target_dependencies(tetra_joy rclcpp serial geometry_msgs sensor_msgs std_msgs)

install(TARGETS
tetra_joy
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
