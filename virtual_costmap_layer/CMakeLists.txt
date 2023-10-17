cmake_minimum_required(VERSION 3.5)
project(virtual_costmap_layer)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tetra_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)
set(dependencies
  geometry_msgs
  pluginlib
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  tetra_msgs
  nav2_costmap_2d
)

add_library(virtual_layer SHARED
  src/virtual_layer.cpp
)
ament_target_dependencies(virtual_layer
  ${dependencies}
)

target_compile_definitions(virtual_layer PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS virtual_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_definitions("PLUGINLIB__DISBLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_package()
