cmake_minimum_required(VERSION 3.5)
project(virtual_costmap_layer2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tetra_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
  include
  ${BOOST_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
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

add_library(virtual_layer2 SHARED
  src/virtual_layer2.cpp
)
ament_target_dependencies(virtual_layer2
  ${dependencies}
)

target_compile_definitions(virtual_layer2 PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

target_link_libraries(virtual_layer2
${Boost_LIBRARIES}
)

install(TARGETS virtual_layer2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_export_include_directories(include)
ament_export_libraries(virtual_layer2)

ament_package()
