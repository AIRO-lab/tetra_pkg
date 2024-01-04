cmake_minimum_required(VERSION 3.8)
project(tetra)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tetra_msgs REQUIRED)

include_directories(
		${PROJECT_SOURCE_DIR}
		${PROJECT_SOURCE_DIR}/include
		${PROJECT_SOURCE_DIR}/src
		${rclcpp_INCLUDE_DIRS}
		${rmw_implementation_INCLUDE_DIRS}
		${std_msgs_INCLUDE_DIRS}
		${std_srvs_INCLUDE_DIRS}
		${sensor_msgs_INCLUDE_DIRS}
		${tf2_INCLUDE_DIRS}
		${tf2_ros_INCLUDE_DIRS}
		${nav_msgs_INCLUDE_DIRS}
		${geometry_msgs_INCLUDE_DIRS})

add_executable(tetra include/rs232.c include/rs232.h include/rs232_common.c include/rs232_common.h src/drive_module.c src/drive_module.h src/dssp_rs232_drive_module.c src/dssp_rs232_drive_module.h src/tetra.cpp)

set(dependencies
  rclcpp
  rmw_implementation
  std_msgs
  std_srvs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  tetra_msgs
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}
                      ${rclcpp_LIBRARIES}
                      ${rmw_implementation_LIBRARIES}
                      ${std_msgs_LIBRARIES}
                      ${std_srvs_LIBRARIES}
                      ${sensor_msgs_LIBRARIES}
                      ${tf2_LIBRARIES}
                      ${tf2_ros_LIBRARIES}
                      ${nav_msgs_LIBRARIES}
                      ${geometry_msgs_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
