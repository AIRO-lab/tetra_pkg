cmake_minimum_required(VERSION 3.8)
project(tetra_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(PCL 1.1 REQUIRED)
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
  ${geometry_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(tetra_interface include/rs232.c include/rs232.h include/rs232_common.c include/rs232_common.h src/power_module.c src/power_module.h src/dssp_rs232_power_module.c src/dssp_rs232_power_module.h src/tetra_interface.cpp)

set(dependencies
  rclcpp
  rmw_implementation
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  pcl_conversions
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
                      ${geometry_msgs_LIBRARIES}
                      ${PCL_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
