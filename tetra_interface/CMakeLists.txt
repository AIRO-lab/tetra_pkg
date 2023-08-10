cmake_minimum_required(VERSION 3.8)
project(tetra_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tetra_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS 
  date_time
)


include_directories(include)

add_executable(tetra_interface include/rs232.c include/rs232.h include/rs232_common.c include/rs232_common.h src/power_module.c src/power_module.h src/dssp_rs232_power_module.c src/dssp_rs232_power_module.h src/tetraDS_interface.cpp)
target_link_libraries(tetra_interface
                      ${Boost_LIBRARIES})
ament_target_dependencies(tetra_interface rclcpp std_srvs std_msgs sensor_msgs tf2 tf2_geometry_msgs pcl_ros nav_msgs geometry_msgs tetra_msgs)

install(TARGETS
  tetra_interface
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
