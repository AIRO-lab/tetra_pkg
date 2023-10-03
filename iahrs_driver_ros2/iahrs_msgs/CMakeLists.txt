cmake_minimum_required(VERSION 3.5)
project(iahrs_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AllDataReset.srv"
  "srv/EulerAngleInit.srv"
  "srv/EulerAngleReset.srv"
  "srv/PoseVelocityReset.srv"
  "srv/RebootSensor.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
