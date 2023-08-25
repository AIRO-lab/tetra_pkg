cmake_minimum_required(VERSION 3.8)
project(tetra_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gpio.msg"
  "srv/AngularPositionMove.srv"
  "srv/ConveyorAutoMovement.srv"
  "srv/ConveyorDataRead.srv"
  "srv/ConveyorManualMovement.srv"
  "srv/ConveyorParameterRead.srv"
  "srv/ConveyorParameterWrite.srv"
  "srv/Integrallog.srv"
  "srv/Ledcontrol.srv"
  "srv/Ledtogglecontrol.srv"
  "srv/LoadcellCallibration.srv"
  "srv/PowerAdcRead.srv"
  "srv/PowerDataRead.srv"
  "srv/PowerGetIoStatus.srv"
  "srv/PowerParameterRead.srv"
  "srv/PowerParameterWrite.srv"
  "srv/PowerSetEnable.srv"
  "srv/PowerSetOutport.srv"
  "srv/PowerSetSingleEnable.srv"
  "srv/PowerSetSingleOutport.srv"
  "srv/PowerSonarCmd.srv"
  "srv/PowerSonarRead.srv"
  "srv/PowerVersionRead.srv"
  "srv/PowerWheelEnable.srv"
  "srv/Toggleon.srv"
  "srv/ParameterRead.srv"
  "srv/ParameterWrite.srv"
  "srv/Savemark.srv"
  "srv/SetMoveMode.srv"
  "srv/LinearPositionMove.srv"
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
