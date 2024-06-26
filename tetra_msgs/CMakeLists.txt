cmake_minimum_required(VERSION 3.8)
project(tetra_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gpio.msg"
  "msg/Form.msg"
  "msg/Form2.msg"
  "msg/LandmarkEntry.msg"
  "msg/LandmarkList.msg"
  "msg/Obstacles.msg"
  "msg/Obstacles2.msg"
  "msg/Zone.msg"
  "msg/Zone2.msg"
  "srv/Accelerationslop.srv"
  "srv/AllDataReset.srv"
  "srv/AngularPositionMove.srv"
  "srv/ConveyorAutoMovement.srv"
  "srv/ConveyorDataRead.srv"
  "srv/ConveyorManualMovement.srv"
  "srv/ConveyorParameterRead.srv"
  "srv/ConveyorParameterWrite.srv"
  "srv/DeleteDataAll.srv"
  "srv/DeleteLandmark.srv"
  "srv/DeleteLocation.srv"
  "srv/DeleteMap.srv"
  "srv/DockingControl.srv"
  "srv/DockingStop.srv"
  "srv/EulerAngleInit.srv"
  "srv/EulerAngleReset.srv"
  "srv/GetInformation.srv"
  "srv/GetLandmarkList.srv"
  "srv/GetLocation.srv"
  "srv/GetLocationList.srv"
  "srv/GetMapList.srv"
  "srv/GetObjectLocation.srv"
  "srv/GotoCancel.srv"
  "srv/GotoConveyor.srv"
  "srv/GotoLocation.srv"
  "srv/GotoLocation2.srv"
  "srv/ImuReset.srv"
  "srv/Integrallog.srv"
  "srv/LedControl.srv"
  "srv/LedToggleControl.srv"
  "srv/LinearPositionMove.srv"
  "srv/LoadcellCallibration.srv"
  "srv/LoadingCheck.srv"
  "srv/ParameterRead.srv"
  "srv/ParameterWrite.srv"
  "srv/Patrol.srv"
  "srv/PatrolConveyor.srv"
  "srv/PoseEstimate.srv"
  "srv/PoseVelocityReset.srv"
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
  "srv/RebootSensor.srv"
  "srv/RosNodeKill.srv"
  "srv/RunMapping.srv"
  "srv/RunNavigation.srv"
  "srv/SaveMap.srv"
  "srv/SaveMarker.srv"
  "srv/Servo.srv"
  "srv/SetEkf.srv"
  "srv/SetHomeId.srv"
  "srv/SetInitpose.srv"
  "srv/SetLocation.srv"
  "srv/SetMaxSpeed.srv"
  "srv/SetMoveMode.srv"
  "srv/SetPose.srv"
  "srv/ToggleOn.srv"
  "srv/UnloadingCheck.srv"
  "srv/VirtualObstacle.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs cartographer_ros_msgs
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
