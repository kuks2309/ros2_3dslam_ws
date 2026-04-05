cmake_minimum_required(VERSION 3.8)
project(amr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/AMRMotionSpin.action"
  "action/AMRMotionTurn.action"
  "action/AMRMotionTranslate.action"
  "action/AMRMotionYawControl.action"
  "action/AMRMotionPurePursuit.action"
  "action/AMRMotionStanley.action"
  "srv/AMRControlStop.srv"
  "srv/UpdateTranslateEndpoint.srv"
  "srv/UpdatePurePursuitPath.srv"
  "srv/UpdateStanleyPath.srv"
  "msg/SafetyStatus.msg"
  DEPENDENCIES std_msgs nav_msgs
)

ament_package()
