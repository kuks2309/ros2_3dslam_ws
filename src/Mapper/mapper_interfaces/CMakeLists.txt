cmake_minimum_required(VERSION 3.8)
project(mapper_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WallAlign.action"
  "action/MapAlignmentCheck.action"
  "action/ExploreUnknown.action"
  "srv/SlamControl.srv"
  "srv/MapperCommand.srv"
  "msg/MapperStatus.msg"
  DEPENDENCIES geometry_msgs
)

ament_package()
