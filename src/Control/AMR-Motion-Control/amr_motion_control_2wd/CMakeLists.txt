cmake_minimum_required(VERSION 3.8)
project(amr_motion_control_2wd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Motion profile static library (pure C++ math, no ROS deps)
add_library(motion_profile STATIC src/motion_profile.cpp)
target_include_directories(motion_profile PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motion_profile PUBLIC cxx_std_17)

# Main executable
add_executable(amr_motion_control_2wd_node
  src/main.cpp
  src/motion_common.cpp
  src/localization_watchdog.cpp
  src/spin_action_server.cpp
  src/turn_action_server.cpp
  src/yaw_control_action_server.cpp
  src/translate_action_server.cpp
  src/translate_reverse_action_server.cpp
  src/pure_pursuit_action_server.cpp
  src/stanley_action_server.cpp)
target_include_directories(amr_motion_control_2wd_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amr_motion_control_2wd_node
  rclcpp rclcpp_action amr_interfaces
  sensor_msgs std_msgs std_srvs geometry_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(amr_motion_control_2wd_node motion_profile)
target_compile_features(amr_motion_control_2wd_node PRIVATE cxx_std_17)

install(TARGETS motion_profile ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS amr_motion_control_2wd_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
