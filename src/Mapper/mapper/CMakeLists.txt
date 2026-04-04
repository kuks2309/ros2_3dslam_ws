cmake_minimum_required(VERSION 3.8)
project(mapper)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(mapper_interfaces REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(waypoint_interfaces REQUIRED)

set(DEPS
  rclcpp rclcpp_action sensor_msgs nav_msgs geometry_msgs
  tf2_ros tf2_geometry_msgs tf2 mapper_interfaces
  amr_interfaces waypoint_interfaces)

# wall_aligner_node
add_executable(wall_aligner_node
  src/wall_aligner_node.cpp
  src/main_wall_aligner.cpp)
ament_target_dependencies(wall_aligner_node ${DEPS})
target_include_directories(wall_aligner_node PUBLIC include)
target_link_libraries(wall_aligner_node ${OpenCV_LIBS})

# map_alignment_checker_node
add_executable(map_alignment_checker_node
  src/map_alignment_checker_node.cpp
  src/main_map_alignment_checker.cpp)
ament_target_dependencies(map_alignment_checker_node ${DEPS})
target_include_directories(map_alignment_checker_node PUBLIC include)
target_link_libraries(map_alignment_checker_node ${OpenCV_LIBS})

# exploration_planner_node (stub - M5 태스크가 생성)
# mapper_orchestrator_node (stub - M6 태스크가 생성)

install(TARGETS
  wall_aligner_node
  map_alignment_checker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wall_aligner
    test/test_wall_aligner.cpp
    src/wall_aligner_node.cpp)
  ament_target_dependencies(test_wall_aligner ${DEPS})
  target_include_directories(test_wall_aligner PUBLIC include)

  ament_add_gtest(test_map_alignment_checker
    test/test_map_alignment_checker.cpp
    src/map_alignment_checker_node.cpp)
  ament_target_dependencies(test_map_alignment_checker ${DEPS})
  target_include_directories(test_map_alignment_checker PUBLIC include)
  target_link_libraries(test_map_alignment_checker ${OpenCV_LIBS})
endif()

ament_package()
