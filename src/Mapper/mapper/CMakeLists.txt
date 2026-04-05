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
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(mapper_interfaces REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(waypoint_interfaces REQUIRED)

set(DEPS
  rclcpp rclcpp_action sensor_msgs nav_msgs geometry_msgs
  tf2_ros tf2 mapper_interfaces
  amr_interfaces waypoint_interfaces)

# Static libraries
add_library(wall_aligner_lib STATIC src/wall_aligner_node.cpp)
ament_target_dependencies(wall_aligner_lib ${DEPS})
target_include_directories(wall_aligner_lib PUBLIC include)

add_library(map_alignment_checker_lib STATIC src/map_alignment_checker_node.cpp)
ament_target_dependencies(map_alignment_checker_lib ${DEPS})
target_include_directories(map_alignment_checker_lib PUBLIC include)
target_link_libraries(map_alignment_checker_lib ${OpenCV_LIBS})

add_library(exploration_planner_lib STATIC src/exploration_planner_node.cpp)
ament_target_dependencies(exploration_planner_lib ${DEPS})
target_include_directories(exploration_planner_lib PUBLIC include)

add_library(mapper_orchestrator_lib STATIC src/mapper_orchestrator_node.cpp)
ament_target_dependencies(mapper_orchestrator_lib ${DEPS})
target_include_directories(mapper_orchestrator_lib PUBLIC include)

# Executables
add_executable(wall_aligner_node src/main_wall_aligner.cpp)
target_link_libraries(wall_aligner_node wall_aligner_lib)

add_executable(map_alignment_checker_node src/main_map_alignment_checker.cpp)
target_link_libraries(map_alignment_checker_node map_alignment_checker_lib)

add_executable(exploration_planner_node src/main_exploration_planner.cpp)
target_link_libraries(exploration_planner_node exploration_planner_lib)

add_executable(mapper_orchestrator_node src/main_orchestrator.cpp)
target_link_libraries(mapper_orchestrator_node mapper_orchestrator_lib)

install(TARGETS
  wall_aligner_lib map_alignment_checker_lib exploration_planner_lib mapper_orchestrator_lib
  wall_aligner_node map_alignment_checker_node exploration_planner_node mapper_orchestrator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_wall_aligner test/test_wall_aligner.cpp)
  target_link_libraries(test_wall_aligner wall_aligner_lib)

  ament_add_gtest(test_map_alignment_checker test/test_map_alignment_checker.cpp)
  target_link_libraries(test_map_alignment_checker map_alignment_checker_lib)

  ament_add_gtest(test_exploration_planner test/test_exploration_planner.cpp)
  target_link_libraries(test_exploration_planner exploration_planner_lib)

  ament_add_gtest(test_orchestrator_states test/test_orchestrator_states.cpp)
  target_link_libraries(test_orchestrator_states mapper_orchestrator_lib)
endif()

ament_export_include_directories(include)
ament_package()
