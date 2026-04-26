cmake_minimum_required(VERSION 3.8)
project(wall_detector)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mapper_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

set(DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  mapper_interfaces)

# 라이브러리 (테스트·재사용)
add_library(wall_detector_lib STATIC
  src/wall_detector_node.cpp)
ament_target_dependencies(wall_detector_lib ${DEPS})
target_include_directories(wall_detector_lib PUBLIC include)
target_link_libraries(wall_detector_lib ${OpenCV_LIBS})
target_include_directories(wall_detector_lib PUBLIC ${OpenCV_INCLUDE_DIRS})

# 실행 파일
add_executable(wall_detector_node src/main_wall_detector.cpp)
ament_target_dependencies(wall_detector_node ${DEPS})
target_link_libraries(wall_detector_node wall_detector_lib ${OpenCV_LIBS})
target_include_directories(wall_detector_node PUBLIC include)

install(TARGETS wall_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wall_detector
    test/test_wall_detector.cpp)
  if(TARGET test_wall_detector)
    target_link_libraries(test_wall_detector wall_detector_lib ${OpenCV_LIBS})
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(wall_detector_lib)
ament_export_dependencies(${DEPS})
ament_package()
