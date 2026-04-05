cmake_minimum_required(VERSION 3.8)
project(amr_motion_control_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

# Static library
add_library(sil_predictor STATIC
  src/trapezoidal_profile.cpp
  src/path_controller_2wd.cpp
  src/sil_predictor.cpp
)
target_include_directories(sil_predictor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(sil_predictor PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_trapezoidal_profile test/test_trapezoidal_profile.cpp)
  target_link_libraries(test_trapezoidal_profile sil_predictor)

  ament_add_gtest(test_path_controller_2wd test/test_path_controller_2wd.cpp)
  target_link_libraries(test_path_controller_2wd sil_predictor)

  ament_add_gtest(test_sil_predictor test/test_sil_predictor.cpp)
  target_link_libraries(test_sil_predictor sil_predictor)
endif()

# pybind11 Python binding
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

pybind11_add_module(_sil_predictor_cpp src/bindings.cpp)
target_link_libraries(_sil_predictor_cpp PRIVATE sil_predictor)

set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages")
install(TARGETS _sil_predictor_cpp
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/amr_motion_control_simulation"
)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/__init__.py"
  "from amr_motion_control_simulation._sil_predictor_cpp import *\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/__init__.py"
  DESTINATION "${PYTHON_INSTALL_DIR}/amr_motion_control_simulation"
)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/pythonpath.sh"
  "export PYTHONPATH=\"${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}:\$PYTHONPATH\"\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/pythonpath.sh"
  DESTINATION "share/${PROJECT_NAME}/hook"
)
ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/pythonpath.sh")

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install library
install(TARGETS sil_predictor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()
