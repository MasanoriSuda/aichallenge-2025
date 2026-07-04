cmake_minimum_required(VERSION 3.5.0)
project(multi_purpose_mpc_ros)

# Default to C++23
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(osqp REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/boost_commander.cpp
)

ament_auto_add_executable(boost_commander
  src/boost_commander_node.cpp
)

ament_auto_add_executable(mpc_controller_cpp
  src/mpc_controller_cpp.cpp
)
target_include_directories(mpc_controller_cpp PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(mpc_controller_cpp
  osqp::osqp
  yaml-cpp
  ${OpenCV_LIBRARIES}
)

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/run_mpc_controller.bash
  scripts/run_mpc_simulation.bash
  scripts/run_reference_path_visualizer.bash
  scripts/run_reference_velocity_configulator.bash
  scripts/path_constraints_provider.bash
  scripts/mpc_controller
  scripts/mpc_simulation
  scripts/obstacle_parser.py
  scripts/path_constraints_provider
  scripts/publish_initialpose
  scripts/publish_initialpose_gnss
  scripts/reference_path_visualizer
  scripts/reference_velocity_configulator
  scripts/pure_pursuit_trajectory_editor
  scripts/trajectory_editor
  scripts/v2x_position_editor
  DESTINATION lib/${PROJECT_NAME}
)

# create python virtual environment
execute_process(
  COMMAND /usr/bin/python3 -m venv ${CMAKE_INSTALL_PREFIX}/.venv
  RESULT_VARIABLE venv_result
)
if(venv_result)
    message(FATAL_ERROR "Failed to create virtual environment")
endif()

# install python dependencies in virtual environment
execute_process(
    COMMAND ${CMAKE_INSTALL_PREFIX}/.venv/bin/pip install -r ./resources/requirements.txt
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE install_result
)
if(install_result)
    message(FATAL_ERROR "Failed to install python dependencies in virtual environment")
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
  env
  resources
  include
)
