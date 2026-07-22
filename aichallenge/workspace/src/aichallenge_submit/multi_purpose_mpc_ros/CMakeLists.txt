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

ament_auto_add_library(${PROJECT_NAME}_path_core SHARED
  NO_TARGET_LINK_LIBRARIES
  src/path_core.cpp
)

ament_auto_add_library(${PROJECT_NAME}_awsim_boost_start_dash SHARED
  NO_TARGET_LINK_LIBRARIES
  src/awsim_boost_start_dash.cpp
)

ament_auto_add_library(${PROJECT_NAME}_start_grid_grace SHARED
  NO_TARGET_LINK_LIBRARIES
  src/start_grid_grace.cpp
)

ament_auto_add_library(${PROJECT_NAME}_mpc_velocity_limit SHARED
  NO_TARGET_LINK_LIBRARIES
  src/mpc_velocity_limit.cpp
)

ament_auto_add_library(${PROJECT_NAME}_mpc_state_prediction SHARED
  NO_TARGET_LINK_LIBRARIES
  src/mpc_state_prediction.cpp
)

ament_auto_add_library(${PROJECT_NAME}_mpc_waypoint_preview SHARED
  NO_TARGET_LINK_LIBRARIES
  src/mpc_waypoint_preview.cpp
)

ament_auto_add_library(${PROJECT_NAME}_mpc_waypoint_association SHARED
  NO_TARGET_LINK_LIBRARIES
  src/mpc_waypoint_association.cpp
)

ament_auto_add_library(${PROJECT_NAME}_v2x_overtake_core SHARED
  NO_TARGET_LINK_LIBRARIES
  src/v2x_overtake_core.cpp
)

ament_auto_add_library(${PROJECT_NAME}_stuck_recovery_core SHARED
  NO_TARGET_LINK_LIBRARIES
  src/stuck_recovery_core.cpp
)

ament_auto_add_library(${PROJECT_NAME}_recovery_footprint SHARED
  NO_TARGET_LINK_LIBRARIES
  src/recovery_footprint.cpp
)

ament_auto_add_library(${PROJECT_NAME}_recovery_mpc SHARED
  NO_TARGET_LINK_LIBRARIES
  src/recovery_mpc.cpp
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
  ${PROJECT_NAME}_awsim_boost_start_dash
  ${PROJECT_NAME}_path_core
  ${PROJECT_NAME}_mpc_state_prediction
  ${PROJECT_NAME}_mpc_waypoint_association
  ${PROJECT_NAME}_mpc_waypoint_preview
  ${PROJECT_NAME}_mpc_velocity_limit
  ${PROJECT_NAME}_recovery_footprint
  ${PROJECT_NAME}_recovery_mpc
  ${PROJECT_NAME}_start_grid_grace
  ${PROJECT_NAME}_stuck_recovery_core
  ${PROJECT_NAME}_v2x_overtake_core
  osqp::osqp
  yaml-cpp
  ${OpenCV_LIBRARIES}
)

ament_auto_add_executable(reference_path_validator
  NO_TARGET_LINK_LIBRARIES
  src/reference_path_validator.cpp
)
target_link_libraries(reference_path_validator
  ${PROJECT_NAME}_path_core
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_path_core
    test/test_path_core.cpp
  )
  if(TARGET test_path_core)
    target_link_libraries(test_path_core
      ${PROJECT_NAME}_path_core
    )
    target_compile_definitions(test_path_core PRIVATE
      MULTI_PURPOSE_MPC_ROS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
  endif()

  ament_add_gtest(test_awsim_boost_start_dash
    test/test_awsim_boost_start_dash.cpp
  )
  if(TARGET test_awsim_boost_start_dash)
    target_link_libraries(test_awsim_boost_start_dash
      ${PROJECT_NAME}_awsim_boost_start_dash
    )
  endif()

  ament_add_gtest(test_start_grid_grace
    test/test_start_grid_grace.cpp
  )
  if(TARGET test_start_grid_grace)
    target_link_libraries(test_start_grid_grace
      ${PROJECT_NAME}_start_grid_grace
    )
  endif()

  ament_add_gtest(test_mpc_velocity_limit
    test/test_mpc_velocity_limit.cpp
  )
  if(TARGET test_mpc_velocity_limit)
    target_link_libraries(test_mpc_velocity_limit
      ${PROJECT_NAME}_mpc_velocity_limit
    )
  endif()

  ament_add_gtest(test_mpc_state_prediction
    test/test_mpc_state_prediction.cpp
  )
  if(TARGET test_mpc_state_prediction)
    target_link_libraries(test_mpc_state_prediction
      ${PROJECT_NAME}_mpc_state_prediction
    )
  endif()

  ament_add_gtest(test_mpc_waypoint_preview
    test/test_mpc_waypoint_preview.cpp
  )
  if(TARGET test_mpc_waypoint_preview)
    target_link_libraries(test_mpc_waypoint_preview
      ${PROJECT_NAME}_mpc_waypoint_preview
    )
  endif()

  ament_add_gtest(test_mpc_waypoint_association
    test/test_mpc_waypoint_association.cpp
  )
  if(TARGET test_mpc_waypoint_association)
    target_link_libraries(test_mpc_waypoint_association
      ${PROJECT_NAME}_mpc_waypoint_association
    )
  endif()

  ament_add_gtest(test_v2x_overtake_core
    test/test_v2x_overtake_core.cpp
  )
  if(TARGET test_v2x_overtake_core)
    target_link_libraries(test_v2x_overtake_core
      ${PROJECT_NAME}_v2x_overtake_core
    )
  endif()

  ament_add_gtest(test_stuck_recovery_core
    test/test_stuck_recovery_core.cpp
  )
  if(TARGET test_stuck_recovery_core)
    target_link_libraries(test_stuck_recovery_core
      ${PROJECT_NAME}_stuck_recovery_core
    )
  endif()

  ament_add_gtest(test_recovery_footprint
    test/test_recovery_footprint.cpp
  )
  if(TARGET test_recovery_footprint)
    target_link_libraries(test_recovery_footprint
      ${PROJECT_NAME}_recovery_footprint
    )
  endif()

  ament_add_gtest(test_recovery_mpc
    test/test_recovery_mpc.cpp
  )
  if(TARGET test_recovery_mpc)
    target_link_libraries(test_recovery_mpc
      ${PROJECT_NAME}_recovery_mpc
    )
  endif()

  add_test(
    NAME test_reference_path_validator_rejects_nonfinite_derived_metric
    COMMAND ${CMAKE_COMMAND}
      -DVALIDATOR=$<TARGET_FILE:reference_path_validator>
      -DCSV=${CMAKE_CURRENT_SOURCE_DIR}/test/data/derived_metric_overflow.csv
      -DEXPECTED_EXIT=2
      -DEXPECTED_ERROR=lateral_acceleration_mps2
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/assert_process_result.cmake
  )

  add_test(
    NAME test_trajectory_editor_cpp_contract
    COMMAND ${CMAKE_COMMAND} -E env
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}"
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_trajectory_cpp_contract.py
      --validator $<TARGET_FILE:reference_path_validator>
  )

  ament_add_pytest_test(test_v2x_vehicle_tracker
    test/test_v2x_vehicle_tracker.py
  )
  ament_add_pytest_test(test_reference_path_utils
    test/test_reference_path_utils.py
  )
  ament_add_pytest_test(test_trajectory_contract
    test/test_trajectory_contract.py
  )
  ament_add_pytest_test(test_trajectory_editor_compat
    test/test_trajectory_editor_compat.py
  )
  ament_add_pytest_test(test_trajectory_clearance_dialog
    test/test_trajectory_clearance_dialog.py
  )
  ament_add_pytest_test(test_trajectory_clearance
    test/test_trajectory_clearance.py
  )
  ament_add_pytest_test(test_trajectory_processing
    test/test_trajectory_processing.py
  )
  ament_add_pytest_test(test_trajectory_plot
    test/test_trajectory_plot.py
  )
  ament_add_pytest_test(test_trajectory_speed
    test/test_trajectory_speed.py
  )
endif()

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(kaleidoscope
  PACKAGE_DIR tools/kaleidoscope/kaleidoscope
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
