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
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_path_core SHARED
  NO_TARGET_LINK_LIBRARIES
  src/path_core.cpp
)

ament_auto_add_library(${PROJECT_NAME}_awsim_boost_start_dash SHARED
  NO_TARGET_LINK_LIBRARIES
  src/awsim_boost_start_dash.cpp
)

ament_auto_add_library(${PROJECT_NAME}_awsim_control_mode_guard SHARED
  NO_TARGET_LINK_LIBRARIES
  src/awsim_control_mode_guard.cpp
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

ament_auto_add_library(${PROJECT_NAME}_runtime_speed_profile SHARED
  NO_TARGET_LINK_LIBRARIES
  src/runtime_speed_profile.cpp
)

ament_auto_add_library(${PROJECT_NAME}_persistent_osqp SHARED
  NO_TARGET_LINK_LIBRARIES
  src/persistent_osqp.cpp
)
target_include_directories(${PROJECT_NAME}_persistent_osqp PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_persistent_osqp
  osqp::osqp
)

ament_auto_add_library(${PROJECT_NAME}_mpcc_progress SHARED
  NO_TARGET_LINK_LIBRARIES
  src/mpcc_progress.cpp
)
target_include_directories(${PROJECT_NAME}_mpcc_progress PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME}_mpcc_execution_contract SHARED
  NO_TARGET_LINK_LIBRARIES
  src/mpcc_execution_contract.cpp
)

ament_auto_add_library(${PROJECT_NAME}_mpcc_steering_state_contract SHARED
  NO_TARGET_LINK_LIBRARIES
  src/mpcc_steering_state_contract.cpp
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved STATIC
  src/mpcc_rate_resolved.cpp
)
target_include_directories(${PROJECT_NAME}_mpcc_rate_resolved PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_problem STATIC
  src/mpcc_rate_resolved_problem.cpp
)
target_include_directories(${PROJECT_NAME}_mpcc_rate_resolved_problem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_problem
  ${PROJECT_NAME}_mpcc_rate_resolved
  ${PROJECT_NAME}_persistent_osqp
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_execution_artifact STATIC
  src/mpcc_rate_resolved_execution_artifact.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_execution_artifact PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_execution_artifact
  ${PROJECT_NAME}_mpcc_rate_resolved
  ${PROJECT_NAME}_mpcc_execution_contract
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_adapter STATIC
  src/mpcc_rate_resolved_adapter.cpp
)
target_include_directories(${PROJECT_NAME}_mpcc_rate_resolved_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_adapter
  ${PROJECT_NAME}_mpcc_rate_resolved_problem
)

add_library(${PROJECT_NAME}_mpcc_architecture_snapshot STATIC
  src/mpcc_architecture_snapshot.cpp
)
target_include_directories(${PROJECT_NAME}_mpcc_architecture_snapshot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_architecture_snapshot
  ${PROJECT_NAME}_mpcc_rate_resolved_adapter
  ${PROJECT_NAME}_mpcc_execution_contract
  ${PROJECT_NAME}_persistent_osqp
  ${PROJECT_NAME}_recovery_footprint
  yaml-cpp
)

add_library(${PROJECT_NAME}_mpcc_stateless_maneuver STATIC
  src/mpcc_stateless_maneuver.cpp
)
target_include_directories(${PROJECT_NAME}_mpcc_stateless_maneuver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_stateless_maneuver
  ${PROJECT_NAME}_mpcc_architecture_snapshot
  ${PROJECT_NAME}_mpcc_execution_contract
  ${PROJECT_NAME}_mpc_stage_geometry
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_shadow STATIC
  src/mpcc_rate_resolved_shadow.cpp
  src/mpcc_rate_resolved_dynamic_obstacle.cpp
  src/mpcc_rate_resolved_wall_refinement.cpp
)
target_include_directories(${PROJECT_NAME}_mpcc_rate_resolved_shadow PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_shadow
  ${PROJECT_NAME}_mpcc_rate_resolved_adapter
  ${PROJECT_NAME}_mpcc_architecture_snapshot
  ${PROJECT_NAME}_mpcc_rate_resolved_execution_artifact
  ${PROJECT_NAME}_mpcc_progress
  ${PROJECT_NAME}_mpc_stage_geometry
  ${PROJECT_NAME}_recovery_footprint
  ${PROJECT_NAME}_persistent_osqp
  Threads::Threads
)

add_executable(mpcc_qp_replay
  src/mpcc_qp_replay.cpp
)
target_include_directories(mpcc_qp_replay PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(mpcc_qp_replay
  ${PROJECT_NAME}_mpcc_architecture_snapshot
)
install(TARGETS mpcc_qp_replay
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(mpcc_prepared_suffix_replay
  src/mpcc_prepared_suffix_replay.cpp
)
target_include_directories(mpcc_prepared_suffix_replay PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(mpcc_prepared_suffix_replay
  ${PROJECT_NAME}_mpcc_architecture_snapshot
  ${PROJECT_NAME}_mpcc_rate_resolved_shadow
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_adapter
  ${PROJECT_NAME}_race_mpcc_foundation
  ${PROJECT_NAME}_mpcc_rate_resolved_problem
  ${PROJECT_NAME}_persistent_osqp
)
install(TARGETS mpcc_prepared_suffix_replay
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(mpcc_maneuver_replay
  src/mpcc_maneuver_replay.cpp
)
target_link_libraries(mpcc_maneuver_replay
  ${PROJECT_NAME}_mpcc_stateless_maneuver
)
install(TARGETS mpcc_maneuver_replay
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_add_library(${PROJECT_NAME}_latest_only_worker SHARED
  NO_TARGET_LINK_LIBRARIES
  src/latest_only_worker.cpp
)
target_link_libraries(${PROJECT_NAME}_latest_only_worker Threads::Threads)

ament_auto_add_library(${PROJECT_NAME}_bounded_single_job_executor SHARED
  NO_TARGET_LINK_LIBRARIES
  src/bounded_single_job_executor.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_bounded_single_job_executor Threads::Threads)

ament_auto_add_library(${PROJECT_NAME}_overtake_decision_trace SHARED
  NO_TARGET_LINK_LIBRARIES
  src/overtake_decision_trace.cpp
)

ament_auto_add_library(${PROJECT_NAME}_overtake_execution_orchestrator SHARED
  NO_TARGET_LINK_LIBRARIES
  src/overtake_execution_orchestrator.cpp
)
target_link_libraries(${PROJECT_NAME}_overtake_execution_orchestrator
  ${PROJECT_NAME}_mpcc_execution_contract
)

ament_auto_add_library(${PROJECT_NAME}_mpc_waypoint_preview SHARED
  NO_TARGET_LINK_LIBRARIES
  src/mpc_waypoint_preview.cpp
)

ament_auto_add_library(${PROJECT_NAME}_mpc_waypoint_association SHARED
  NO_TARGET_LINK_LIBRARIES
  src/mpc_waypoint_association.cpp
)

ament_auto_add_library(${PROJECT_NAME}_mpc_stage_geometry SHARED
  NO_TARGET_LINK_LIBRARIES
  src/mpc_stage_geometry.cpp
)

ament_auto_add_library(${PROJECT_NAME}_race_mpcc_foundation SHARED
  NO_TARGET_LINK_LIBRARIES
  src/race_mpcc_foundation.cpp
)
target_link_libraries(${PROJECT_NAME}_race_mpcc_foundation
  ${PROJECT_NAME}_mpcc_execution_contract
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_physical_adapter STATIC
  src/mpcc_rate_resolved_physical_adapter.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_physical_adapter
  ${PROJECT_NAME}_mpcc_rate_resolved_execution_artifact
  ${PROJECT_NAME}_race_mpcc_foundation
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

add_library(${PROJECT_NAME}_mpcc_rate_resolved_physical_wall STATIC
  src/mpcc_rate_resolved_physical_wall.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_wall PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_physical_wall
  ${PROJECT_NAME}_mpcc_rate_resolved_execution_artifact
  ${PROJECT_NAME}_mpc_stage_geometry
  ${PROJECT_NAME}_mpcc_execution_contract
  ${PROJECT_NAME}_race_mpcc_foundation
  ${PROJECT_NAME}_recovery_footprint
  Threads::Threads
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_stop_control_lattice STATIC
  src/mpcc_rate_resolved_stop_control_lattice.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_stop_control_lattice PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_stop_control_lattice
  ${PROJECT_NAME}_mpcc_rate_resolved_shadow
  ${PROJECT_NAME}_mpcc_rate_resolved_adapter
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_adapter
  ${PROJECT_NAME}_mpcc_rate_resolved_execution_artifact
  ${PROJECT_NAME}_mpcc_architecture_snapshot
  ${PROJECT_NAME}_mpc_stage_geometry
  ${PROJECT_NAME}_mpcc_execution_contract
  ${PROJECT_NAME}_recovery_footprint
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_certified_plan STATIC
  src/mpcc_rate_resolved_certified_plan.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_certified_plan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_certified_plan
  ${PROJECT_NAME}_mpcc_rate_resolved_execution_artifact
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_wall
  Threads::Threads
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_normal_branch_bank STATIC
  src/mpcc_rate_resolved_normal_branch_bank.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_normal_branch_bank PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_normal_branch_bank
  ${PROJECT_NAME}_mpcc_rate_resolved_certified_plan
  ${PROJECT_NAME}_mpcc_rate_resolved_shadow
  Threads::Threads
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_overtake_branch_bank STATIC
  src/mpcc_rate_resolved_overtake_branch_bank.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_overtake_branch_bank PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_overtake_branch_bank
  ${PROJECT_NAME}_mpcc_rate_resolved_certified_plan
  ${PROJECT_NAME}_mpcc_rate_resolved_shadow
  Threads::Threads
)

add_library(${PROJECT_NAME}_mpcc_overtake_sibling_adoption STATIC
  src/mpcc_overtake_sibling_adoption.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_overtake_sibling_adoption PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_mpcc_overtake_sibling_adoption
  ${PROJECT_NAME}_mpcc_rate_resolved_execution_artifact
)

add_library(${PROJECT_NAME}_mpcc_on_trajectory_connector STATIC
  src/mpcc_on_trajectory_connector.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_on_trajectory_connector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_mpcc_on_trajectory_connector
  ${PROJECT_NAME}_mpcc_rate_resolved_certified_plan
)

add_library(${PROJECT_NAME}_mpcc_latest_state_feedback STATIC
  src/mpcc_latest_state_feedback.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_latest_state_feedback PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_execution_source STATIC
  src/mpcc_rate_resolved_execution_source.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_execution_source PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_execution_source
  ${PROJECT_NAME}_mpcc_rate_resolved_certified_plan
  ${PROJECT_NAME}_mpcc_rate_resolved_retained_revalidation
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_dynamic_proof STATIC
  src/mpcc_rate_resolved_dynamic_proof.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_dynamic_proof PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_dynamic_proof
  ${PROJECT_NAME}_recovery_footprint
  ${PROJECT_NAME}_mpcc_rate_resolved_shadow
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_wall
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_retained_revalidation STATIC
  src/mpcc_rate_resolved_retained_revalidation.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_retained_revalidation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_retained_revalidation
  ${PROJECT_NAME}_mpcc_rate_resolved_certified_plan
  ${PROJECT_NAME}_mpcc_latest_state_feedback
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_adapter
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_wall
  ${PROJECT_NAME}_mpcc_rate_resolved_dynamic_proof
  ${PROJECT_NAME}_mpc_stage_geometry
  ${PROJECT_NAME}_mpcc_execution_contract
  ${PROJECT_NAME}_recovery_footprint
)

add_library(${PROJECT_NAME}_mpcc_architecture_comparison STATIC
  src/mpcc_architecture_comparison.cpp
)
target_include_directories(${PROJECT_NAME}_mpcc_architecture_comparison PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mpcc_architecture_comparison
  ${PROJECT_NAME}_mpcc_architecture_snapshot
  ${PROJECT_NAME}_mpcc_stateless_maneuver
  ${PROJECT_NAME}_mpcc_rate_resolved_shadow
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_adapter
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_wall
  ${PROJECT_NAME}_mpcc_rate_resolved_dynamic_proof
  ${PROJECT_NAME}_mpcc_rate_resolved_stop_control_lattice
  ${PROJECT_NAME}_mpc_stage_geometry
  ${PROJECT_NAME}_mpcc_execution_contract
)

add_executable(mpcc_architecture_compare
  src/mpcc_architecture_compare.cpp
)
target_link_libraries(mpcc_architecture_compare
  ${PROJECT_NAME}_mpcc_architecture_comparison
)
install(TARGETS mpcc_architecture_compare
  DESTINATION lib/${PROJECT_NAME}
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_command_candidate STATIC
  src/mpcc_rate_resolved_command_candidate.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_command_candidate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_command_candidate
  ${PROJECT_NAME}_mpcc_rate_resolved_retained_revalidation
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_production_adapter STATIC
  src/mpcc_rate_resolved_production_adapter.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_production_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_mpcc_rate_resolved_production_adapter
  ${PROJECT_NAME}_mpcc_execution_contract
  ${PROJECT_NAME}_mpc_stage_geometry
  ${PROJECT_NAME}_mpcc_rate_resolved_command_candidate
)

add_library(${PROJECT_NAME}_mpcc_rate_resolved_stop_successor_bundle STATIC
  src/mpcc_rate_resolved_stop_successor_bundle.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_rate_resolved_stop_successor_bundle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  ${PROJECT_NAME}_mpcc_rate_resolved_stop_successor_bundle
  ${PROJECT_NAME}_mpcc_rate_resolved_retained_revalidation
  ${PROJECT_NAME}_mpcc_rate_resolved_certified_plan
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_wall
  ${PROJECT_NAME}_mpcc_execution_contract
)

add_library(${PROJECT_NAME}_mpcc_certified_stop_successor_observation STATIC
  src/mpcc_certified_stop_successor_observation.cpp
)
target_include_directories(
  ${PROJECT_NAME}_mpcc_certified_stop_successor_observation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  ${PROJECT_NAME}_mpcc_certified_stop_successor_observation
  ${PROJECT_NAME}_mpcc_rate_resolved_production_adapter
  ${PROJECT_NAME}_mpcc_rate_resolved_stop_successor_bundle
  ${PROJECT_NAME}_mpcc_execution_contract
)

ament_auto_add_library(${PROJECT_NAME}_recovery_mpc SHARED
  NO_TARGET_LINK_LIBRARIES
  src/recovery_mpc.cpp
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
  ${PROJECT_NAME}_awsim_control_mode_guard
  ${PROJECT_NAME}_path_core
  ${PROJECT_NAME}_mpcc_progress
  ${PROJECT_NAME}_mpcc_rate_resolved_shadow
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_adapter
  ${PROJECT_NAME}_mpcc_rate_resolved_physical_wall
  ${PROJECT_NAME}_mpcc_rate_resolved_certified_plan
  ${PROJECT_NAME}_mpcc_rate_resolved_normal_branch_bank
  ${PROJECT_NAME}_mpcc_rate_resolved_overtake_branch_bank
  ${PROJECT_NAME}_mpcc_overtake_sibling_adoption
  ${PROJECT_NAME}_mpcc_on_trajectory_connector
  ${PROJECT_NAME}_mpcc_rate_resolved_execution_source
  ${PROJECT_NAME}_mpcc_rate_resolved_retained_revalidation
  ${PROJECT_NAME}_mpcc_rate_resolved_dynamic_proof
  ${PROJECT_NAME}_mpcc_stateless_maneuver
  ${PROJECT_NAME}_mpcc_rate_resolved_command_candidate
  ${PROJECT_NAME}_mpcc_rate_resolved_production_adapter
  ${PROJECT_NAME}_mpcc_certified_stop_successor_observation
  ${PROJECT_NAME}_mpcc_execution_contract
  ${PROJECT_NAME}_mpcc_steering_state_contract
  ${PROJECT_NAME}_persistent_osqp
  ${PROJECT_NAME}_bounded_single_job_executor
  ${PROJECT_NAME}_latest_only_worker
  ${PROJECT_NAME}_overtake_decision_trace
  ${PROJECT_NAME}_overtake_execution_orchestrator
  ${PROJECT_NAME}_mpc_state_prediction
  ${PROJECT_NAME}_mpc_stage_geometry
  ${PROJECT_NAME}_race_mpcc_foundation
  ${PROJECT_NAME}_runtime_speed_profile
  ${PROJECT_NAME}_mpc_waypoint_association
  ${PROJECT_NAME}_mpc_waypoint_preview
  ${PROJECT_NAME}_mpc_velocity_limit
  ${PROJECT_NAME}_recovery_footprint
  ${PROJECT_NAME}_recovery_mpc
  ${PROJECT_NAME}_start_grid_grace
  ${PROJECT_NAME}_stuck_recovery_core
  ${PROJECT_NAME}_v2x_overtake_core
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

  ament_add_gtest(test_mpcc_steering_state_contract
    test/test_mpcc_steering_state_contract.cpp
  )
  if(TARGET test_mpcc_steering_state_contract)
    target_link_libraries(test_mpcc_steering_state_contract
      ${PROJECT_NAME}_mpcc_steering_state_contract
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved
    test/test_mpcc_rate_resolved.cpp
  )
  if(TARGET test_mpcc_rate_resolved)
    target_include_directories(test_mpcc_rate_resolved PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_rate_resolved
      ${PROJECT_NAME}_mpcc_rate_resolved
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_problem
    test/test_mpcc_rate_resolved_problem.cpp
  )
  if(TARGET test_mpcc_rate_resolved_problem)
    target_include_directories(test_mpcc_rate_resolved_problem PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_rate_resolved_problem
      ${PROJECT_NAME}_mpcc_rate_resolved_problem
      ${PROJECT_NAME}_persistent_osqp
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_dynamic_obstacle
    test/test_mpcc_rate_resolved_dynamic_obstacle.cpp
  )
  if(TARGET test_mpcc_rate_resolved_dynamic_obstacle)
    target_include_directories(test_mpcc_rate_resolved_dynamic_obstacle PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(test_mpcc_rate_resolved_dynamic_obstacle
      ${PROJECT_NAME}_mpcc_rate_resolved_shadow
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_adapter
    test/test_mpcc_rate_resolved_adapter.cpp
  )
  if(TARGET test_mpcc_rate_resolved_adapter)
    target_include_directories(test_mpcc_rate_resolved_adapter PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_rate_resolved_adapter
      ${PROJECT_NAME}_mpcc_rate_resolved_adapter
      ${PROJECT_NAME}_persistent_osqp
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_shadow
    test/test_mpcc_rate_resolved_shadow.cpp
  )
  if(TARGET test_mpcc_rate_resolved_shadow)
    target_include_directories(test_mpcc_rate_resolved_shadow PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_rate_resolved_shadow
      ${PROJECT_NAME}_mpcc_rate_resolved_shadow
      ${PROJECT_NAME}_mpcc_rate_resolved_physical_adapter
    )
  endif()

  ament_add_gtest(test_mpcc_architecture_snapshot
    test/test_mpcc_architecture_snapshot.cpp
  )
  if(TARGET test_mpcc_architecture_snapshot)
    target_include_directories(test_mpcc_architecture_snapshot PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_architecture_snapshot
      ${PROJECT_NAME}_mpcc_architecture_snapshot
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_dynamic_proof
    test/test_mpcc_rate_resolved_dynamic_proof.cpp
  )
  if(TARGET test_mpcc_rate_resolved_dynamic_proof)
    target_link_libraries(test_mpcc_rate_resolved_dynamic_proof
      ${PROJECT_NAME}_mpcc_rate_resolved_dynamic_proof
    )
  endif()

  ament_add_gtest(test_mpcc_stateless_maneuver
    test/test_mpcc_stateless_maneuver.cpp
  )
  if(TARGET test_mpcc_stateless_maneuver)
    target_include_directories(test_mpcc_stateless_maneuver PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_stateless_maneuver
      ${PROJECT_NAME}_mpcc_stateless_maneuver
    )
  endif()

  ament_add_gtest(test_mpcc_architecture_comparison
    test/test_mpcc_architecture_comparison.cpp
  )
  if(TARGET test_mpcc_architecture_comparison)
    target_include_directories(test_mpcc_architecture_comparison PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_architecture_comparison
      ${PROJECT_NAME}_mpcc_architecture_comparison
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_physical_adapter
    test/test_mpcc_rate_resolved_physical_adapter.cpp
  )
  if(TARGET test_mpcc_rate_resolved_physical_adapter)
    target_include_directories(test_mpcc_rate_resolved_physical_adapter PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_rate_resolved_physical_adapter
      ${PROJECT_NAME}_mpcc_rate_resolved_physical_adapter
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_physical_wall
    test/test_mpcc_rate_resolved_physical_wall.cpp
  )
  if(TARGET test_mpcc_rate_resolved_physical_wall)
    target_include_directories(test_mpcc_rate_resolved_physical_wall PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_rate_resolved_physical_wall
      ${PROJECT_NAME}_mpcc_rate_resolved_physical_wall
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_certified_plan
    test/test_mpcc_rate_resolved_certified_plan.cpp
  )
  if(TARGET test_mpcc_rate_resolved_certified_plan)
    target_include_directories(test_mpcc_rate_resolved_certified_plan PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_rate_resolved_certified_plan
      ${PROJECT_NAME}_mpcc_rate_resolved_certified_plan
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_normal_branch_bank
    test/test_mpcc_rate_resolved_normal_branch_bank.cpp
  )
  if(TARGET test_mpcc_rate_resolved_normal_branch_bank)
    target_include_directories(test_mpcc_rate_resolved_normal_branch_bank PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_rate_resolved_normal_branch_bank
      ${PROJECT_NAME}_mpcc_rate_resolved_normal_branch_bank
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_overtake_branch_bank
    test/test_mpcc_rate_resolved_overtake_branch_bank.cpp
  )
  if(TARGET test_mpcc_rate_resolved_overtake_branch_bank)
    target_include_directories(test_mpcc_rate_resolved_overtake_branch_bank PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_rate_resolved_overtake_branch_bank
      ${PROJECT_NAME}_mpcc_rate_resolved_overtake_branch_bank
    )
  endif()

  ament_add_gtest(test_mpcc_overtake_sibling_adoption
    test/test_mpcc_overtake_sibling_adoption.cpp
  )
  if(TARGET test_mpcc_overtake_sibling_adoption)
    target_link_libraries(test_mpcc_overtake_sibling_adoption
      ${PROJECT_NAME}_mpcc_overtake_sibling_adoption
    )
  endif()

  ament_add_gtest(test_mpcc_on_trajectory_connector
    test/test_mpcc_on_trajectory_connector.cpp
  )
  if(TARGET test_mpcc_on_trajectory_connector)
    target_link_libraries(test_mpcc_on_trajectory_connector
      ${PROJECT_NAME}_mpcc_on_trajectory_connector
    )
  endif()

  ament_add_gtest(test_mpcc_latest_state_feedback
    test/test_mpcc_latest_state_feedback.cpp
  )
  if(TARGET test_mpcc_latest_state_feedback)
    target_link_libraries(test_mpcc_latest_state_feedback
      ${PROJECT_NAME}_mpcc_latest_state_feedback
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_execution_source
    test/test_mpcc_rate_resolved_execution_source.cpp
  )
  if(TARGET test_mpcc_rate_resolved_execution_source)
    target_include_directories(test_mpcc_rate_resolved_execution_source PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_rate_resolved_execution_source
      ${PROJECT_NAME}_mpcc_rate_resolved_execution_source
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_retained_revalidation
    test/test_mpcc_rate_resolved_retained_revalidation.cpp
  )
  if(TARGET test_mpcc_rate_resolved_retained_revalidation)
    target_include_directories(
      test_mpcc_rate_resolved_retained_revalidation PRIVATE
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_mpcc_rate_resolved_retained_revalidation
      ${PROJECT_NAME}_mpcc_rate_resolved_retained_revalidation
      ${PROJECT_NAME}_mpcc_rate_resolved_production_adapter
      ${PROJECT_NAME}_mpcc_rate_resolved_stop_successor_bundle
    )
  endif()

  ament_add_gtest(test_mpcc_rate_resolved_command_candidate
    test/test_mpcc_rate_resolved_command_candidate.cpp
  )
  if(TARGET test_mpcc_rate_resolved_command_candidate)
    target_link_libraries(test_mpcc_rate_resolved_command_candidate
      ${PROJECT_NAME}_mpcc_rate_resolved_command_candidate
      ${PROJECT_NAME}_mpcc_rate_resolved_production_adapter
    )
  endif()

  ament_add_gtest(test_mpcc_certified_stop_successor_observation
    test/test_mpcc_certified_stop_successor_observation.cpp
  )
  if(TARGET test_mpcc_certified_stop_successor_observation)
    target_link_libraries(test_mpcc_certified_stop_successor_observation
      ${PROJECT_NAME}_mpcc_certified_stop_successor_observation
    )
  endif()

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

  ament_add_gtest(test_awsim_control_mode_guard
    test/test_awsim_control_mode_guard.cpp
  )
  if(TARGET test_awsim_control_mode_guard)
    target_link_libraries(test_awsim_control_mode_guard
      ${PROJECT_NAME}_awsim_control_mode_guard
    )
  endif()

  ament_add_gtest(test_external_speed_loss_monitor
    test/test_external_speed_loss_monitor.cpp
  )
  if(TARGET test_external_speed_loss_monitor)
    target_include_directories(test_external_speed_loss_monitor PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
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

  ament_add_gtest(test_runtime_speed_profile
    test/test_runtime_speed_profile.cpp
  )
  if(TARGET test_runtime_speed_profile)
    target_link_libraries(test_runtime_speed_profile
      ${PROJECT_NAME}_runtime_speed_profile
    )
  endif()

  ament_add_gtest(test_persistent_osqp
    test/test_persistent_osqp.cpp
  )
  if(TARGET test_persistent_osqp)
    target_link_libraries(test_persistent_osqp
      ${PROJECT_NAME}_persistent_osqp
    )
  endif()

  ament_add_gtest(test_mpcc_progress
    test/test_mpcc_progress.cpp
  )
  if(TARGET test_mpcc_progress)
    target_link_libraries(test_mpcc_progress
      ${PROJECT_NAME}_mpcc_progress
    )
  endif()

  ament_add_gtest(test_latest_only_worker
    test/test_latest_only_worker.cpp
  )
  if(TARGET test_latest_only_worker)
    target_link_libraries(test_latest_only_worker
      ${PROJECT_NAME}_latest_only_worker
    )
  endif()

  ament_add_gtest(test_bounded_single_job_executor
    test/test_bounded_single_job_executor.cpp
  )
  if(TARGET test_bounded_single_job_executor)
    target_link_libraries(test_bounded_single_job_executor
      ${PROJECT_NAME}_bounded_single_job_executor
    )
  endif()

  ament_add_gtest(test_overtake_decision_trace
    test/test_overtake_decision_trace.cpp
  )
  if(TARGET test_overtake_decision_trace)
    target_link_libraries(test_overtake_decision_trace
      ${PROJECT_NAME}_overtake_decision_trace
    )
  endif()

  ament_add_gtest(test_overtake_execution_orchestrator
    test/test_overtake_execution_orchestrator.cpp
  )
  if(TARGET test_overtake_execution_orchestrator)
    target_link_libraries(test_overtake_execution_orchestrator
      ${PROJECT_NAME}_overtake_execution_orchestrator
    )
  endif()

  ament_add_gtest(test_mpcc_execution_contract
    test/test_mpcc_execution_contract.cpp
  )
  if(TARGET test_mpcc_execution_contract)
    target_link_libraries(test_mpcc_execution_contract
      ${PROJECT_NAME}_mpcc_execution_contract
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

  ament_add_gtest(test_mpc_stage_geometry
    test/test_mpc_stage_geometry.cpp
  )
  if(TARGET test_mpc_stage_geometry)
    target_link_libraries(test_mpc_stage_geometry
      ${PROJECT_NAME}_mpc_stage_geometry
    )
  endif()

  ament_add_gtest(test_race_mpcc_foundation
    test/test_race_mpcc_foundation.cpp
  )
  if(TARGET test_race_mpcc_foundation)
    target_link_libraries(test_race_mpcc_foundation
      ${PROJECT_NAME}_race_mpcc_foundation
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
  ament_add_pytest_test(test_localization_scope
    test/test_localization_scope.py
  )
  ament_add_pytest_test(test_single_authority_source_contract
    test/test_single_authority_source_contract.py
  )
  ament_add_pytest_test(test_mpcc_architecture_audit
    test/test_mpcc_architecture_audit.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools/mpcc_architecture_audit
  )
endif()

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(kaleidoscope
  PACKAGE_DIR tools/kaleidoscope/kaleidoscope
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(localization_scope
  PACKAGE_DIR tools/localization_scope/localization_scope
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(mpcc_architecture_audit
  PACKAGE_DIR tools/mpcc_architecture_audit/mpcc_architecture_audit
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
  scripts/localization_scope
  scripts/mpcc_architecture_audit
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
