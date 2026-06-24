cmake_minimum_required(VERSION 3.14)
project(multi_purpose_mpc_ros_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(multi_purpose_mpc_ros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(osqp_interface REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED CONFIG)

add_library(${PROJECT_NAME}_core
  src/controller.cpp
  src/osqp_mpc.cpp
  src/ref_velocity_config.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  ament_index_cpp
  autoware_auto_control_msgs
  geometry_msgs
  multi_purpose_mpc_ros_msgs
  nav_msgs
  osqp_interface
  rcl_interfaces
  rclcpp
  std_msgs
  yaml_cpp_vendor
)

target_link_libraries(${PROJECT_NAME}_core yaml-cpp)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)

add_executable(mpc_controller_cpp
  src/main.cpp
)
target_link_libraries(mpc_controller_cpp
  ${PROJECT_NAME}_core
)

install(TARGETS
  ${PROJECT_NAME}_core
  mpc_controller_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ref_velocity_config
    test/test_ref_velocity_config.cpp
  )
  if(TARGET test_ref_velocity_config)
    target_link_libraries(test_ref_velocity_config ${PROJECT_NAME}_core)
    target_compile_definitions(
      test_ref_velocity_config
      PRIVATE TEST_REF_VEL_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/ref_vel.yaml"
    )
  endif()

  ament_add_gtest(test_reference_path_bounds
    test/test_reference_path_bounds.cpp
  )
  if(TARGET test_reference_path_bounds)
    target_link_libraries(test_reference_path_bounds ${PROJECT_NAME}_core)
    target_compile_definitions(
      test_reference_path_bounds
      PRIVATE TEST_SIMPLE_MAP_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/simple_map.yaml"
    )
  endif()

  ament_add_gtest(test_lap_tracker
    test/test_lap_tracker.cpp
  )
  if(TARGET test_lap_tracker)
    target_link_libraries(test_lap_tracker ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_path_constraint_bounds
    test/test_path_constraint_bounds.cpp
  )
  if(TARGET test_path_constraint_bounds)
    target_link_libraries(test_path_constraint_bounds ${PROJECT_NAME}_core)
  endif()
endif()

ament_package()
