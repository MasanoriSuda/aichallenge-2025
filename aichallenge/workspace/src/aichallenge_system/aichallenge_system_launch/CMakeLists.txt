cmake_minimum_required(VERSION 3.5)
project(aichallenge_system_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(PROGRAMS
  script/control_mode_adapter.py
  script/object_marker.py
  script/motion_analytics.py
  script/v2x_marker_publisher.py
  script/single_vehicle_empty_v2x_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_single_vehicle_v2x_contract
    test/test_single_vehicle_v2x_contract.py
  )
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
