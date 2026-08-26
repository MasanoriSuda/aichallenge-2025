cmake_minimum_required(VERSION 3.5)
project(aichallenge_submit_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_localization_timing_contract
    test/test_localization_timing_contract.py
  )
endif()

ament_auto_package(INSTALL_TO_SHARE launch map data config)
