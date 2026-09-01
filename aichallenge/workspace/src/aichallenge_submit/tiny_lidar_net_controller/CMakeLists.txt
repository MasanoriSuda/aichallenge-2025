cmake_minimum_required(VERSION 3.8)
project(tiny_lidar_net_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install entry point script for the node
install(PROGRAMS
  tiny_lidar_net_controller/tiny_lidar_net_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, config and checkpoint files to share
install(DIRECTORY
  launch
  config
  ckpt
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_tiny_lidar_net_contract
    test/test_tiny_lidar_net_contract.py
  )
  ament_add_pytest_test(
    test_speed_governor
    test/test_speed_governor.py
  )
endif()

ament_package()
