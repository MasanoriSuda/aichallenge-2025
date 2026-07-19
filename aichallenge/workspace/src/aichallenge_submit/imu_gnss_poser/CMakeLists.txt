cmake_minimum_required(VERSION 3.8)
project(imu_gnss_poser)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(imu_gnss_poser_heading_reference SHARED
  src/heading_reference.cpp
)

ament_auto_add_executable(imu_gnss_poser_node
  src/imu_gnss_poser_node.cpp
)
target_link_libraries(imu_gnss_poser_node
  imu_gnss_poser_heading_reference
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_heading_reference
    test/test_heading_reference.cpp
  )
  if(TARGET test_heading_reference)
    target_link_libraries(test_heading_reference
      imu_gnss_poser_heading_reference
    )
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  data
)
