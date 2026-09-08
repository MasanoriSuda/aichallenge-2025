cmake_minimum_required(VERSION 3.14)
project(racing_kart_gnss_poser)

find_package(autoware_cmake REQUIRED)
autoware_package()

## Find non-ROS library
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

set(GNSS_POSER_HEADERS
  include/gnss_poser/convert.hpp
  include/gnss_poser/gnss_poser_core.hpp
  include/gnss_poser/gnss_stat.hpp
)

ament_auto_add_library(gnss_poser_node SHARED
  src/gnss_poser_core.cpp
  ${GNSS_POSER_HEADERS}
)

target_link_libraries(gnss_poser_node
  Geographic
)

rclcpp_components_register_node(gnss_poser_node
  PLUGIN "gnss_poser::GNSSPoser"
  EXECUTABLE gnss_poser
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_gnss_covariance test/test_gnss_covariance.cpp)
  target_link_libraries(test_gnss_covariance gnss_poser_node Geographic)
  # Run the same package-wide analysis with Google Test macro definitions.
  find_package(ament_cmake_cppcheck REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)
  ament_cppcheck(LIBRARIES googletest INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
