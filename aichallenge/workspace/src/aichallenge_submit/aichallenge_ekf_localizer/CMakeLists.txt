cmake_minimum_required(VERSION 3.14)
project(aichallenge_ekf_localizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_find_build_dependencies()

ament_auto_add_library(aichallenge_ekf_localizer_lib SHARED
  src/ekf_localizer.cpp
  src/covariance.cpp
  src/mahalanobis.cpp
  src/measurement.cpp
  src/state_transition.cpp
  src/warning_message.cpp
)

target_link_libraries(aichallenge_ekf_localizer_lib Eigen3::Eigen)

ament_auto_add_executable(ekf_localizer src/ekf_localizer_node.cpp)

target_compile_options(ekf_localizer PUBLIC -g -Wall -Wextra -Wpedantic -Werror)

target_link_libraries(ekf_localizer aichallenge_ekf_localizer_lib)
target_include_directories(ekf_localizer PUBLIC include)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  ament_add_gtest(${test_name} ${filepath})
  target_link_libraries("${test_name}" aichallenge_ekf_localizer_lib)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_FILES
    test/test_ekf_clock.cpp
    test/test_aged_object_queue.cpp
    test/test_mahalanobis.cpp
    test/test_measurement.cpp
    test/test_numeric.cpp
    test/test_state_transition.cpp
    test/test_string.cpp
    test/test_warning_message.cpp)

  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
  target_compile_features(test_ekf_clock PRIVATE cxx_std_17)
  target_link_libraries(test_ekf_clock dl)
  target_link_options(test_ekf_clock PRIVATE -Wl,--export-dynamic)
  set_tests_properties(test_ekf_clock PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=224")
endif()


install(FILES LICENSE NOTICE UPSTREAM.json README.md
  DESTINATION share/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
