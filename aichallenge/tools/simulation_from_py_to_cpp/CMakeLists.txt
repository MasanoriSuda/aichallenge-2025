cmake_minimum_required(VERSION 3.10)
project(spatial_bicycle_models_test)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  /usr/local/include
  /usr/local/include/osqp
)

link_directories(
  /usr/local/lib
)

add_executable(test_exec
  spatial_bicycle_models.cpp
  reference_path.cpp
  main.cpp
  MPC.cpp
  csv_loader.cpp
  odom_loader.cpp
)

# 🔥 osqpをOsqpEigenより後に置くとリンクエラーになる場合があるので順番を変える！
target_link_libraries(test_exec
  PRIVATE
    Eigen3::Eigen
    OsqpEigen
    osqp
    nlohmann_json::nlohmann_json
)


find_package(nlohmann_json REQUIRED)
