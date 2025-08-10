cmake_minimum_required(VERSION 3.8)
project(simple_mpc)

# Autoware 用セットアップ
find_package(autoware_cmake REQUIRED)
autoware_package()

# ament 系
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# --- vendored ライブラリ読み込み ---
add_subdirectory(vendor/osqp)
add_subdirectory(vendor/osqp-eigen)

# ALIAS ターゲット定義（find_packageを使わない）
add_library(OsqpEigen::OsqpEigen ALIAS OsqpEigen)

# simple_mpc 実行ファイル定義
ament_auto_add_executable(simple_mpc
  src/simple_mpc.cpp
  src/MPC.cpp
  src/reference_path.cpp
  src/spatial_bicycle_models.cpp
)

# include パス
target_include_directories(simple_mpc PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/osqp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/osqp-eigen/include
)

# リンクするライブラリ
target_link_libraries(simple_mpc
  osqpstatic
  OsqpEigen::OsqpEigen
)

# パッケージ化
ament_auto_package()
