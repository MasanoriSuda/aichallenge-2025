cmake_minimum_required(VERSION 3.8)
project(simple_mpc)

# --- Autowareパッケージとしてのセットアップ ---
find_package(autoware_cmake REQUIRED)
autoware_package()

# --- 依存パッケージ自動取得（rclcpp, Eigen, msgsなど） ---
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# --- 外部ライブラリ追加 (OSQP本体 + C++ラッパー) ---
add_subdirectory(vendor/osqp)
add_subdirectory(vendor/OsqpEigen)

# --- OSQPビルドターゲットへの警告回避フラグ（※colcon対応） ---
if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(osqpstatic PRIVATE
    -Wno-error=pedantic
    -Wno-error=parentheses
    -Wno-error=implicit-fallthrough
  )
endif()

# --- 実行ファイル構成 ---
ament_auto_add_executable(simple_mpc
  src/simple_mpc.cpp
  src/csv_loader.cpp
  src/MPC.cpp
  src/odom_loader.cpp
  src/reference_path.cpp
  src/spatial_bicycle_models.cpp
)

# --- ヘッダファイル用のincludeパス追加 ---
target_include_directories(simple_mpc PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/osqp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/OsqpEigen/include
)

# --- リンカ設定（OSQPライブラリ） ---
target_link_libraries(simple_mpc
  osqpstatic
  OsqpEigen
)

# --- パッケージとしての登録 ---
ament_auto_package()
