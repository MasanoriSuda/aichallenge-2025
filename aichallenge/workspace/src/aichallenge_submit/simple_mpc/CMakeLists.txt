cmake_minimum_required(VERSION 3.8)
project(simple_mpc)

# ===== Autoware / ament =====
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===== Eigen =====
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ===== OSQP / OsqpEigen（まずはシステム） =====
find_package(osqp QUIET)          # osqp::osqp
find_package(OsqpEigen QUIET)     # OsqpEigen::OsqpEigen

# ===== フォールバック取得 =====
include(ExternalProject)
set(THIRD_PARTY_PREFIX ${CMAKE_BINARY_DIR}/third_party)

# --- OSQP（未検出ならビルド：静的） ---
if(NOT TARGET osqp::osqp)
  set(OSQP_PREFIX ${THIRD_PARTY_PREFIX}/osqp-install)
  ExternalProject_Add(osqp_ext
    GIT_REPOSITORY https://github.com/osqp/osqp.git
    GIT_TAG        v0.6.3
    UPDATE_DISCONNECTED 1
    SOURCE_DIR     ${THIRD_PARTY_PREFIX}/osqp-src
    BINARY_DIR     ${THIRD_PARTY_PREFIX}/osqp-build
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_INSTALL_PREFIX=${OSQP_PREFIX}
      -DOSQP_BUILD_SHARED_LIB=OFF
      -DOSQP_BUILD_STATIC_LIB=ON
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    BUILD_COMMAND   ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE}
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install --config ${CMAKE_BUILD_TYPE}
  )
  add_library(osqp::osqp STATIC IMPORTED GLOBAL)
  add_dependencies(osqp::osqp osqp_ext)
  set_target_properties(osqp::osqp PROPERTIES
    IMPORTED_LOCATION             ${OSQP_PREFIX}/lib/libosqp.a
    INTERFACE_INCLUDE_DIRECTORIES ${OSQP_PREFIX}/include
  )
endif()

# --- OsqpEigen（未検出ならビルド：静的。IMPORTEDだけ作る※includeは後で通す） ---
set(OSQPEIGEN_PREFIX ${THIRD_PARTY_PREFIX}/osqp-eigen-install)
if(NOT TARGET OsqpEigen::OsqpEigen)
  ExternalProject_Add(osqp_eigen_ext
    GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
    GIT_TAG        v0.8.1
    UPDATE_DISCONNECTED 1
    SOURCE_DIR     ${THIRD_PARTY_PREFIX}/osqp-eigen-src
    BINARY_DIR     ${THIRD_PARTY_PREFIX}/osqp-eigen-build
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_INSTALL_PREFIX=${OSQPEIGEN_PREFIX}
      -DCMAKE_PREFIX_PATH=${OSQP_PREFIX}
      -DBUILD_SHARED_LIBS=OFF
    BUILD_COMMAND   ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE}
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install --config ${CMAKE_BUILD_TYPE}
  )
  # ← ファイルがまだ無くてもOK。存在チェックはリンク時に解決される。
  add_library(OsqpEigen::OsqpEigen STATIC IMPORTED GLOBAL)
  add_dependencies(OsqpEigen::OsqpEigen osqp_eigen_ext)
  set_target_properties(OsqpEigen::OsqpEigen PROPERTIES
    IMPORTED_LOCATION ${OSQPEIGEN_PREFIX}/lib/libOsqpEigen.a
  )
endif()

# ===== 実行ファイル =====
ament_auto_add_executable(simple_mpc
  src/simple_mpc.cpp
  src/MPC.cpp
  src/reference_path.cpp
  src/spatial_bicycle_models.cpp
)

# ===== 自前ヘッダ =====
target_include_directories(simple_mpc PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ===== 警告抑制（GCC/Clangの ignored-qualifiers を無効化） =====
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(simple_mpc PRIVATE
    -Wno-ignored-qualifiers
    -Wno-error=ignored-qualifiers
  )
endif()

# ===== OsqpEigen の include を通す（IMPORTEDに入れず、消失チェックを回避） =====
if(TARGET OsqpEigen::OsqpEigen)
  # システムにある場合：インターフェースから拾えるならSYSTEMで通す
  get_target_property(_osqpeigen_inc OsqpEigen::OsqpEigen INTERFACE_INCLUDE_DIRECTORIES)
  if(_osqpeigen_inc)
    target_include_directories(simple_mpc SYSTEM PRIVATE ${_osqpeigen_inc})
  else()
    # 外部ビルドの場合：インストール先を直接
    target_include_directories(simple_mpc SYSTEM PRIVATE ${OSQPEIGEN_PREFIX}/include)
  endif()
endif()

# ===== リンク（プレーン形式で統一） =====
target_link_libraries(simple_mpc
  OsqpEigen::OsqpEigen
  osqp::osqp
  Eigen3::Eigen
)

# ===== 外部取得の順序保証 =====
if(TARGET osqp_ext)
  add_dependencies(simple_mpc osqp_ext)
endif()
if(TARGET osqp_eigen_ext)
  add_dependencies(simple_mpc osqp_eigen_ext)
endif()

ament_auto_package()
