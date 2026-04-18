cmake_minimum_required(VERSION 3.16)
project(nav_math)

find_package(ament_cmake REQUIRED)

# ── Librería header-only ───────────────────────────────────────────────────────
add_library(nav_math INTERFACE)

target_include_directories(nav_math INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── Instalación ────────────────────────────────────────────────────────────────
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS nav_math
  EXPORT nav_math_targets
)

ament_export_targets(nav_math_targets HAS_LIBRARY_TARGET)

ament_package()
