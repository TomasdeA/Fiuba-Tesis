cmake_minimum_required(VERSION 3.16)
project(depth_obstacle_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_math REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ── Librería interna ───────────────────────────────────────────────────────────
add_library(depth_obstacle_filter_lib STATIC
  src/depth_projector.cpp
  src/gravity_aligner.cpp
  src/imu_filter.cpp
  src/ground_estimator.cpp
)

target_include_directories(depth_obstacle_filter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(depth_obstacle_filter_lib
  nav_math::nav_math
)

ament_target_dependencies(depth_obstacle_filter_lib
  rclcpp
  sensor_msgs
)

# ── Nodo ejecutable ────────────────────────────────────────────────────────────
add_executable(depth_obstacle_filter_node
  src/depth_obstacle_filter_node.cpp
)
target_link_libraries(depth_obstacle_filter_node depth_obstacle_filter_lib)
ament_target_dependencies(depth_obstacle_filter_node
  rclcpp sensor_msgs std_msgs geometry_msgs tf2_ros)

# ── Instalación ────────────────────────────────────────────────────────────────
install(TARGETS depth_obstacle_filter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ── Tests ──────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_depth_projector test/test_depth_projector.cpp)
  target_link_libraries(test_depth_projector depth_obstacle_filter_lib)
  ament_target_dependencies(test_depth_projector sensor_msgs)

  ament_add_gtest(test_gravity_aligner test/test_gravity_aligner.cpp)
  target_link_libraries(test_gravity_aligner depth_obstacle_filter_lib)

  ament_add_gtest(test_ground_estimator test/test_ground_estimator.cpp)
  target_link_libraries(test_ground_estimator depth_obstacle_filter_lib)
endif()

ament_package()
