cmake_minimum_required(VERSION 3.14)
project(depth_grid_encoder)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

# ── depth_to_matrix (pipeline raw: imagen de profundidad → grilla) ──────────
add_executable(depth_to_matrix
    src/depth_to_matrix_node.cpp
    src/depth_utils.cpp
)

target_include_directories(depth_to_matrix PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(depth_to_matrix
    rclcpp
    sensor_msgs
    std_msgs
    custom_interfaces)

# ── obstacle_grid_encoder (pipeline filtered: ObstacleCloud → grilla) ───────
add_executable(obstacle_grid_encoder
    src/obstacle_grid_encoder_node.cpp
)

target_include_directories(obstacle_grid_encoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(obstacle_grid_encoder
    rclcpp
    rcl_interfaces
    sensor_msgs
    std_msgs
    custom_interfaces)

# ── extended_depth_grid (odometría + DepthGrid → DepthGrid con FOV extendido) ─
add_executable(extended_depth_grid
    src/extended_depth_grid_node.cpp
)

target_include_directories(extended_depth_grid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(extended_depth_grid
    rclcpp
    nav_msgs
    std_msgs
    custom_interfaces)

# Instalation
install(TARGETS depth_to_matrix obstacle_grid_encoder extended_depth_grid
    RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
    DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rcl_interfaces sensor_msgs nav_msgs std_msgs)
ament_package()
