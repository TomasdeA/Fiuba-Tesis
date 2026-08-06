cmake_minimum_required(VERSION 3.16)
project(local_mapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_math REQUIRED)
find_package(tf2_ros REQUIRED)

# ── Librería interna ───────────────────────────────────────────────────────────
add_library(local_mapper_lib STATIC
  src/occupancy_mapper.cpp
)

target_include_directories(local_mapper_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(local_mapper_lib
  rclcpp
)

# ── Nodo ejecutable ────────────────────────────────────────────────────────────
add_executable(occupancy_mapper_node
  src/occupancy_mapper_node.cpp
)
target_link_libraries(occupancy_mapper_node local_mapper_lib)
ament_target_dependencies(occupancy_mapper_node
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs message_filters nav_math tf2_ros)

# ── Instalación ────────────────────────────────────────────────────────────────
install(TARGETS occupancy_mapper_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ── Tests ───────────────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_occupancy_mapper test/test_occupancy_mapper.cpp)
  target_link_libraries(test_occupancy_mapper local_mapper_lib)
endif()

ament_package()
