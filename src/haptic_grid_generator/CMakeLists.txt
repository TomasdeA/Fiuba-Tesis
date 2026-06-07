cmake_minimum_required(VERSION 3.16)
project(haptic_grid_generator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(haptic_grid_generator_node
  src/haptic_grid_generator_node.cpp
)

ament_target_dependencies(haptic_grid_generator_node
  rclcpp
  custom_interfaces
)

install(TARGETS haptic_grid_generator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
