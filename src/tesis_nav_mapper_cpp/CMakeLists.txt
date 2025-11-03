cmake_minimum_required(VERSION 3.14)
project(tesis_nav_mapper_cpp)

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
find_package(sensor_msgs REQUIRED)

# Principal executable
add_executable(depth_to_matrix 
    src/depth_to_matrix_node.cpp
)
ament_target_dependencies(depth_to_matrix rclcpp sensor_msgs)

# Instalation
install(TARGETS depth_to_matrix
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
