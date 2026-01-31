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
find_package(std_msgs REQUIRED)
find_package(tesis_nav_interfaces REQUIRED)

# Principal executable
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
    tesis_nav_interfaces)

# Instalation
install(TARGETS depth_to_matrix
    RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
    DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)
  
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs std_msgs)
ament_package()
