cmake_minimum_required(VERSION 3.8)
project(hardware_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(tesis_nav_interfaces REQUIRED)

add_executable(motor_sweep_uart src/motor_sweep_uart.cpp)
ament_target_dependencies(motor_sweep_uart rclcpp)
add_executable(grid_to_motor src/grid_to_motor.cpp)
ament_target_dependencies(grid_to_motor rclcpp tesis_nav_interfaces)

install(TARGETS
  motor_sweep_uart
  grid_to_motor
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
