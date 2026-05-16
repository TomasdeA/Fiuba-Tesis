cmake_minimum_required(VERSION 3.8)
project(hardware_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_interfaces REQUIRED)

# ── libgpiod (gpio_button_node) ───────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

add_executable(motor_sweep_uart src/motor_sweep_uart.cpp)
ament_target_dependencies(motor_sweep_uart rclcpp)
add_executable(grid_to_motor src/grid_to_motor.cpp)
ament_target_dependencies(grid_to_motor rclcpp custom_interfaces)

# ── gpio_button_node (Raspberry Pi only, use_hw:=true) ───────────────────────
add_executable(gpio_button_node
  src/gpio_interface.cpp
  src/gpio_button_node.cpp
)
target_include_directories(gpio_button_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${LIBGPIOD_INCLUDE_DIRS}
)
target_link_libraries(gpio_button_node ${LIBGPIOD_LIBRARIES})
ament_target_dependencies(gpio_button_node rclcpp std_msgs std_srvs)

install(TARGETS
  motor_sweep_uart
  grid_to_motor
  gpio_button_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
