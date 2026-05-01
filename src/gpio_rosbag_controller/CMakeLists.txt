cmake_minimum_required(VERSION 3.8)
project(gpio_rosbag_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS2 dependencies ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ── libgpiod (modern Linux GPIO, replaces deprecated sysfs and WiringPi) ──────
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

# ── Main executable ────────────────────────────────────────────────────────────
add_executable(gpio_rosbag_controller_node
  src/gpio_interface.cpp
  src/gpio_rosbag_controller_node.cpp
)

target_include_directories(gpio_rosbag_controller_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${LIBGPIOD_INCLUDE_DIRS}
)

target_link_libraries(gpio_rosbag_controller_node
  ${LIBGPIOD_LIBRARIES}
)

ament_target_dependencies(gpio_rosbag_controller_node rclcpp)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS gpio_rosbag_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── Linting ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
