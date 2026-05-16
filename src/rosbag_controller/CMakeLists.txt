cmake_minimum_required(VERSION 3.8)
project(rosbag_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ── rosbag_controller_node (hardware-agnostic) ────────────────────────────────
add_executable(rosbag_controller_node
  src/rosbag_controller_node.cpp
)
ament_target_dependencies(rosbag_controller_node rclcpp std_msgs std_srvs)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS rosbag_controller_node
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
