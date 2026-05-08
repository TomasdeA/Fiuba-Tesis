cmake_minimum_required(VERSION 3.16)
project(nav_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_math REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video calib3d features2d)

# ── Librería interna (core puro C++17 + OpenCV) ───────────────────────────────
add_library(nav_odometry_lib STATIC
  src/complementary_filter.cpp
  src/rgbd_tracker.cpp
  src/odometry_estimator.cpp
)

target_include_directories(nav_odometry_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(nav_odometry_lib
  nav_math::nav_math
  ${OpenCV_LIBS}
)

ament_target_dependencies(nav_odometry_lib
  rclcpp
  sensor_msgs
)

# ── Nodo ejecutable (capa ROS2) ───────────────────────────────────────────────
add_executable(odometry_node
  src/odometry_node.cpp
)

target_link_libraries(odometry_node nav_odometry_lib)

ament_target_dependencies(odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  cv_bridge
  image_transport
)

# ── Instalación ───────────────────────────────────────────────────────────────
install(TARGETS odometry_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_complementary_filter
    test/test_complementary_filter.cpp
  )
  target_link_libraries(test_complementary_filter nav_odometry_lib)
endif()

ament_package()
