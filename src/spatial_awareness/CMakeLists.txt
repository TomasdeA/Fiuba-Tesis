cmake_minimum_required(VERSION 3.16)
project(spatial_awareness)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(nav_math REQUIRED)

add_library(spatial_awareness_lib STATIC
  src/risk_evaluator.cpp
)
target_include_directories(spatial_awareness_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(spatial_awareness_lib PUBLIC nav_math::nav_math)

add_executable(spatial_awareness_node
  src/spatial_awareness_node.cpp
)
target_link_libraries(spatial_awareness_node spatial_awareness_lib)
ament_target_dependencies(spatial_awareness_node
  rclcpp nav_msgs geometry_msgs std_msgs visualization_msgs custom_interfaces nav_math)

install(TARGETS spatial_awareness_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_risk_evaluator
    test/test_risk_evaluator.cpp
  )
  target_link_libraries(test_risk_evaluator spatial_awareness_lib)
endif()

ament_package()
