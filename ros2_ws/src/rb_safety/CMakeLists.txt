cmake_minimum_required(VERSION 3.8)
project(rb_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rb_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(rb_safety_node
  src/safety_node.cpp
)

target_compile_features(rb_safety_node PUBLIC cxx_std_17)
ament_target_dependencies(rb_safety_node
  rb_interfaces
  rcl_interfaces
  rclcpp
  sensor_msgs
)

install(TARGETS rb_safety_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
