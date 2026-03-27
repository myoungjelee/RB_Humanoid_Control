cmake_minimum_required(VERSION 3.8)
project(rb_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rb_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(rb_estimator_node
  src/estimator_node.cpp
  src/controller_tilt_observer.cpp
)

target_compile_features(rb_estimator_node PUBLIC cxx_std_17)
ament_target_dependencies(rb_estimator_node
  builtin_interfaces
  rb_interfaces
  rcl_interfaces
  rclcpp
  sensor_msgs
)

install(TARGETS rb_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
