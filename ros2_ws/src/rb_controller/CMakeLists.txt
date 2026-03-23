cmake_minimum_required(VERSION 3.8)
project(rb_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EstimatedState.msg"
  DEPENDENCIES std_msgs
)

add_executable(rb_controller_node
  src/controller_node.cpp
  src/controller_tilt_observer.cpp
  src/controller_debug_logger.cpp
  src/controller_stand_utils.cpp
)

add_executable(rb_estimator_node
  src/estimator_node.cpp
  src/controller_tilt_observer.cpp
)

add_executable(rb_safety_node
  src/safety_node.cpp
)

target_compile_features(rb_controller_node PUBLIC cxx_std_17)
target_compile_features(rb_estimator_node PUBLIC cxx_std_17)
target_compile_features(rb_safety_node PUBLIC cxx_std_17)
ament_target_dependencies(rb_controller_node
  rclcpp
  sensor_msgs
)
ament_target_dependencies(rb_estimator_node
  rclcpp
  sensor_msgs
)
ament_target_dependencies(rb_safety_node
  rclcpp
  sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rb_controller_node "${cpp_typesupport_target}")
target_link_libraries(rb_estimator_node "${cpp_typesupport_target}")
target_link_libraries(rb_safety_node "${cpp_typesupport_target}")

install(TARGETS rb_controller_node rb_estimator_node rb_safety_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch/config를 설치해 `ros2 launch rb_controller ...`에서
# 기본 safety 파라미터 파일을 패키지 share 경로로 찾을 수 있게 한다.
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
