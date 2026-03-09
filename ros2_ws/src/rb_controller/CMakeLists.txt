cmake_minimum_required(VERSION 3.8)
project(rb_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(rb_controller_node
  src/controller_node.cpp
)

target_compile_features(rb_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(rb_controller_node
  rclcpp
  sensor_msgs
)

install(TARGETS rb_controller_node
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

ament_package()
