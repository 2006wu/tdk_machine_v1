cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === [ Find dependencies ] ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rospi_pre REQUIRED)

# === [ Executable ] ===
add_executable(check_point_4 src/check_point_4.cpp)
add_executable(chassis_controller src/chassis_controller.cpp)

ament_target_dependencies(check_point_4
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

ament_target_dependencies(chassis_controller
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  rospi_pre
)

install(TARGETS
  check_point_4
  chassis_controller
  DESTINATION lib/${PROJECT_NAME}
)

# === [ Testing ] ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
