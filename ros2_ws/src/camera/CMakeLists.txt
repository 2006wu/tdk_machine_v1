cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_executable(imread src/imread.cpp)
ament_target_dependencies(imread rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(coffee1 src/coffee1.cpp)
ament_target_dependencies(coffee1 rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

add_executable(coffee2 src/coffee2.cpp)
ament_target_dependencies(coffee2 rclcpp sensor_msgs std_msgs cv_bridge OpenCV image_transport)

add_executable(orange src/orange.cpp)
ament_target_dependencies(orange rclcpp sensor_msgs std_msgs cv_bridge OpenCV image_transport)

add_executable(TFsetup src/TFsetup.cpp)
ament_target_dependencies(TFsetup rclcpp geometry_msgs tf2 tf2_ros)

add_executable(transform src/transform.cpp)
ament_target_dependencies(transform rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)

add_executable(arm src/arm.cpp)
ament_target_dependencies(arm rclcpp std_msgs geometry_msgs)

# 安裝這個執行檔到 ROS 2 的 lib 路徑中
install(TARGETS
  imread
  coffee1
  coffee2
  orange
  TFsetup
  transform
  arm
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
