cmake_minimum_required(VERSION 3.8)
project(auto_marker_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_marker_docking_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(auto_marker_docking_node
  src/auto_marker_docking_node.cpp
  src/aruco_marker_detector.cpp
  src/PD_controller.cpp
  src/aruco_kalman_filter.cpp
)
ament_target_dependencies(auto_marker_docking_node
  auto_marker_docking_interface
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  tf2_eigen
  geometry_msgs
  tf2_ros
)

install(TARGETS auto_marker_docking_node
  DESTINATION lib/${PROJECT_NAME})

install (DIRECTORY
  launch param
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
