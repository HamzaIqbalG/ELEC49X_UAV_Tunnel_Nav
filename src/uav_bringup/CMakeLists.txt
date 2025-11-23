cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Build executables
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)

find_package(px4_msgs REQUIRED)

# Offboard control node
add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control
  rclcpp
  px4_msgs
)

# Tunnel navigation node
add_executable(tunnel_nav src/tunnel_nav.cpp)
ament_target_dependencies(tunnel_nav
  rclcpp
  px4_msgs
  sensor_msgs
  nav_msgs
)

install(TARGETS
  offboard_control
  tunnel_nav
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

