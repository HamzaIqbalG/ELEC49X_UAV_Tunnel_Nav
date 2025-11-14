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
find_package(std_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)

# Basic UAV node (for future use - subscribes to sensor data)
add_executable(basic_uav_node src/basic_node.cpp)
ament_target_dependencies(basic_uav_node
  rclcpp
  sensor_msgs
)

# Simple test node for workspace verification
add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node
  rclcpp
  std_msgs
)

# Motor speed controller node (publishes motor speed commands for X3 model)
# Uses actuator_msgs/msg/Actuators for Gazebo Fortress compatibility
add_executable(motor_speed_controller src/motor_speed_controller.cpp)
ament_target_dependencies(motor_speed_controller
  rclcpp
  actuator_msgs
)

install(TARGETS
  basic_uav_node
  test_node
  motor_speed_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

