cmake_minimum_required(VERSION 3.10)
project(uav_tunnel_gz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
