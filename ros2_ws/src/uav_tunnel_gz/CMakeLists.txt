cmake_minimum_required(VERSION 3.10)
project(uav_tunnel_gz)

find_package(ament_cmake REQUIRED)
find_package(gz-sim7 REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)

add_library(cmd_vel_plugin SHARED src/cmd_vel_plugin.cc)
target_link_libraries(cmd_vel_plugin
  gz-sim7::gz-sim7
  gz-transport12::core
  gz-msgs9::gz-msgs9
)

install(TARGETS cmd_vel_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
