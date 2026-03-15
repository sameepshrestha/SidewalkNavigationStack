cmake_minimum_required(VERSION 3.8)
project(frodobot_mppi_critics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(frodobot_mppi_critics SHARED
  src/direction_critic.cpp
)

target_include_directories(frodobot_mppi_critics
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(frodobot_mppi_critics
  rclcpp
  geometry_msgs
  nav2_core
  nav2_mppi_controller
  pluginlib
  tf2_ros
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_mppi_controller plugins.xml)

install(TARGETS frodobot_mppi_critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
