cmake_minimum_required(VERSION 3.8)
project(sidewalk_mapping_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(grid_map_filters REQUIRED)

add_executable(sidewalk_grid_node src/sidewalk_grid_node.cpp)

ament_target_dependencies(
  sidewalk_grid_node
  rclcpp 
  sensor_msgs 
  grid_map_ros 
  grid_map_msgs 
  grid_map_core 
  tf2_ros 
  tf2_sensor_msgs
  grid_map_filters
)

# Install the node binary
install(TARGETS sidewalk_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files (like your filters.yaml)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
