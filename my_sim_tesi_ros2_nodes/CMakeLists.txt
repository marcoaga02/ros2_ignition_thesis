cmake_minimum_required(VERSION 3.8)
project(my_sim_tesi_ros2_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(my_sim_tesi_ros2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosgraph_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(action_server SHARED
  src/pose_control_action_server.cpp)

add_library(action_client SHARED
  src/pose_control_action_client.cpp)

add_library(orchestrator SHARED
  src/orchestrator_node.cpp)

target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(orchestrator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(action_server
  "my_sim_tesi_ros2_interfaces"
  "rclcpp"
  "rclcpp_action"
  "nav_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "rclcpp_components"
  "tf2_geometry_msgs")

ament_target_dependencies(action_client
  "my_sim_tesi_ros2_interfaces"
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "rclcpp_components")

ament_target_dependencies(orchestrator
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "rosgraph_msgs"
  "rclcpp_components"
  "tf2")

rclcpp_components_register_node(action_server 
  PLUGIN "my_sim_tesi_ros2_nodes::PoseControlActionServer" 
  EXECUTABLE pose_control_action_server)

rclcpp_components_register_node(action_client
  PLUGIN "my_sim_tesi_ros2_nodes::PoseControlActionClient" 
  EXECUTABLE pose_control_action_client)

rclcpp_components_register_node(orchestrator
  PLUGIN "my_sim_tesi_ros2_nodes::OrchestratorNode" 
  EXECUTABLE orchestrator_node)

install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  orchestrator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_package()
