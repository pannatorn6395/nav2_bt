cmake_minimum_required(VERSION 3.8)

project(nav2_bt)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(obo_nav_msgs REQUIRED)

nav2_package()
include_directories(
  include
)


set(library_name ${PROJECT_NAME})
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  nav2_util
  nav2_behavior_tree
  obo_nav_msgs
)


add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
  obo_nav_msgs
  rclcpp
  behaviortree_cpp_v3
  nav2_behavior_tree
)

# add_library(nav2_compute_path_to_pose_action_bt_node_test SHARED plugins/action/compute_path_to_pose_action_test.cpp)
# list(APPEND plugin_libs nav2_compute_path_to_pose_action_bt_node_test)

add_library(nav2_request_docking_state_service_bt_node SHARED plugins/action/request_docking_state_service.cpp)
list(APPEND plugin_libs nav2_request_docking_state_service_bt_node)

add_library(nav2_dock_action_bt_node SHARED plugins/action/dock_action.cpp)
list(APPEND plugin_libs nav2_dock_action_bt_node)

add_library(nav2_is_blackboard_true_condition_node SHARED plugins/condition/is_blackboard_true_condition.cpp)
list(APPEND plugin_libs nav2_is_blackboard_true_condition_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

target_include_directories(nav2_request_docking_state_service_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${obo_nav_msgs_INCLUDE_DIRS}
)
install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# install(FILES nav2_node_test.xml DESTINATION share/${PROJECT_NAME})


ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})


ament_package()
