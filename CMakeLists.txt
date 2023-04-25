cmake_minimum_required(VERSION 3.5)
project(plansys2_gpsr_ros2d2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2_geometry_msgs
  nav2_msgs
  plansys2_msgs
  plansys2_domain_expert
  plansys2_executor
  plansys2_planner
  plansys2_problem_expert
  plansys2_pddl_parser
  ament_index_cpp
  plansys2_bt_actions
  OpenCV
  cv_bridge
)

include_directories(include)

add_library(ros2d2_move_bt_node SHARED src/behavior_tree_nodes/Move.cpp)
list(APPEND plugin_libs ros2d2_move_bt_node)
add_library(ros2d2_move_through_door_bt_node SHARED src/behavior_tree_nodes/MoveThroughDoor.cpp)
list(APPEND plugin_libs ros2d2_move_through_door_bt_node)
add_library(ros2d2_open_door_bt_node SHARED src/behavior_tree_nodes/OpenDoor.cpp)
list(APPEND plugin_libs ros2d2_open_door_bt_node)
add_library(ros2d2_close_door_bt_node SHARED src/behavior_tree_nodes/CloseDoor.cpp)
list(APPEND plugin_libs ros2d2_close_door_bt_node)
add_library(ros2d2_pick_bt_node SHARED src/behavior_tree_nodes/Pick.cpp)
list(APPEND plugin_libs ros2d2_pick_bt_node)
add_library(ros2d2_drop_bt_node SHARED src/behavior_tree_nodes/Drop.cpp)
list(APPEND plugin_libs ros2d2_drop_bt_node)
add_library(ros2d2_give_object_bt_node SHARED src/behavior_tree_nodes/GiveObject.cpp)
list(APPEND plugin_libs ros2d2_give_object_bt_node)
add_library(ros2d2_arrange_object_bt_node SHARED src/behavior_tree_nodes/ArrangeObject.cpp)
list(APPEND plugin_libs ros2d2_arrange_object_bt_node)
add_library(ros2d2_requested_door_opened_bt_node SHARED src/behavior_tree_nodes/RequestedDoorOpened.cpp)
list(APPEND plugin_libs ros2d2_requested_door_opened_bt_node)
add_library(ros2d2_requested_door_closed_bt_node SHARED src/behavior_tree_nodes/RequestedDoorClosed.cpp)
list(APPEND plugin_libs ros2d2_requested_door_closed_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node ${dependencies})

install(DIRECTORY launch pddl behavior_trees_xml map img config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  controller_node
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(tests)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
