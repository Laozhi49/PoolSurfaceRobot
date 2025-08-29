cmake_minimum_required(VERSION 3.8)
project(robot_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  std_msgs
  geometry_msgs
  nav2_msgs
  behaviortree_cpp
  robot_interfaces
  rclcpp_components
)

# BT 插件库（独立 SHARED library）

# 条件节点
add_library(is_obstacle_ahead SHARED src/condition_nodes/is_obstacle_ahead.cpp)
ament_target_dependencies(is_obstacle_ahead ${dependencies})

# 动作节点
add_library(set_speed SHARED src/action_nodes/set_speed.cpp)
ament_target_dependencies(set_speed ${dependencies})

add_library(rotate SHARED src/action_nodes/rotate.cpp)
ament_target_dependencies(rotate ${dependencies})

add_library(move_distance SHARED src/action_nodes/move_distance.cpp)
ament_target_dependencies(move_distance ${dependencies})

add_library(toggle_direction SHARED src/action_nodes/toggle_direction.cpp)
ament_target_dependencies(toggle_direction ${dependencies})

add_library(compute_path_to_pose_node SHARED src/action_nodes/compute_path_to_pose_node.cpp)
ament_target_dependencies(compute_path_to_pose_node ${dependencies})

add_library(follow_path_node SHARED src/action_nodes/follow_path_node.cpp)
ament_target_dependencies(follow_path_node ${dependencies})

# 安装插件库
install(TARGETS
  is_obstacle_ahead
  set_speed
  rotate
  move_distance
  toggle_direction
  compute_path_to_pose_node
  follow_path_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ----------- BehaviorTreeComponent 作为组件库 -----------
add_library(behavior_tree_component SHARED 
  src/behavior_tree_component.cpp
  src/NavigateToPose_Action.cpp
)
ament_target_dependencies(behavior_tree_component ${dependencies})

# 注册 component（对应 cpp 里的 RCLCPP_COMPONENTS_REGISTER_NODE(robot_behavior::BehaviorTreeComponent)）
rclcpp_components_register_nodes(behavior_tree_component "robot_behavior::BehaviorTreeComponent")

install(TARGETS
  behavior_tree_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# --------- main_bt 可执行文件 -----------
#add_executable(main_bt src/main_bt.cpp)
#ament_target_dependencies(main_bt ${dependencies})

#target_link_libraries(main_bt
#  is_obstacle_ahead
#  set_speed
#  rotate
#  move_distance
#  toggle_direction
#)

#install(TARGETS
#  main_bt
#  DESTINATION lib/${PROJECT_NAME})

# 创建可执行文件（独立运行）
add_executable(robot_behavior_node 
  src/main.cpp
)

target_link_libraries(robot_behavior_node
  behavior_tree_component
)

ament_target_dependencies(robot_behavior_node
  rclcpp
  rclcpp_lifecycle
)

install(TARGETS
  robot_behavior_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # 可选：按项目目录组织
)

# 安装 BT XML
install(DIRECTORY behavior_trees
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
