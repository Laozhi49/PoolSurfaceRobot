cmake_minimum_required(VERSION 3.8)
project(robot_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)


include_directories(include)

add_library(robot_planner_component SHARED
  src/robot_planner_component.cpp
  src/planner/astar_planner.cpp
)

ament_target_dependencies(robot_planner_component
  rclcpp 
  rclcpp_action 
  rclcpp_lifecycle 
  rclcpp_components
  nav_msgs geometry_msgs 
  visualization_msgs
  lifecycle_msgs 
  robot_interfaces
)

# 组件注册
rclcpp_components_register_nodes(robot_planner_component "robot_planner::PlannerComponent")

# 然后创建可执行文件（独立运行）
add_executable(robot_planner_node 
  src/robot_planner_main.cpp
)

target_link_libraries(robot_planner_node
  robot_planner_component
)

ament_target_dependencies(robot_planner_node
  rclcpp
  rclcpp_lifecycle
)

install(TARGETS 
  robot_planner_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  robot_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # 可选：按项目目录组织
)

install(DIRECTORY include/
DESTINATION include)

install(DIRECTORY launch/
DESTINATION share/${PROJECT_NAME}/launch)

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
