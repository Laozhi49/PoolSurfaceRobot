cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_interfaces REQUIRED)

# 首先创建共享库（用于组件）
add_library(robot_control_component SHARED
  src/robot_control_component.cpp
  src/pid.cpp
  src/action/rotation_action.cpp
  src/action/move_distance_action.cpp
  src/action/follow_path_action.cpp
)

target_include_directories(robot_control_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_control_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  robot_interfaces
)

# 注册为组件
rclcpp_components_register_nodes(robot_control_component "robot_control::RobotControlComponent")

# 然后创建可执行文件（独立运行）
add_executable(robot_control_node 
  src/robot_control_node.cpp
)

target_link_libraries(robot_control_node
  robot_control_component
)

ament_target_dependencies(robot_control_node
  rclcpp
  rclcpp_lifecycle
)

# 安装配置
install(TARGETS
  robot_control_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin  # 可执行文件应安装到bin
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  robot_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # 可选：按项目目录组织
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()