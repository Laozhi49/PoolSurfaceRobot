cmake_minimum_required(VERSION 3.8)
project(pointcloud_to_2dmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 添加这一行以消除FLANN警告
cmake_policy(SET CMP0144 NEW)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)  # 新增
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_eigen REQUIRED)

# 1. 首先创建共享库（用于组件）
add_library(pointcloud_to_2dmap_component SHARED
  src/pointcloud_to_2dmap.cpp
)
target_include_directories(pointcloud_to_2dmap_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pointcloud_to_2dmap_component
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  pcl_ros
  rclcpp_components
)

# 注册为组件
rclcpp_components_register_nodes(pointcloud_to_2dmap_component "pointcloud_to_2dmap::PointCloudTo2DMap")

# 2. 然后创建可执行文件（独立运行）
add_executable(pointcloud_to_2dmap_node
  src/main.cpp
)
target_link_libraries(pointcloud_to_2dmap_node
  pointcloud_to_2dmap_component
)
ament_target_dependencies(pointcloud_to_2dmap_node
  rclcpp
  rclcpp_lifecycle
)

# 安装配置
install(TARGETS
  pointcloud_to_2dmap_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin  # 可执行文件应安装到bin
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  pointcloud_to_2dmap_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # 可选：按项目目录组织
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
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
