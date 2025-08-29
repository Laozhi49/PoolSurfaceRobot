# 作为组件运行
ros2 component standalone pointcloud_to_2dmap pointcloud_to_2dmap::PointCloudTo2DMap

# 或作为独立节点运行
ros2 run pointcloud_to_2dmap pointcloud_to_2dmap_node


# 终端1 - 启动组件容器
ros2 run rclcpp_components component_container_mt

# 终端2 - 加载组件
ros2 component load /ComponentManager pointcloud_to_2dmap pointcloud_to_2dmap::PointCloudTo2DMap