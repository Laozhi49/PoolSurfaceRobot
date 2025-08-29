#include "pointcloud_to_2dmap/pointcloud_to_2dmap.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // 使用生命周期节点
  auto node = std::make_shared<pointcloud_to_2dmap::PointCloudTo2DMap>(rclcpp::NodeOptions());
  
  // 手动配置和激活节点
  node->configure();
  node->activate();
  
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}