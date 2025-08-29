#include "rclcpp/rclcpp.hpp"
#include "robot_behavior/behavior_tree_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_behavior::BehaviorTreeComponent>(rclcpp::NodeOptions());

  // 手动配置和激活节点
  node->configure();
  node->activate();
  
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}