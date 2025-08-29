#include "rclcpp/rclcpp.hpp"
#include "robot_costmap/robot_costmap_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_costmap::RobotCostmapComponent>(rclcpp::NodeOptions());

  node->configure();
  node->activate();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
