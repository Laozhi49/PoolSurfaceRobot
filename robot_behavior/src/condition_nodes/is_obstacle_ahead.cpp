#include "robot_behavior/condition_nodes/is_obstacle_ahead.hpp"

IsObstacleAhead::IsObstacleAhead(const std::string& name,
                                 const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
  // 从blackboard里获取共享节点
  // 从 blackboard 里获取 LifecycleNode
  node_ = config.blackboard->get<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>("node");

  sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/ultrasonic_data", 10,
    [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      if (msg->data.size() >= 2) {
        left_ = msg->data[0];
        right_ = msg->data[1];
      }
    });
}

IsObstacleAhead::~IsObstacleAhead()
{

}

BT::NodeStatus IsObstacleAhead::tick()
{
  float threshold = 50;  // 50cm
  if (left_ < threshold || right_ < threshold){
    RCLCPP_INFO(node_->get_logger(), "Obstacle detected! left=%.2f right=%.2f", left_, right_);
    return BT::NodeStatus::SUCCESS;   // 有障碍
  }
  RCLCPP_INFO(node_->get_logger(), "No Obstacle");
  return BT::NodeStatus::FAILURE;     // 无障碍
}

#include "behaviortree_cpp/bt_factory.h"

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<IsObstacleAhead>("IsObstacleAhead");
// }

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<IsObstacleAhead>("IsObstacleAhead");
}
