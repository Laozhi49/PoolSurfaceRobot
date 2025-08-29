#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class IsObstacleAhead : public BT::ConditionNode
{
public:
  IsObstacleAhead(const std::string& name,
                  const BT::NodeConfiguration& config);

  ~IsObstacleAhead() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() { return {}; }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  
  float left_{100.0}, right_{100.0};
};
