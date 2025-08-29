#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/set_robot_speed.hpp"
#include <chrono>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

class SetSpeed : public BT::SyncActionNode
{
public:
  SetSpeed(const std::string& name,
           const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  rclcpp::Client<robot_interfaces::srv::SetRobotSpeed>::SharedPtr client_;
};
