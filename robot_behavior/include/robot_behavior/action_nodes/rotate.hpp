#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/rotation.hpp"
#include <chrono>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

class Rotate : public BT::StatefulActionNode
{
public:
  using Rotation = robot_interfaces::action::Rotation;

  Rotate(const std::string& name,
         const BT::NodeConfiguration& config);
         
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  rclcpp_action::Client<Rotation>::SharedPtr client_;
  std::shared_future<rclcpp_action::ClientGoalHandle<Rotation>::SharedPtr> future_goal_handle_;
  std::shared_future<rclcpp_action::Client<Rotation>::WrappedResult> result_future_;
};
