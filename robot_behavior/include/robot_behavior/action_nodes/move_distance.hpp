#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/move_distance.hpp"
#include <chrono>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

class MoveDistance_ : public BT::StatefulActionNode
{
public:
  using MoveDistance = robot_interfaces::action::MoveDistance;

  MoveDistance_(const std::string& name,
                    const BT::NodeConfiguration& config);
                    
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  rclcpp_action::Client<MoveDistance>::SharedPtr client_;
  std::shared_future<rclcpp_action::ClientGoalHandle<MoveDistance>::SharedPtr> future_goal_handle_;
  std::shared_future<rclcpp_action::Client<MoveDistance>::WrappedResult> result_future_;
};
