#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "robot_interfaces/action/follow_path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using FollowPath = robot_interfaces::action::FollowPath;

class FollowPathNode : public BT::StatefulActionNode
{
public:
  FollowPathNode(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp_action::Client<FollowPath>::SharedPtr client_;

  std::shared_future<rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr> future_goal_handle_;
  std::shared_future<rclcpp_action::Client<FollowPath>::WrappedResult> result_future_;
};
