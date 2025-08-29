#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_interfaces/action/compute_path_to_pose.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using ComputePathToPose = robot_interfaces::action::ComputePathToPose;

class ComputePathToPoseNode : public BT::StatefulActionNode
{
public:
  ComputePathToPoseNode(const std::string& name,
                        const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr client_;

  std::shared_future<rclcpp_action::ClientGoalHandle<ComputePathToPose>::SharedPtr> future_goal_handle_;
  std::shared_future<rclcpp_action::Client<ComputePathToPose>::WrappedResult> result_future_;
};
