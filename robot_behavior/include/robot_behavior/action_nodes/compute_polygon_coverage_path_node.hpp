#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_interfaces/action/compute_polygon_coverage_path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using ComputePolygonCoveragePath = robot_interfaces::action::ComputePolygonCoveragePath;

class ComputePolygonCoveragePathNode : public BT::StatefulActionNode
{
public:
  ComputePolygonCoveragePathNode(const std::string& name,
                        const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp_action::Client<ComputePolygonCoveragePath>::SharedPtr client_;

  std::shared_future<rclcpp_action::ClientGoalHandle<ComputePolygonCoveragePath>::SharedPtr> future_goal_handle_;
  std::shared_future<rclcpp_action::Client<ComputePolygonCoveragePath>::WrappedResult> result_future_;
};
