#include "robot_behavior/action_nodes/compute_path_to_pose_node.hpp"

using namespace std::chrono_literals;

ComputePathToPoseNode::ComputePathToPoseNode(const std::string& name,
                                             const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>("node");
  client_ = rclcpp_action::create_client<ComputePathToPose>(node_, "/compute_path_to_pose");
}

BT::PortsList ComputePathToPoseNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Target pose"),
    BT::OutputPort<nav_msgs::msg::Path>("path", "Computed path")
  };
}

BT::NodeStatus ComputePathToPoseNode::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "ComputePath Start...");
  geometry_msgs::msg::PoseStamped target_pose;
  if (!getInput("goal", target_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "[ComputePathToPose] Missing input [goal]");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_action_server(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "[ComputePathToPose] Action server not available");
    return BT::NodeStatus::FAILURE;
  }

  // 构造 goal
  ComputePathToPose::Goal goal;
  goal.target_pose = target_pose;

  // 设置回调选项
  auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
  future_goal_handle_ = client_->async_send_goal(goal, send_goal_options);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputePathToPoseNode::onRunning()
{
  // 检查 goal handle 是否已经就绪
  if (!future_goal_handle_.valid())
      return BT::NodeStatus::FAILURE;

  // goal handle 已就绪
  if (future_goal_handle_.wait_for(0s) == std::future_status::ready)
  {
      auto goal_handle = future_goal_handle_.get();

      // 如果 result_future_ 还没创建，就创建它
      if (!result_future_.valid())
          result_future_ = client_->async_get_result(goal_handle);

      // 检查动作是否完成
      if (result_future_.wait_for(0s) == std::future_status::ready)
      {
          auto result = result_future_.get();
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
              return BT::NodeStatus::SUCCESS;
          else
              return BT::NodeStatus::FAILURE;
      }

      // 动作还没完成
      return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::RUNNING;
}

void ComputePathToPoseNode::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "[ComputePathToPose] Halted");
  if (future_goal_handle_.valid()) {
    if (auto goal_handle = future_goal_handle_.get()) {
      client_->async_cancel_goal(goal_handle);
    }
  }
}

#include "behaviortree_cpp/bt_factory.h"

// 注册插件
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<ComputePathToPoseNode>("ComputePathToPose");
}
