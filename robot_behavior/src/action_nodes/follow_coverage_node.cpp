#include "robot_behavior/action_nodes/follow_coverage_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
using namespace std::chrono_literals;

FollowCoverageNode::FollowCoverageNode(const std::string& name,
                               const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>("node");
  client_ = rclcpp_action::create_client<FollowCoverage>(node_, "/follow_coverage");
}

BT::PortsList FollowCoverageNode::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("coverage_path", "Path to follow")
  };
}

BT::NodeStatus FollowCoverageNode::onStart()
{
  nav_msgs::msg::Path coverage_path;
  if (!getInput("coverage_path", coverage_path)) {
    RCLCPP_ERROR(node_->get_logger(), "[FollowCoverage] Missing input [coverage_path]");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_action_server(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "[FollowCoverage] Action server not available");
    return BT::NodeStatus::FAILURE;
  }

  FollowCoverage::Goal goal;
  goal.path = coverage_path;

  auto send_goal_options = rclcpp_action::Client<FollowCoverage>::SendGoalOptions();
  future_goal_handle_ = client_->async_send_goal(goal, send_goal_options);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowCoverageNode::onRunning()
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

          // 确认 goal_id 匹配
          if (result.goal_id != goal_handle->get_goal_id()) {
            RCLCPP_WARN(node_->get_logger(), "Received result from old goal, ignoring");
            result_future_ = std::shared_future<rclcpp_action::Client<FollowCoverage>::WrappedResult>();
            return BT::NodeStatus::RUNNING;
          }

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

void FollowCoverageNode::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "[FollowCoverage] Halted");
  if (future_goal_handle_.valid()) {
    if (auto goal_handle = future_goal_handle_.get()) {
      client_->async_cancel_goal(goal_handle);
      // 主动取一次旧结果并丢掉，避免它以后一直冒出来
      auto throwaway_future = client_->async_get_result(goal_handle);
    }
  }
  future_goal_handle_ = std::shared_future<rclcpp_action::ClientGoalHandle<FollowCoverage>::SharedPtr>();
  result_future_ = std::shared_future<rclcpp_action::Client<FollowCoverage>::WrappedResult>();
}

// 注册插件
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<FollowCoverageNode>("FollowCoverage");
}
