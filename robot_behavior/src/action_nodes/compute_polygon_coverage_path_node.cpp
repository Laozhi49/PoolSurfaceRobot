#include "robot_behavior/action_nodes/compute_polygon_coverage_path_node.hpp"

using namespace std::chrono_literals;

ComputePolygonCoveragePathNode::ComputePolygonCoveragePathNode(const std::string& name,
                                             const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>("node");
  client_ = rclcpp_action::create_client<ComputePolygonCoveragePath>(node_, "/compute_polygon_coverage_path");
}

BT::PortsList ComputePolygonCoveragePathNode::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::Point32>>("polygon", "Coverage polygon vertices"),
    BT::OutputPort<nav_msgs::msg::Path>("coverage_path", "Computed coverage_path")
  };
}

BT::NodeStatus ComputePolygonCoveragePathNode::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "ComputeCoveragePath Start...");
  std::vector<geometry_msgs::msg::Point32> polygon;
  if (!getInput("polygon", polygon)) {
    RCLCPP_ERROR(node_->get_logger(), "[ComputePolygonCoveragePath] Missing input [polygon]");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[ComputePolygonCoveragePath] polygon size = %zu", polygon.size());
  for (size_t i = 0; i < polygon.size(); i++) {
    const auto &pt = polygon[i];
    RCLCPP_INFO(node_->get_logger(), "  polygon[%zu] = (%.3f, %.3f, %.3f)", i, pt.x, pt.y, pt.z);
  }

  if (!client_->wait_for_action_server(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "[ComputePolygonCoveragePath] Action server not available");
    return BT::NodeStatus::FAILURE;
  }

  // 构造 goal
  ComputePolygonCoveragePath::Goal goal;
  goal.polygon = polygon;

  RCLCPP_INFO(node_->get_logger(), "[ComputePolygonCoveragePath] goal.polygon size = %zu", goal.polygon.size());
  for (size_t i = 0; i < goal.polygon.size(); i++) {
    const auto &pt = goal.polygon[i];
    RCLCPP_INFO(node_->get_logger(), "  goal.polygon[%zu] = (%.3f, %.3f, %.3f)", i, pt.x, pt.y, pt.z);
  }

  // 设置回调选项
  auto send_goal_options = rclcpp_action::Client<ComputePolygonCoveragePath>::SendGoalOptions();
  future_goal_handle_ = client_->async_send_goal(goal, send_goal_options);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputePolygonCoveragePathNode::onRunning()
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
            result_future_ = std::shared_future<rclcpp_action::Client<ComputePolygonCoveragePath>::WrappedResult>();
            return BT::NodeStatus::RUNNING;
          }

          if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
          {
            setOutput("coverage_path", result.result->path);
            return BT::NodeStatus::SUCCESS;
          } 
          else
          {
            return BT::NodeStatus::FAILURE;
          }
            
      }

      // 动作还没完成
      return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::RUNNING;
}

void ComputePolygonCoveragePathNode::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "[ComputePolygonCoveragePath] Halted");
  if (future_goal_handle_.valid()) {
    if (auto goal_handle = future_goal_handle_.get()) {
      client_->async_cancel_goal(goal_handle);
    }
  }

  future_goal_handle_ = std::shared_future<rclcpp_action::ClientGoalHandle<ComputePolygonCoveragePath>::SharedPtr>();
  result_future_ = std::shared_future<rclcpp_action::Client<ComputePolygonCoveragePath>::WrappedResult>();
}

#include "behaviortree_cpp/bt_factory.h"

// 注册插件
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<ComputePolygonCoveragePathNode>("ComputePolygonCoveragePath");
}
