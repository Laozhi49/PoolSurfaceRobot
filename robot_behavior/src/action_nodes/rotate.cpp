#include "robot_behavior/action_nodes/rotate.hpp"

Rotate::Rotate(const std::string& name,
               const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>("node");

  client_ = rclcpp_action::create_client<Rotation>(node_, "/rotation");

  RCLCPP_INFO(node_->get_logger(), "[Rotate] constructor called");
}

BT::PortsList Rotate::providedPorts()
{
  return { BT::InputPort<float>("target_angle") };
}

BT::NodeStatus Rotate::onStart()
{
  float angle;
  if (!getInput("target_angle", angle))
  {
    RCLCPP_INFO(node_->get_logger(), "Can not find target_angle");
    return BT::NodeStatus::FAILURE;
  }
    
  RCLCPP_INFO(node_->get_logger(), "Rotate onStart, target=%.2f", angle);

  if (!client_->wait_for_action_server(1s))
    return BT::NodeStatus::FAILURE;

  Rotation::Goal goal;
  goal.target_angle = angle;

  auto send_goal_options = rclcpp_action::Client<Rotation>::SendGoalOptions();
  future_goal_handle_ = client_->async_send_goal(goal, send_goal_options);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Rotate::onRunning()
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

#include "behaviortree_cpp/bt_factory.h"

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<Rotate>("Rotate");
// }

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<Rotate>("Rotate");
}
