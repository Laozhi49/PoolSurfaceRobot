#include "robot_behavior/behavior_tree_component.hpp"

namespace robot_behavior{

void BehaviorTreeComponent::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received /goal_pose from RViz (x=%.2f, y=%.2f)",
              msg->pose.position.x, msg->pose.position.y);

  if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available!");
    return;
  }

  // 如果有正在跑的 goal，先 cancel（非阻塞）
  if (current_goal_) {
    RCLCPP_WARN(get_logger(), "Canceling previous goal before sending new one");
    nav_to_pose_client_->async_cancel_goal(current_goal_);
    // 不立即 reset，因为 result_callback 可能会在之后被调用；但为简单起见这里可以 reset 指针（可选）
    current_goal_.reset();
    return;
  }
  
  NavigateToPose::Goal goal_msg;
  goal_msg.pose = *msg;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // NOTE: 对于你当前的 rclcpp 版本，goal_response_callback 接受的是
  // ClientGoalHandle<NavigateToPose>::SharedPtr（不是 shared_future）。
  send_goal_options.goal_response_callback =
    [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        this->current_goal_ = goal_handle;  // 保存 handle，便于后续取消
      }
    };

  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(this->get_logger(), "Navigation was canceled");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Navigation failed");
      }
      this->current_goal_.reset();
    };

  // feedback_callback 可选：如果需要 progress feedback，可在此添加

  // 异步发送 goal（不要 .get()）
  nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
}

rclcpp_action::GoalResponse BehaviorTreeComponent::handle_nav_to_pose_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request: (%.2f, %.2f)",
                goal->pose.pose.position.x, goal->pose.pose.position.y);

    // 把 goal_pose 放到 blackboard
    blackboard_->set("goal_pose", goal->pose);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 处理取消请求
rclcpp_action::CancelResponse BehaviorTreeComponent::handle_nav_to_pose_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 处理接收的目标（异步执行任务）
void BehaviorTreeComponent::handle_nav_to_pose_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
    std::thread{std::bind(&BehaviorTreeComponent::execute_nav_to_pose, this, goal_handle)}.detach();
}

// 执行逻辑（这里模拟导航过程）
void BehaviorTreeComponent::execute_nav_to_pose(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Start executing BT for NavigateToPose");

    auto result = std::make_shared<NavigateToPose::Result>();
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        nav_to_pose_tree_->haltTree();
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return;
      }

      status_ = nav_to_pose_tree_->tickOnce();

      if (status_ == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(get_logger(), "BT finished successfully!");
        goal_handle->succeed(result);
        nav_to_pose_tree_->haltTree();
        return;
      } else if (status_ == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(get_logger(), "BT failed!");
        goal_handle->abort(result);
        nav_to_pose_tree_->haltTree();
        return;
      }

      loop_rate.sleep();
    }
}

} //namespace robot_behavior