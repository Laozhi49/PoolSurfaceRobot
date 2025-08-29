#include "robot_behavior/behavior_tree_component.hpp"

namespace robot_behavior{

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
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return;
      }

      status_ = nav_to_pose_tree_->tickOnce();

      if (status_ == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(get_logger(), "BT finished successfully!");
        goal_handle->succeed(result);
        return;
      } else if (status_ == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(get_logger(), "BT failed!");
        goal_handle->abort(result);
        return;
      }

      loop_rate.sleep();
    }
}

} //namespace robot_behavior