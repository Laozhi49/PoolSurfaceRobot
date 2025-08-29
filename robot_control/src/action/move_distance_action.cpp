#include "robot_control/robot_control_component.hpp"

namespace robot_control {

rclcpp_action::GoalResponse RobotControlComponent::handle_movedistance_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveDistance::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal: distance=%.2f, speed=%.2f", goal->distance, goal->speed);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotControlComponent::handle_movedistance_cancel(
  const std::shared_ptr<GoalHandleMoveDistance> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Canceling goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotControlComponent::handle_movedistance_accepted(
  const std::shared_ptr<GoalHandleMoveDistance> goal_handle)
{
  std::thread{std::bind(&RobotControlComponent::execute_movedistance, this, std::placeholders::_1), goal_handle}.detach();
}

void RobotControlComponent::execute_movedistance(
  const std::shared_ptr<GoalHandleMoveDistance> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing Forward Distance");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MoveDistance::Feedback>();
  auto result = std::make_shared<MoveDistance::Result>();

  double distance = goal->distance;
  double speed = goal->speed;
  double duration = distance / speed;
  auto start = this->now();

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    auto elapsed = (this->now() - start).seconds();
    feedback->current_distance = elapsed * speed;
    goal_handle->publish_feedback(feedback);

    if (elapsed < duration) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = speed;
      cmd_vel_pub_->publish(cmd);
    } else {
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      result->success = true;
      result->message = "Forward complete";
      goal_handle->succeed(result);
      break;
    }
    rate.sleep();
  }
  RCLCPP_INFO(get_logger(), "Forward Distance completed");
}

} // namespace robot_control
