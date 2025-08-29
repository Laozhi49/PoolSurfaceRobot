#include "robot_control/robot_control_component.hpp"

namespace robot_control {

rclcpp_action::GoalResponse RobotControlComponent::handle_rotation_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Rotation::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Received rotation goal: %f degrees", goal->target_angle);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotControlComponent::handle_rotation_cancel(
  const std::shared_ptr<GoalHandleRotation> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotControlComponent::handle_rotation_accepted(
  const std::shared_ptr<GoalHandleRotation> goal_handle)
{
  std::thread{std::bind(&RobotControlComponent::execute_rotation, this, std::placeholders::_1), goal_handle}.detach();
}

void RobotControlComponent::execute_rotation(
  const std::shared_ptr<GoalHandleRotation> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing rotation");
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Rotation::Result>();
  auto feedback = std::make_shared<Rotation::Feedback>();

  rclcpp::Rate loop_rate(50);
  float target_angle = goal->target_angle + yaw_cur;
  float start_angle = yaw_cur;
  float angle_turned = 0.0f;

  int stable_time = 0;

  auto twist = geometry_msgs::msg::Twist();

  pid_init_absolute(&yaw_angle_pid, 0.5f, 0.001f, 0.005f, 10.0f,10.0f);

  RCLCPP_INFO(get_logger(), "yaw_cur:%.6f",yaw_cur);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      result->final_angle = angle_turned;
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Rotation canceled");
      return;
    }

    twist.angular.z = pid_absolute(target_angle,yaw_cur,&yaw_angle_pid) * 0.1f;

    angle_turned = yaw_cur - start_angle;
    feedback->current_angle = angle_turned;

    cmd_vel_pub_->publish(twist);
    goal_handle->publish_feedback(feedback);

    if(abs(target_angle - yaw_cur)<=1.0f)
    {
      RCLCPP_INFO(get_logger(), "target_angle:%.6f,yaw_cur:%.6f",target_angle,yaw_cur);
      stable_time++;
    }else stable_time = 0;

    if(stable_time >= 50) break;

    loop_rate.sleep();
  }

  twist.angular.z = 0.0;
  cmd_vel_pub_->publish(twist);

  result->final_angle = angle_turned;
  goal_handle->succeed(result);
  RCLCPP_INFO(get_logger(), "Rotation completed");
}

} // namespace robot_control
