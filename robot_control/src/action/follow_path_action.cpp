#include "robot_control/robot_control_component.hpp"

namespace robot_control{

rclcpp_action::GoalResponse RobotControlComponent::handle_followpath_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowPath::Goal> goal)
{
  (void)uuid;
  if (goal->path.poses.empty()) {
    RCLCPP_WARN(get_logger(), "[FollowPath] Received empty path, rejecting goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "[FollowPath] Accepted goal with %zu poses", goal->path.poses.size());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotControlComponent::handle_followpath_cancel(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  (void)goal_handle;
  RCLCPP_WARN(get_logger(), "[FollowPath] Goal canceled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotControlComponent::handle_followpath_accepted(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  std::thread{std::bind(&RobotControlComponent::execute_followpath, this, std::placeholders::_1), goal_handle}.detach();
}

void RobotControlComponent::execute_followpath(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
    auto result = std::make_shared<FollowPath::Result>();
    auto goal = goal_handle->get_goal();

    global_path_ = goal->path;

    rclcpp::Rate loop_rate(10); // 10 Hz
    auto twist = geometry_msgs::msg::Twist();

    // 取终点
    auto target_pose = global_path_.poses.back();

    const double goal_tolerance = 0.1; // 10cm 停止阈值

    while (rclcpp::ok())
    {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_WARN(get_logger(), "[FollowPath] Goal canceled during execution");
            return;
        }

        // 计算当前机器人和终点的距离
        double dx = current_pose_.pose.position.x - target_pose.pose.position.x;
        double dy = current_pose_.pose.position.y - target_pose.pose.position.y;
        double dist_to_goal = std::sqrt(dx*dx + dy*dy);

        if (dist_to_goal < goal_tolerance) {
            RCLCPP_INFO(get_logger(), "[FollowPath] Reached goal, distance=%.3f", dist_to_goal);
            break;
        }
        
        // 这里你可以调用你的控制逻辑，把机器人移动到 goal->path.poses[i]
        twist = computeVelocityCommands();

        cmd_vel_pub_->publish(twist);

        // 更新 feedback
        auto feedback = std::make_shared<FollowPath::Feedback>();
        feedback->progress = 50.0;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }

    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub_->publish(twist);

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "[FollowPath] Goal completed successfully");
}

geometry_msgs::msg::Twist RobotControlComponent::computeVelocityCommands()
{
    // 1. 先把全局路径转到 base_link
    auto local_path = transformPathToBaseLink(global_path_);

    if (local_path.poses.empty()) {
        return geometry_msgs::msg::Twist();
    }

    // 2. 找前瞻点
    double lookahead_distance = 0.5; // 可调参数
    auto lookahead_point = findLookaheadPoint(local_path, lookahead_distance);

    // 3. Pure Pursuit 控制公式
    double Ld = std::hypot(lookahead_point.x, lookahead_point.y);
    if (Ld < 1e-6) {
        return geometry_msgs::msg::Twist();
    }

    double linear_vel = 0.2; // 可调参数
    double curvature = 2.0 * lookahead_point.y / (Ld * Ld);
    double angular_vel = linear_vel * curvature;

    // 4. 输出速度
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;
    return cmd_vel;
}


nav_msgs::msg::Path RobotControlComponent::transformPathToBaseLink(const nav_msgs::msg::Path & path)
{
    nav_msgs::msg::Path transformed_path;
    transformed_path.header.frame_id = "base_link";

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
        tf_msg = tf_buffer_->lookupTransform(
            "base_link",
            path.header.frame_id,  // e.g. "map"
            tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform path: %s", ex.what());
        return transformed_path;
    }

    for (auto & pose : path.poses) {
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf2::doTransform(pose, transformed_pose, tf_msg);
        transformed_path.poses.push_back(transformed_pose);
    }
    return transformed_path;
}

geometry_msgs::msg::Point RobotControlComponent::findLookaheadPoint(
    const nav_msgs::msg::Path & path, double lookahead_distance)
{
    for (auto & pose : path.poses) {
        double dx = pose.pose.position.x;
        double dy = pose.pose.position.y;
        double dist = std::hypot(dx, dy);

        if (dist >= lookahead_distance) {
            return pose.pose.position; // 已经在 base_link 下了
        }
    }
    return path.poses.back().pose.position; // 没找到就返回终点
}

} // namespace robot_control