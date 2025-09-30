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

    pid_init_absolute(&follow_path_pid, Kp_, Ki_, Kd_, 30.0, 0.8);
    ffdInit(&follow_path_ffd, 1.0, 0.5, 0.3);
    pid_init_antiintegral(&follow_path_pid_antiintegral, 2.5, 0.02, 0.5, 1.0, 30.0, 0.8);

    while (rclcpp::ok())
    {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            cmd_vel_pub_->publish(twist);
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
    double lookahead_distance = 0.25; // 可调参数
    // auto lookahead_point = findLookaheadPoint(local_path, lookahead_distance);
    auto err = findLookaheadPoint(local_path, lookahead_distance);

    // 计算pid
    double angular_vel = pid_absolute(err.lateral_error*0.25 + err.yaw_error, 0.0, &follow_path_pid)+forwardfeed(&follow_path_ffd, err.yaw_error);
    // double angular_vel = pid_antiintegral_update(err.lateral_error*0.25 + err.yaw_error*0.5, 0.0, &follow_path_pid_antiintegral);
    
    double linear_vel = 0.25;
    if(abs(angular_vel)>0.1)  // 转弯速度过大时只进行旋转
      linear_vel = 0;

    // 4. 输出速度
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;
    return cmd_vel;
}

TrackingError RobotControlComponent::findLookaheadPoint(
    const nav_msgs::msg::Path & path, double lookahead_distance)
{
    if (path.poses.empty()) {
        throw std::runtime_error("Path is empty");
    }

    // 1. 找最近点
    size_t closest_index = 0;
    double min_dist = std::hypot(path.poses[0].pose.position.x,
                                 path.poses[0].pose.position.y);

    for (size_t i = 1; i < path.poses.size(); ++i) {
        double dx = path.poses[i].pose.position.x;
        double dy = path.poses[i].pose.position.y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_index = i;
        }
    }

    // 2. 从最近点往后找前瞻点
    size_t lookahead_index = path.poses.size()-1;
    for (size_t i = closest_index; i < path.poses.size(); ++i) {
        double dx = path.poses[i].pose.position.x;
        double dy = path.poses[i].pose.position.y;
        double dist = std::hypot(dx, dy);

        if (dist >= lookahead_distance) {
            lookahead_index = i;
            break;
        }
    }

    // 3. 横向误差（最近点的 y，带符号）
    double angle = std::atan2(path.poses[closest_index].pose.position.y, path.poses[closest_index].pose.position.x);
    if(min_dist<0.05)
      angle = 0;
    double lateral_error = angle;
    // double lateral_error = path.poses[closest_index].pose.position.y;

    // 4. 偏航误差（最近点 → 前瞻点 的方向）
    double dx = path.poses[lookahead_index].pose.position.x -
                path.poses[closest_index].pose.position.x;
    double dy = path.poses[lookahead_index].pose.position.y -
                path.poses[closest_index].pose.position.y;

    double yaw_error = 0.0;
    if (std::hypot(dx, dy) > 1e-6) {   // 有效的向量
        double path_yaw = std::atan2(dy, dx);
        yaw_error = path_yaw;
    } else {
        // 最近点和前瞻点重合 → 已经在终点
        yaw_error = 0.0;
    }

    // 5. 返回
    TrackingError result;
    result.lateral_error = lateral_error;
    result.yaw_error = yaw_error;
    result.lookahead_point = path.poses[lookahead_index].pose.position;
    return result;
}

} // namespace robot_control