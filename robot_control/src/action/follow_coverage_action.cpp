#include "robot_control/robot_control_component.hpp"

namespace robot_control{

rclcpp_action::GoalResponse RobotControlComponent::handle_followcoverage_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowCoverage::Goal> goal)
{
  (void)uuid;
  if (goal->path.poses.empty()) {
    RCLCPP_WARN(get_logger(), "[FollowCoverage] Received empty path, rejecting goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "[FollowCoverage] Accepted goal with %zu poses", goal->path.poses.size());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotControlComponent::handle_followcoverage_cancel(
  const std::shared_ptr<GoalHandleFollowCoverage> goal_handle)
{
  (void)goal_handle;
  RCLCPP_WARN(get_logger(), "[FollowCoverage] Goal canceled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotControlComponent::handle_followcoverage_accepted(
  const std::shared_ptr<GoalHandleFollowCoverage> goal_handle)
{
  std::thread{std::bind(&RobotControlComponent::execute_followcoverage, this, std::placeholders::_1), goal_handle}.detach();
}

void RobotControlComponent::execute_followcoverage(
  const std::shared_ptr<GoalHandleFollowCoverage> goal_handle)
{
    auto result = std::make_shared<FollowCoverage::Result>();
    auto goal = goal_handle->get_goal();

    global_path_ = goal->path;

    rclcpp::Rate loop_rate(10); // 10 Hz
    auto twist = geometry_msgs::msg::Twist();

    // 取终点
    auto target_pose = global_path_.poses.back();

    const double goal_tolerance = 0.1; // 10cm 停止阈值

    pid_init_absolute(&follow_coverage_pid, Kp_, Ki_, Kd_, 30.0, 0.8);
    ffdInit(&follow_coverage_ffd, 1.0, 0.5, 0.3);
    pid_init_antiintegral(&follow_coverage_pid_antiintegral, 2.5, 0.02, 0.5, 1.0, 30.0, 0.8);

    // path_segments_ = splitPathByLength(global_path_, 20);
    path_segments_ = splitPathIntoSegments(goal->path, M_PI/12); // 15° 阈值
    current_segment_index_ = 0;
    startpoint_reach_ = false;  // 先去起点
    
    RCLCPP_INFO(get_logger(), "path_segments_size=%d", path_segments_.size());
    // RCLCPP_INFO(get_logger(), "current_segment_size=%d", path_segments_[current_segment_index_].length());
    // tracking_index_ = 0;
    // to do : 先导航到起点
    
    while (rclcpp::ok())
    {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            cmd_vel_pub_->publish(twist);
            RCLCPP_WARN(get_logger(), "[FollowCoverage] Goal canceled during execution");
            return;
        }

        // 计算当前机器人和终点的距离
        if(current_segment_index_ == path_segments_.size()-1)
        {
          double dx = current_pose_.pose.position.x - target_pose.pose.position.x;
          double dy = current_pose_.pose.position.y - target_pose.pose.position.y;
          double dist_to_goal = std::sqrt(dx*dx + dy*dy);

          if (dist_to_goal < goal_tolerance) {
              RCLCPP_INFO(get_logger(), "[FollowCoverage] Reached goal, distance=%.3f", dist_to_goal);
              break;
          }
        }
        
        // 这里你可以调用你的控制逻辑，把机器人移动到 goal->path.poses[i]
        twist = computeVelocityCommands_coverage();

        cmd_vel_pub_->publish(twist);

        // 更新 feedback
        auto feedback = std::make_shared<FollowCoverage::Feedback>();
        feedback->progress = (current_segment_index_+1)/path_segments_.size()*100.0;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }

    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub_->publish(twist);

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "[FollowCoverage] Goal completed successfully");
}

geometry_msgs::msg::Twist RobotControlComponent::computeVelocityCommands_coverage()
{
    TrackingError err;

    // 县导航到起点
    if(!startpoint_reach_)
    {
      // 先把全局路径转到 base_link
      auto local_path = transformPathToBaseLink(path_segments_[0]);

      if (local_path.poses.empty()) {
          return geometry_msgs::msg::Twist();
      }

      visualize_tracking(path_segments_[0].poses.front().pose.position.x,path_segments_[0].poses.front().pose.position.y,0);

      auto segment_goal = local_path.poses.front().pose.position;
      double dist_to_goal = std::hypot(segment_goal.x, segment_goal.y);
      if (dist_to_goal < 0.3) {  // 0.2m 阈值，可调
          startpoint_reach_ = true;
          RCLCPP_INFO(get_logger(), "navigate to start point finished");
      }

      err = TrackStartPoint_segment(local_path);
    }else
    {
      // 先把全局路径转到 base_link
      auto local_path = transformPathToBaseLink(path_segments_[current_segment_index_]);

      if (local_path.poses.empty()) {
          return geometry_msgs::msg::Twist();
      }

      visualize_tracking(path_segments_[current_segment_index_].poses.back().pose.position.x,path_segments_[current_segment_index_].poses.back().pose.position.y,0);

      auto segment_goal = local_path.poses.back().pose.position;
      double dist_to_goal = std::hypot(segment_goal.x, segment_goal.y);
      // RCLCPP_INFO(get_logger(), "dist_to_goal=%f", dist_to_goal);
      if (dist_to_goal < 0.1) {  // 0.2m 阈值，可调
          current_segment_index_ = current_segment_index_ + 1;
          if(current_segment_index_ >= path_segments_.size()-1)
            current_segment_index_ = path_segments_.size()-1;
          RCLCPP_INFO(get_logger(), "current_segment_index_=%d", current_segment_index_);
          // RCLCPP_INFO(get_logger(), "current_segment_size=%d", path_segments_[current_segment_index_].length());
      }

      // 找前瞻点
      double lookahead_distance = 0.25; // 可调参数
      // auto lookahead_point = findLookaheadPoint(local_path, lookahead_distance);
      err = findLookaheadPoint_segment(local_path, lookahead_distance);
    }
    
    // pid计算
    double angular_vel = pid_absolute(err.lateral_error*0.25 + err.yaw_error, 0.0, &follow_coverage_pid)+forwardfeed(&follow_coverage_ffd, err.yaw_error);
    // double angular_vel = pid_antiintegral_update(err.lateral_error*0.25 + err.yaw_error*0.5, 0.0, &follow_coverage_pid_antiintegral);
    
    double linear_vel = 0.25;
    if(abs(angular_vel)>0.1)  // 转弯速度过大时只进行旋转
      linear_vel = 0;

    // 输出速度
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;
    return cmd_vel;
}

// 跟踪起点
TrackingError RobotControlComponent::TrackStartPoint_segment(
    const nav_msgs::msg::Path & segment)
{
    if (segment.poses.empty()) {
        throw std::runtime_error("Segment is empty");
    }

    // --- 横向误差 ---
    double angle = std::atan2(segment.poses[0].pose.position.y, segment.poses[0].pose.position.x);
    // if(min_dist<0.1)
    //   angle = 0;
    double lateral_error = angle;

    // --- 偏航误差 ---
    double yaw_error = 0.0;

    TrackingError result;
    result.lateral_error = lateral_error;
    result.yaw_error = yaw_error;
    result.lookahead_point = segment.poses[0].pose.position;
    return result;
}

TrackingError RobotControlComponent::findLookaheadPoint_segment(
    const nav_msgs::msg::Path & segment, double lookahead_distance)
{
    if (segment.poses.empty()) {
        throw std::runtime_error("Segment is empty");
    }

    // --- 跟你现在的一样，只不过 segment 是子路径 ---
    size_t closest_index = 0;
    double min_dist = std::hypot(segment.poses[0].pose.position.x,
                                 segment.poses[0].pose.position.y);

    for (size_t i = 1; i < segment.poses.size(); ++i) {
        double dx = segment.poses[i].pose.position.x;
        double dy = segment.poses[i].pose.position.y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_index = i;
        }
    }

    // --- 找前瞻点 ---
    size_t lookahead_index = segment.poses.size() - 1;
    for (size_t i = closest_index; i < segment.poses.size(); ++i) {
        double dx = segment.poses[i].pose.position.x;
        double dy = segment.poses[i].pose.position.y;
        double dist = std::hypot(dx, dy);

        if (dist >= lookahead_distance) {
            lookahead_index = i;
            break;
        }
    }

    // --- 横向误差 ---
    double angle = std::atan2(segment.poses[closest_index].pose.position.y, segment.poses[closest_index].pose.position.x);
    if(min_dist<0.1)
      angle = 0;
    double lateral_error = angle;

    // --- 偏航误差 ---
    double dx = segment.poses[lookahead_index].pose.position.x -
                segment.poses[closest_index].pose.position.x;
    double dy = segment.poses[lookahead_index].pose.position.y -
                segment.poses[closest_index].pose.position.y;

    double yaw_error = 0.0;
    if (std::hypot(dx, dy) > 1e-6) {
        yaw_error = std::atan2(dy, dx);
    }

    TrackingError result;
    result.lateral_error = lateral_error;
    result.yaw_error = yaw_error;
    result.lookahead_point = segment.poses[lookahead_index].pose.position;
    return result;
}

// 将路径分段
std::vector<nav_msgs::msg::Path> RobotControlComponent::splitPathByLength(
    const nav_msgs::msg::Path & full_path, size_t max_points)
{
    std::vector<nav_msgs::msg::Path> segments;
    if (full_path.poses.empty()) return segments;

    nav_msgs::msg::Path segment;
    segment.header = full_path.header;

    for (size_t i = 0; i < full_path.poses.size(); ++i) {
        segment.poses.push_back(full_path.poses[i]);
        if (segment.poses.size() >= max_points) {
            segments.push_back(segment);
            segment.poses.clear();
            segment.header = full_path.header;
        }
    }

    if (!segment.poses.empty()) {
        segments.push_back(segment);
    }

    return segments;
}

// 按折返点分割
std::vector<nav_msgs::msg::Path> RobotControlComponent::splitPathIntoSegments(
    const nav_msgs::msg::Path & full_path, double angle_threshold_rad)
{
    std::vector<nav_msgs::msg::Path> segments;
    if (full_path.poses.empty()) return segments;

    nav_msgs::msg::Path current_segment;
    current_segment.header = full_path.header;
    current_segment.poses.push_back(full_path.poses[0]);

    for (size_t i = 1; i + 1 < full_path.poses.size(); ++i) {
        const auto & prev = full_path.poses[i-1].pose.position;
        const auto & curr = full_path.poses[i].pose.position;
        const auto & next = full_path.poses[i+1].pose.position;

        double dx1 = curr.x - prev.x;
        double dy1 = curr.y - prev.y;
        double dx2 = next.x - curr.x;
        double dy2 = next.y - curr.y;

        double yaw1 = std::atan2(dy1, dx1);
        double yaw2 = std::atan2(dy2, dx2);
        double dtheta = std::fabs(yaw2 - yaw1);
        if (dtheta > M_PI) dtheta = 2*M_PI - dtheta;

        // 转角大于阈值（比如 45°），认为是一段的结束
        current_segment.poses.push_back(full_path.poses[i]);
        if (dtheta > angle_threshold_rad) {
            segments.push_back(current_segment);
            current_segment.poses.clear();
            current_segment.header = full_path.header;
            current_segment.poses.push_back(full_path.poses[i]);
        }
    }

    // 最后一段补上
    current_segment.poses.push_back(full_path.poses.back());
    segments.push_back(current_segment);

    return segments;
}

// 可视化点
void RobotControlComponent::visualize_tracking(double x, double y, double z)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map"; // 你可以改成 "odom" 或 "base_link"
  marker.header.stamp = this->now();
  marker.ns = "points";
  marker.id = 0;  // 同一个 id 会覆盖上一个 marker
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = 1.0;

  // 球的大小
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // 颜色 (RGBA)
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  // marker.lifetime = rclcpp::Duration::from_seconds(0); // 0 表示永久存在

  tracking_marker_pub_->publish(marker);
}

} // namespace robot_control