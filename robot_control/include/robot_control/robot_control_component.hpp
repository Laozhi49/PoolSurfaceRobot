#ifndef ROBOT_CONTROL_COMPONENT_HPP
#define ROBOT_CONTROL_COMPONENT_HPP

#include <memory>
#include <cmath>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/srv/get_robot_pose.hpp"
#include "robot_interfaces/srv/set_robot_speed.hpp"
#include "robot_interfaces/action/rotation.hpp"
#include "robot_interfaces/action/move_distance.hpp"
#include "robot_interfaces/action/follow_path.hpp"
#include "robot_interfaces/action/follow_coverage.hpp"

#include "robot_control/pid.hpp"

namespace robot_control{

struct TrackingError
{
    double lateral_error;  // 横向误差
    double yaw_error;      // 偏航误差
    geometry_msgs::msg::Point lookahead_point; // 保留前瞻点坐标
};

class RobotControlComponent : public rclcpp_lifecycle::LifecycleNode
{
public:
  using Rotation = robot_interfaces::action::Rotation;
  using GoalHandleRotation = rclcpp_action::ServerGoalHandle<Rotation>;

  using MoveDistance = robot_interfaces::action::MoveDistance;
  using GoalHandleMoveDistance = rclcpp_action::ServerGoalHandle<MoveDistance>;

  using FollowPath = robot_interfaces::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

  using FollowCoverage = robot_interfaces::action::FollowCoverage;
  using GoalHandleFollowCoverage = rclcpp_action::ServerGoalHandle<FollowCoverage>;

  explicit RobotControlComponent(const rclcpp::NodeOptions & options);

  // 生命周期状态回调
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // 属性
  float speed_x_;
  float speed_z_tar_;
  float speed_z_cur_;
  float yaw_;
  float roll_;
  float pitch_;
  float wz_;

  float yaw_cur;
  float yaw_last;
  int   circle;

  // pid
  PID_AbsoluteType yaw_angle_pid;

  double Kp_, Ki_, Kd_;
  PID_AbsoluteType follow_path_pid;
  PID_AntiIntegralType follow_path_pid_antiintegral;
  ForwardFeed follow_path_ffd;

  PID_AbsoluteType follow_coverage_pid;
  PID_AntiIntegralType follow_coverage_pid_antiintegral;
  ForwardFeed follow_coverage_ffd;

  // 参数
  std::string cmd_vel_topic_;
  std::string camera_pose_topic_;
  std::string aruco_pose_topic_;
  std::string projected_marker_topic_;
  std::string imu_topic_;
  std::string map_frame_;
  std::string robot_frame_;
  std::string camera_frame_;
  std::string aruco_marker_frame_;
  
  std::string pose_service_name_;
  std::string speed_service_name_;
  std::string rotation_action_name_;
  std::string movedistnace_action_name_;
  std::string followpath_action_name_;
  std::string followcoverage_action_name_;

  geometry_msgs::msg::PoseStamped current_pose_;
  sensor_msgs::msg::Imu imu_data_;

  double goal_tolerance_;

  nav_msgs::msg::Path global_path_;
  bool startpoint_reach_;
  size_t current_segment_index_;
  std::vector<nav_msgs::msg::Path> path_segments_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void yaw_angle_continue(float angle);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & params);

  // ROS2接口
  rclcpp::Service<robot_interfaces::srv::GetRobotPose>::SharedPtr get_pose_service_;
  rclcpp::Service<robot_interfaces::srv::SetRobotSpeed>::SharedPtr set_speed_service_;

  rclcpp_action::Server<Rotation>::SharedPtr rotation_action_server_;
  rclcpp_action::Server<MoveDistance>::SharedPtr movedistance_action_server_;
  rclcpp_action::Server<FollowPath>::SharedPtr followpath_action_server_;
  rclcpp_action::Server<FollowCoverage>::SharedPtr followcoverage_action_server_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_sub_;

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr projected_marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr tracking_marker_pub_;
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // 回调函数
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void visualizeProjectedPose(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
    const geometry_msgs::msg::PoseStamped &pose_in,
    const std::string &frame_id,
    rclcpp::Clock::SharedPtr clock);

  void handle_get_pose(
    const std::shared_ptr<robot_interfaces::srv::GetRobotPose::Request>,
    std::shared_ptr<robot_interfaces::srv::GetRobotPose::Response> response);
  void handle_set_speed(
    const std::shared_ptr<robot_interfaces::srv::SetRobotSpeed::Request> request,
    std::shared_ptr<robot_interfaces::srv::SetRobotSpeed::Response> response);
  
  // 动作服务器相关函数
  // rotation
  rclcpp_action::GoalResponse handle_rotation_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Rotation::Goal> goal);
  rclcpp_action::CancelResponse handle_rotation_cancel(
    const std::shared_ptr<GoalHandleRotation> goal_handle);
  void handle_rotation_accepted(const std::shared_ptr<GoalHandleRotation> goal_handle);
  void execute_rotation(const std::shared_ptr<GoalHandleRotation> goal_handle);

  // move_distance
  rclcpp_action::GoalResponse handle_movedistance_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveDistance::Goal> goal);
  rclcpp_action::CancelResponse handle_movedistance_cancel(
    const std::shared_ptr<GoalHandleMoveDistance> goal_handle);
  void handle_movedistance_accepted(const std::shared_ptr<GoalHandleMoveDistance> goal_handle);
  void execute_movedistance(const std::shared_ptr<GoalHandleMoveDistance> goal_handle);

  // follow_path
  rclcpp_action::GoalResponse handle_followpath_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowPath::Goal> goal);
  rclcpp_action::CancelResponse handle_followpath_cancel(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  void handle_followpath_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  void execute_followpath(const std::shared_ptr<GoalHandleFollowPath> goal_handle);

  geometry_msgs::msg::Twist computeVelocityCommands();
  TrackingError findLookaheadPoint(const nav_msgs::msg::Path & path, double lookahead_distance);

  // follow_coverage
  rclcpp_action::GoalResponse handle_followcoverage_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowCoverage::Goal> goal);
  rclcpp_action::CancelResponse handle_followcoverage_cancel(
    const std::shared_ptr<GoalHandleFollowCoverage> goal_handle);
  void handle_followcoverage_accepted(const std::shared_ptr<GoalHandleFollowCoverage> goal_handle);
  void execute_followcoverage(const std::shared_ptr<GoalHandleFollowCoverage> goal_handle);
  
  geometry_msgs::msg::Twist computeVelocityCommands_coverage();
  TrackingError findLookaheadPoint_segment(
    const nav_msgs::msg::Path & segment, double lookahead_distance);
  TrackingError TrackStartPoint_segment(
    const nav_msgs::msg::Path & segment);
  std::vector<nav_msgs::msg::Path> splitPathByLength(
    const nav_msgs::msg::Path & full_path, size_t max_points);

  std::vector<nav_msgs::msg::Path> splitPathIntoSegments(
    const nav_msgs::msg::Path & full_path, double angle_threshold_rad);

  void visualize_tracking(double x, double y, double z);

  nav_msgs::msg::Path transformPathToBaseLink(const nav_msgs::msg::Path & path);
};

} //namespace robot_control

#endif  // ROBOT_CONTROL_COMPONENT_HPP