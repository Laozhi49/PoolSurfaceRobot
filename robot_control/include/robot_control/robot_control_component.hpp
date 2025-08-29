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

#include "robot_control/pid.hpp"

namespace robot_control{

class RobotControlComponent : public rclcpp_lifecycle::LifecycleNode
{
public:
  using Rotation = robot_interfaces::action::Rotation;
  using GoalHandleRotation = rclcpp_action::ServerGoalHandle<Rotation>;

  using MoveDistance = robot_interfaces::action::MoveDistance;
  using GoalHandleMoveDistance = rclcpp_action::ServerGoalHandle<MoveDistance>;

  using FollowPath = robot_interfaces::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

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

  PID_AbsoluteType yaw_angle_pid;

  geometry_msgs::msg::PoseStamped current_pose_;
  sensor_msgs::msg::Imu imu_data_;

  nav_msgs::msg::Path global_path_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void yaw_angle_continue(float angle);

  // ROS2接口
  rclcpp::Service<robot_interfaces::srv::GetRobotPose>::SharedPtr get_pose_service_;
  rclcpp::Service<robot_interfaces::srv::SetRobotSpeed>::SharedPtr set_speed_service_;

  rclcpp_action::Server<Rotation>::SharedPtr rotation_action_server_;
  rclcpp_action::Server<MoveDistance>::SharedPtr forwarddistance_action_server_;
  rclcpp_action::Server<FollowPath>::SharedPtr followpath_action_server_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // 回调函数
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void handle_get_pose(
    const std::shared_ptr<robot_interfaces::srv::GetRobotPose::Request>,
    std::shared_ptr<robot_interfaces::srv::GetRobotPose::Response> response);
  void handle_set_speed(
    const std::shared_ptr<robot_interfaces::srv::SetRobotSpeed::Request> request,
    std::shared_ptr<robot_interfaces::srv::SetRobotSpeed::Response> response);
  
  // 动作服务器相关函数
  rclcpp_action::GoalResponse handle_rotation_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Rotation::Goal> goal);
  rclcpp_action::CancelResponse handle_rotation_cancel(
    const std::shared_ptr<GoalHandleRotation> goal_handle);
  void handle_rotation_accepted(const std::shared_ptr<GoalHandleRotation> goal_handle);
  void execute_rotation(const std::shared_ptr<GoalHandleRotation> goal_handle);

  rclcpp_action::GoalResponse handle_movedistance_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveDistance::Goal> goal);
  rclcpp_action::CancelResponse handle_movedistance_cancel(
    const std::shared_ptr<GoalHandleMoveDistance> goal_handle);
  void handle_movedistance_accepted(const std::shared_ptr<GoalHandleMoveDistance> goal_handle);
  void execute_movedistance(const std::shared_ptr<GoalHandleMoveDistance> goal_handle);

  rclcpp_action::GoalResponse handle_followpath_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowPath::Goal> goal);
  rclcpp_action::CancelResponse handle_followpath_cancel(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  void handle_followpath_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  void execute_followpath(const std::shared_ptr<GoalHandleFollowPath> goal_handle);


  geometry_msgs::msg::Twist computeVelocityCommands();
  nav_msgs::msg::Path transformPathToBaseLink(const nav_msgs::msg::Path & path);
  geometry_msgs::msg::Point findLookaheadPoint(
    const nav_msgs::msg::Path & path, double lookahead_distance);
};

} //namespace robot_control

#endif  // ROBOT_CONTROL_COMPONENT_HPP