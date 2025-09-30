#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "robot_interfaces/action/navigate_coverage_path.hpp"

namespace robot_behavior{

class BehaviorTreeComponent : public rclcpp_lifecycle::LifecycleNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  
  using NavigateCoveragePath = robot_interfaces::action::NavigateCoveragePath;
  using GoalHandleNavigateCoveragePath = rclcpp_action::ServerGoalHandle<NavigateCoveragePath>;

  explicit BehaviorTreeComponent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void tickTree();

  // 参数
  std::string navigate_to_pose_name_;
  std::string goal_pose_topic_;
  
  std::string navigate_coverage_path_name_;
  std::string clicked_point_topic_;

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> cow_walk_tree_;
  std::unique_ptr<BT::Tree> nav_to_pose_tree_;
  std::unique_ptr<BT::Tree> nav_coverage_path_tree_;
  BT::NodeStatus status_;

  rclcpp::TimerBase::SharedPtr timer_;
  BT::Blackboard::Ptr blackboard_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_to_pose_server_;
  rclcpp_action::Server<NavigateCoveragePath>::SharedPtr navigate_coverage_path_server_;

  // NavigateToPose
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr current_goal_;
  
  // NavigateCoveragePath
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp_action::Client<NavigateCoveragePath>::SharedPtr nav_coverage_client_;
  rclcpp_action::ClientGoalHandle<NavigateCoveragePath>::SharedPtr current_coverage_goal_;
  // 缓存 clicked_point
  std::vector<geometry_msgs::msg::Point32> polygon_points_;

  // 回调函数
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onClickedPointReceived(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  rclcpp_action::GoalResponse handle_nav_to_pose_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse handle_nav_to_pose_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void handle_nav_to_pose_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void execute_nav_to_pose(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  rclcpp_action::GoalResponse handle_nav_coverage_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateCoveragePath::Goal> goal);
  rclcpp_action::CancelResponse handle_nav_coverage_cancel(
    const std::shared_ptr<GoalHandleNavigateCoveragePath> goal_handle);
  void handle_nav_coverage_accepted(
    const std::shared_ptr<GoalHandleNavigateCoveragePath> goal_handle);
  void execute_nav_coverage_path(
    const std::shared_ptr<GoalHandleNavigateCoveragePath> goal_handle);
};

} // namespace robot_behavior