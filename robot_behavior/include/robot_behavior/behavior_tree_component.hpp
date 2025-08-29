#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace robot_behavior{

class BehaviorTreeComponent : public rclcpp_lifecycle::LifecycleNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit BehaviorTreeComponent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void tickTree();

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> cow_walk_tree_;
  std::unique_ptr<BT::Tree> nav_to_pose_tree_;
  BT::NodeStatus status_;

  rclcpp::TimerBase::SharedPtr timer_;
  BT::Blackboard::Ptr blackboard_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_to_pose_server_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp_action::GoalResponse handle_nav_to_pose_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse handle_nav_to_pose_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void handle_nav_to_pose_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void execute_nav_to_pose(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

};

} // namespace robot_behavior