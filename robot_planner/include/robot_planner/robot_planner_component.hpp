#ifndef ROBOT_PLANNER__ROBOT_PLANNER_COMPONENT_HPP_
#define ROBOT_PLANNER__ROBOT_PLANNER_COMPONENT_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <lifecycle_msgs/msg/state.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_interfaces/srv/get_robot_pose.hpp"
#include "robot_interfaces/action/compute_path_to_pose.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "robot_planner/planner/astar_planner.hpp"

namespace robot_planner
{

class PlannerComponent : public rclcpp_lifecycle::LifecycleNode
{
public:
  using GetRobotPose = robot_interfaces::srv::GetRobotPose;

  using ComputePathToPose = robot_interfaces::action::ComputePathToPose;
  using GoalHandleComputePathToPose = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

  explicit PlannerComponent(const rclcpp::NodeOptions & options);

protected:
  // 生命周期回调
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // 订阅回调
  void CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;
  // Action 回调
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePathToPose::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

  void execute(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

  nav_msgs::msg::Path planAStar(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::OccupancyGrid & map);

  void publishMarker(const nav_msgs::msg::Path & path);

  // 成员变量
  std::string costmap_topic_;
  std::string pose_service_name_;
  std::string action_name_;

  // 存储最新的代价地图
  std::optional<nav_msgs::msg::OccupancyGrid> costmap_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Client<GetRobotPose>::SharedPtr pose_client_;

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::unique_ptr<AStar_Planner> planner_;
};

}  // namespace robot_planner

#endif  // ROBOT_PLANNER__ROBOT_PLANNER_COMPONENT_HPP_
