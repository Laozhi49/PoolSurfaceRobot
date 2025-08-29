#pragma once

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot_costmap
{

class RobotCostmapComponent : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit RobotCostmapComponent(const rclcpp::NodeOptions & options);

protected:
  // 生命周期回调
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void publishCostmap();
  void inflateCostmap(nav_msgs::msg::OccupancyGrid & costmap, double inflation_radius);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string map_topic_;
  std::string global_costmap_topic_;
  double publish_frequency_;
  double inflation_radius_;
};

} // namespace robot_costmap
