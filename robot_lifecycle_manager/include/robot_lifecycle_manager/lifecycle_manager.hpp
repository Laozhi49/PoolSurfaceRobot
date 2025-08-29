#pragma once
#include <memory>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace robot_lifecycle_manager
{

class LifecycleManager : public rclcpp::Node {
public:
  explicit LifecycleManager(const rclcpp::NodeOptions & options);
  
private:
  void execute_lifecycle_transition(
    const std::string & node_name,
    uint8_t transition_id);
  
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> clients_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> managed_nodes_;
};

} // namespace robot_lifecycle_manager