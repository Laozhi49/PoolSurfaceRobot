#include "robot_lifecycle_manager/lifecycle_manager.hpp"

namespace robot_lifecycle_manager{

LifecycleManager::LifecycleManager(const rclcpp::NodeOptions & options)
: Node("lifecycle_manager", options) {
  declare_parameter("managed_nodes", std::vector<std::string>{});
  managed_nodes_ = get_parameter("managed_nodes").as_string_array();
  
  // 延迟执行，等 container 开始 spin
  timer_ = this->create_wall_timer(
  std::chrono::seconds(1),
  [this]() {
    for (const auto & node : managed_nodes_) {
      RCLCPP_INFO(this->get_logger(), "Sending CONFIGURE to %s", node.c_str());
      execute_lifecycle_transition(
        node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      
      // 稍微等一下再 activate，或者直接也发 activate
      RCLCPP_INFO(this->get_logger(), "Sending ACTIVATE to %s", node.c_str());
      execute_lifecycle_transition(
        node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }

    // 执行完就取消定时器
    timer_->cancel();
  }
);

}

void LifecycleManager::execute_lifecycle_transition(
  const std::string & node_name,
  uint8_t transition_id) 
{
  if (clients_.find(node_name) == clients_.end()) {
    clients_[node_name] = this->create_client<lifecycle_msgs::srv::ChangeState>(
      node_name + "/change_state");
  }

  auto client = clients_[node_name];
  
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "Service unavailable for %s", node_name.c_str());
    return;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition_id;

  // 使用 lambda 捕获 node_name 和 transition_id
  auto callback = [this, node_name, transition_id](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(get_logger(), "Transition %d for %s SUCCESS", transition_id, node_name.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Transition %d for %s FAILED", transition_id, node_name.c_str());
    }
  };
  
  client->async_send_request(request, callback);
}

} // namespace robot_lifecycle_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_lifecycle_manager::LifecycleManager)