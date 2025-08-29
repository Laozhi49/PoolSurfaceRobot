#include "robot_behavior/action_nodes/set_speed.hpp"

SetSpeed::SetSpeed(const std::string& name,
                   const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>("node");

  client_ = node_->create_client<robot_interfaces::srv::SetRobotSpeed>("/set_robot_speed");
}

BT::PortsList SetSpeed::providedPorts()
{
  return { 
    BT::InputPort<float>("speed_x"), 
    BT::InputPort<float>("speed_z_tar") 
  };
}

BT::NodeStatus SetSpeed::tick()
{
  float speed_x, speed_z;
  if (!getInput("speed_x", speed_x) || !getInput("speed_z_tar", speed_z))
    return BT::NodeStatus::FAILURE;

  auto req = std::make_shared<robot_interfaces::srv::SetRobotSpeed::Request>();
  req->speed_x = speed_x;
  req->speed_z_tar = speed_z;

  if (!client_->wait_for_service(1s))
    return BT::NodeStatus::FAILURE;

  // 直接发请求，不要自己 spin，结果由 executor 回调处理
  auto future = client_->async_send_request(req);

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp/bt_factory.h"

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<SetSpeed>("SetSpeed");
// }

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<SetSpeed>("SetSpeed");
}


