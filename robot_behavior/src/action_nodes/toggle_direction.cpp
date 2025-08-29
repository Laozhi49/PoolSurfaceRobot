#include "robot_behavior/action_nodes/toggle_direction.hpp"
#include <iostream>

ToggleDirection::ToggleDirection(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList ToggleDirection::providedPorts()
{
  // 定义一个可读可写的端口
  return { BT::BidirectionalPort<float>("turn_angle") };
}

BT::NodeStatus ToggleDirection::tick()
{
  float current_angle = 90.0;  // 默认值
  if (!getInput("turn_angle", current_angle))
  {
    std::cerr << "[ToggleDirection] 没有在黑板上找到 turn_angle，使用默认 90.0" << std::endl;
  }

  float new_angle = (current_angle == 90.0) ? -90.0 : 90.0;

  setOutput("turn_angle", new_angle);

  std::cout << "[ToggleDirection] 切换方向: " << current_angle
            << " -> " << new_angle << std::endl;

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp/bt_factory.h"

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<ToggleDirection>("ToggleDirection");
// }

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<ToggleDirection>("ToggleDirection");
}