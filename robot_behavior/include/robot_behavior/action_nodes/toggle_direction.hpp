#ifndef TOGGLE_DIRECTION_HPP_
#define TOGGLE_DIRECTION_HPP_

#include "behaviortree_cpp/bt_factory.h"

class ToggleDirection : public BT::SyncActionNode
{
public:
  ToggleDirection(const std::string& name, const BT::NodeConfiguration& config);

  // 声明需要的端口（这里是黑板里的 turn_angle）
  static BT::PortsList providedPorts();

  // 核心执行逻辑
  BT::NodeStatus tick() override;
};

#endif  // TOGGLE_DIRECTION_HPP_
