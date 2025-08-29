#include "behaviortree_cpp/bt_factory.h"
#include "robot_behavior/condition_nodes/is_obstacle_ahead.hpp"
#include "robot_behavior/action_nodes/set_speed.hpp"
#include "robot_behavior/action_nodes/rotate.hpp"
#include "robot_behavior/action_nodes/move_distance.hpp"
#include "robot_behavior/action_nodes/toggle_direction.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 创建一个共享的 ROS2 Node
  auto shared_node = rclcpp::Node::make_shared("bt_node");

  // 建立 blackboard，把 node 放进去
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", shared_node);

  BT::BehaviorTreeFactory factory;

  // 加载动态库
  factory.registerFromPlugin("libis_obstacle_ahead.so");
  factory.registerFromPlugin("libmove_distance.so");
  factory.registerFromPlugin("librotate.so");
  factory.registerFromPlugin("libset_speed.so");
  factory.registerFromPlugin("libtoggle_direction.so");

  auto tree = factory.createTreeFromFile(
    "install/robot_behavior/share/robot_behavior/behavior_trees/cow_walk.xml"
    ,blackboard);

  
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  rclcpp::Rate loop_rate(10);

  // 用 executor 管理 shared_node
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(shared_node);

  while (rclcpp::ok())// && status == BT::NodeStatus::RUNNING)
  { 
    status = tree.tickOnce();
    executor.spin_some();  // 统一处理 ROS 回调
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
