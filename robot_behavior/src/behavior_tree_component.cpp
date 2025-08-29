#include "robot_behavior/behavior_tree_component.hpp"

namespace robot_behavior{

BehaviorTreeComponent::BehaviorTreeComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("bt_component", options)
{
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn
BehaviorTreeComponent::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring Behavior Tree...");

  blackboard_ = BT::Blackboard::create();
  // LifecycleNode 转换成普通 Node

  blackboard_->set("node", shared_from_this());
  blackboard_->set("turn_angle", 90.0f);
  // 注册插件
  factory_.registerFromPlugin("libis_obstacle_ahead.so");
  factory_.registerFromPlugin("libmove_distance.so");
  factory_.registerFromPlugin("librotate.so");
  factory_.registerFromPlugin("libset_speed.so");
  factory_.registerFromPlugin("libtoggle_direction.so");
  factory_.registerFromPlugin("libcompute_path_to_pose_node.so");
  factory_.registerFromPlugin("libfollow_path_node.so");
  
  try {
    cow_walk_tree_ = std::make_unique<BT::Tree>(
      factory_.createTreeFromFile(
        "install/robot_behavior/share/robot_behavior/behavior_trees/cow_walk.xml",
        blackboard_));
    nav_to_pose_tree_ = std::make_unique<BT::Tree>(
      factory_.createTreeFromFile(
        "install/robot_behavior/share/robot_behavior/behavior_trees/nav_to_pose.xml",
        blackboard_));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to create tree: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  status_ = BT::NodeStatus::IDLE;
  
  navigate_to_pose_server_ = rclcpp_action::create_server<NavigateToPose>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "navigate_to_pose",
      std::bind(&BehaviorTreeComponent::handle_nav_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&BehaviorTreeComponent::handle_nav_to_pose_cancel, this,std::placeholders::_1),
      std::bind(&BehaviorTreeComponent::handle_nav_to_pose_accepted, this,std::placeholders::_1)
    );

  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&BehaviorTreeComponent::onGoalPoseReceived, this, std::placeholders::_1));

  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn
BehaviorTreeComponent::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating Behavior Tree...");
  timer_ = create_wall_timer(std::chrono::milliseconds(100),
                             std::bind(&BehaviorTreeComponent::tickTree, this));
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn
BehaviorTreeComponent::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating Behavior Tree...");
  timer_.reset();
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn
BehaviorTreeComponent::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Behavior Tree...");
  cow_walk_tree_.reset();
  nav_to_pose_tree_.reset();
  blackboard_.reset();
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn
BehaviorTreeComponent::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down Behavior Tree...");
  cow_walk_tree_.reset();
  nav_to_pose_tree_.reset();
  blackboard_.reset();
  return CallbackReturn::SUCCESS;
}

// 将rviz2发布的/goal_pose转换为navigate_to_pose Action
void BehaviorTreeComponent::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received /goal_pose from RViz (x=%.2f, y=%.2f)",
              msg->pose.position.x, msg->pose.position.y);

  if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available!");
    return;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = *msg;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Navigation succeeded!");
      } else {
        RCLCPP_ERROR(get_logger(), "Navigation failed or was canceled");
      }
    };

  nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
}

void BehaviorTreeComponent::tickTree()
{
  if (!cow_walk_tree_) {
    RCLCPP_WARN(get_logger(), "Tree is not initialized!");
    return;
  }
  // status_ = cow_walk_tree_->tickOnce();
}

} // namespace robot_behavior

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_behavior::BehaviorTreeComponent)