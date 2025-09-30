#include "robot_behavior/behavior_tree_component.hpp"

namespace robot_behavior{

BehaviorTreeComponent::BehaviorTreeComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("bt_component", options)
{
  this->declare_parameter("navigate_to_pose_name", "navigate_to_pose");
  this->declare_parameter("goal_pose_topic", "/goal_pose");
  this->declare_parameter("navigate_coverage_path_name", "navigate_coverage_path");
  this->declare_parameter("clicked_point_topic", "/clicked_point");
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn
BehaviorTreeComponent::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring Behavior Tree...");

  this->get_parameter("navigate_to_pose_name", navigate_to_pose_name_);
  this->get_parameter("goal_pose_topic", goal_pose_topic_);
  this->get_parameter("navigate_coverage_path_name", navigate_coverage_path_name_);
  this->get_parameter("clicked_point_topic", clicked_point_topic_);

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
  factory_.registerFromPlugin("libfollow_coverage_node.so");
  factory_.registerFromPlugin("libcompute_polygon_coverage_path_node.so");
  
  try {
    cow_walk_tree_ = std::make_unique<BT::Tree>(
      factory_.createTreeFromFile(
        "install/robot_behavior/share/robot_behavior/behavior_trees/cow_walk.xml",
        blackboard_));
    nav_to_pose_tree_ = std::make_unique<BT::Tree>(
      factory_.createTreeFromFile(
        "install/robot_behavior/share/robot_behavior/behavior_trees/nav_to_pose.xml",
        blackboard_));
    nav_coverage_path_tree_ = std::make_unique<BT::Tree>(
      factory_.createTreeFromFile(
        "install/robot_behavior/share/robot_behavior/behavior_trees/navigate_coverage_path.xml",
        blackboard_));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to create tree: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  status_ = BT::NodeStatus::IDLE;
  
  // NavigateToPose
  navigate_to_pose_server_ = rclcpp_action::create_server<NavigateToPose>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      navigate_to_pose_name_,
      std::bind(&BehaviorTreeComponent::handle_nav_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&BehaviorTreeComponent::handle_nav_to_pose_cancel, this,std::placeholders::_1),
      std::bind(&BehaviorTreeComponent::handle_nav_to_pose_accepted, this,std::placeholders::_1)
    );

  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_pose_topic_, 10,
    std::bind(&BehaviorTreeComponent::onGoalPoseReceived, this, std::placeholders::_1));

  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_to_pose_name_);

  // NavigateCoveragePath
  navigate_coverage_path_server_ = rclcpp_action::create_server<NavigateCoveragePath>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      navigate_coverage_path_name_, 
      std::bind(&BehaviorTreeComponent::handle_nav_coverage_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&BehaviorTreeComponent::handle_nav_coverage_cancel, this, std::placeholders::_1),
      std::bind(&BehaviorTreeComponent::handle_nav_coverage_accepted, this, std::placeholders::_1)
    );

  clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    clicked_point_topic_, 10,
    std::bind(&BehaviorTreeComponent::onClickedPointReceived, this, std::placeholders::_1));

  nav_coverage_client_ = rclcpp_action::create_client<NavigateCoveragePath>(
    this, navigate_coverage_path_name_);

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
  nav_coverage_path_tree_.reset();
  blackboard_.reset();
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn
BehaviorTreeComponent::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down Behavior Tree...");
  cow_walk_tree_.reset();
  nav_to_pose_tree_.reset();
  nav_coverage_path_tree_.reset();
  blackboard_.reset();
  return CallbackReturn::SUCCESS;
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