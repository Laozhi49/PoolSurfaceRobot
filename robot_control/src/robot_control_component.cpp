#include "robot_control/robot_control_component.hpp"

namespace robot_control{

RobotControlComponent::RobotControlComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("robot_control", options)
{
  // 初始化参数
  speed_x_ = 0.0;
  speed_z_tar_ = 0.0;
  speed_z_cur_ = 0.0;
  yaw_ = 0.0;
  wz_ = 0.0;

  // 初始化 TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pid_init_absolute(&yaw_angle_pid,1.0f, 0.0f, 0.0f, 1000.0f,1000.0f);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlComponent::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring...");

  // 初始化发布器
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  RCLCPP_INFO(get_logger(), "Configuration completed.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlComponent::on_activate(const rclcpp_lifecycle::State &)
{
  cmd_vel_pub_->on_activate();

  // 初始化服务、动作服务器
  get_pose_service_ = create_service<robot_interfaces::srv::GetRobotPose>(
    "get_robot_pose",
    std::bind(&RobotControlComponent::handle_get_pose, this, std::placeholders::_1, std::placeholders::_2));
  
  set_speed_service_ = create_service<robot_interfaces::srv::SetRobotSpeed>(
    "set_robot_speed",
    std::bind(&RobotControlComponent::handle_set_speed, this, std::placeholders::_1, std::placeholders::_2));
  
  rotation_action_server_ = rclcpp_action::create_server<Rotation>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "rotation",
    std::bind(&RobotControlComponent::handle_rotation_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RobotControlComponent::handle_rotation_cancel, this, std::placeholders::_1),
    std::bind(&RobotControlComponent::handle_rotation_accepted, this, std::placeholders::_1));

  forwarddistance_action_server_ = rclcpp_action::create_server<MoveDistance>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "move_distance",
    std::bind(&RobotControlComponent::handle_movedistance_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RobotControlComponent::handle_movedistance_cancel, this, std::placeholders::_1),
    std::bind(&RobotControlComponent::handle_movedistance_accepted, this, std::placeholders::_1));

  followpath_action_server_ = rclcpp_action::create_server<FollowPath>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "follow_path",
    std::bind(&RobotControlComponent::handle_followpath_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RobotControlComponent::handle_followpath_cancel, this, std::placeholders::_1),
    std::bind(&RobotControlComponent::handle_followpath_accepted, this, std::placeholders::_1)
  );

  // 初始化订阅器
  camera_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/orb_slam3/pose", 10, std::bind(&RobotControlComponent::pose_callback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu_data", 10,
    std::bind(&RobotControlComponent::imu_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlComponent::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");

  cmd_vel_pub_->on_deactivate();

  get_pose_service_.reset();
  set_speed_service_.reset();
  rotation_action_server_.reset();
  followpath_action_server_.reset();
  camera_pose_sub_.reset();
  imu_sub_.reset();

  RCLCPP_INFO(get_logger(), "Deactivation completed.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlComponent::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaned up...");
  // 清理资源
  cmd_vel_pub_.reset();

  get_pose_service_.reset();
  set_speed_service_.reset();
  rotation_action_server_.reset();
  followpath_action_server_.reset();
  camera_pose_sub_.reset();
  imu_sub_.reset();
  
  RCLCPP_INFO(get_logger(), "Cleaned up complited.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlComponent::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shut down.");

  cmd_vel_pub_.reset();

  get_pose_service_.reset();
  set_speed_service_.reset();
  rotation_action_server_.reset();
  followpath_action_server_.reset();
  camera_pose_sub_.reset();
  imu_sub_.reset();
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void RobotControlComponent::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 发布 map->camera_link
  geometry_msgs::msg::TransformStamped map_to_camera;

  map_to_camera.header.stamp = msg->header.stamp;
  map_to_camera.header.frame_id = "map";
  map_to_camera.child_frame_id = "camera_link";

  map_to_camera.transform.translation.x = msg->pose.position.x;
  map_to_camera.transform.translation.y = msg->pose.position.y;
  map_to_camera.transform.translation.z = msg->pose.position.z;

  map_to_camera.transform.rotation = msg->pose.orientation;
  
  tf_broadcaster_->sendTransform(map_to_camera);


  // 计算map下的机器人位姿
  geometry_msgs::msg::TransformStamped camera_to_base;

  try {
    // 查找 base_link 在 camera_link 下的变换
    camera_to_base = tf_buffer_->lookupTransform(
      "camera_link",  // 目标坐标系
      "base_link",    // 源坐标系
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Could not transform base_link from camera_link: %s", ex.what());
    return;
  }

  // 把相机的 Pose 转换成底盘的 Pose
  geometry_msgs::msg::PoseStamped base_pose;
  tf2::doTransform(*msg, base_pose, camera_to_base);

  // 存储机器人当前位姿
  current_pose_ = base_pose;
}

// 偏航角角度连续化
void RobotControlComponent::yaw_angle_continue(float angle)
{
  float angle_360 = angle + 180.0f;
  if(angle_360 >= 0.0f && yaw_last <=-350.0f)
    circle++;
  else if(angle_360 <= -350.0f && yaw_last >= 0.0f)
    circle--;
  yaw_last = angle_360;
  yaw_cur = yaw_last + circle * 360.0f;
}

void RobotControlComponent::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_data_ = *msg;

  wz_ = imu_data_.angular_velocity.z;

  tf2::Quaternion q(
      imu_data_.orientation.x,
      imu_data_.orientation.y,
      imu_data_.orientation.z,
      imu_data_.orientation.w
  );

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // 转成度
  roll_  = roll  * 180.0 / M_PI;
  pitch_ = pitch * 180.0 / M_PI;
  yaw_   = yaw   * 180.0 / M_PI;

  yaw_angle_continue(yaw_);

  // RCLCPP_INFO(rclcpp::get_logger("imu"), "Roll: %.2f, Pitch: %.2f, Yaw: %.2f wz: %.2f", roll_, pitch_, yaw_, wz_);
}

void RobotControlComponent::handle_get_pose(
  const std::shared_ptr<robot_interfaces::srv::GetRobotPose::Request>,
  std::shared_ptr<robot_interfaces::srv::GetRobotPose::Response> response)
{
  response->current_pose = current_pose_;
  response->success = true;
}

void RobotControlComponent::handle_set_speed(
  const std::shared_ptr<robot_interfaces::srv::SetRobotSpeed::Request> request,
  std::shared_ptr<robot_interfaces::srv::SetRobotSpeed::Response> response)
{
  speed_x_ = request->speed_x;
  speed_z_tar_ = request->speed_z_tar;
  
  // 发布速度命令
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = speed_x_;
  twist.angular.z = speed_z_tar_;
  cmd_vel_pub_->publish(twist);
  
  response->success = true;
  response->message = "Speed set successfully";
}

} //namespace robot_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_control::RobotControlComponent)