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

  // 声明参数
  this->declare_parameter("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter("camera_pose_topic", "/orbslam3/camera_pose");
  this->declare_parameter("aruco_pose_topic", "/pose");
  this->declare_parameter("projected_marker_topic", "/projected_marker");
  this->declare_parameter("imu_topic", "/imu_data");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("robot_frame", "base_link");
  this->declare_parameter("camera_frame", "camera_link");
  this->declare_parameter("aruco_marker_frame", "aruco_marker_frame");

  this->declare_parameter("pose_service_name", "get_robot_pose");
  this->declare_parameter("speed_service_name", "set_robot_speed");
  this->declare_parameter("rotation_action_name", "rotation");
  this->declare_parameter("movedistnace_action_name", "move_distance");
  this->declare_parameter("followpath_action_name", "follow_path");
  this->declare_parameter("followcoverage_action_name", "follow_coverage");

  this->declare_parameter("goal_tolerance", 0.2);

  this->declare_parameter<double>("Kp", 2.5);
  this->declare_parameter<double>("Ki", 0.01);
  this->declare_parameter<double>("Kd", 0.5);

  // 初始化 TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pid_init_absolute(&yaw_angle_pid,1.0f, 0.0f, 0.0f, 1000.0f,1000.0f);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlComponent::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring...");

  // 获取参数
  this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
  this->get_parameter("camera_pose_topic", camera_pose_topic_);
  this->get_parameter("aruco_pose_topic", aruco_pose_topic_);
  this->get_parameter("projected_marker_topic", projected_marker_topic_);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("robot_frame", robot_frame_);
  this->get_parameter("camera_frame", camera_frame_);
  this->get_parameter("aruco_marker_frame", aruco_marker_frame_);

  this->get_parameter("pose_service_name", pose_service_name_);
  this->get_parameter("speed_service_name", speed_service_name_);
  this->get_parameter("rotation_action_name", rotation_action_name_);
  this->get_parameter("movedistnace_action_name", movedistnace_action_name_);
  this->get_parameter("followpath_action_name", followpath_action_name_);
  this->get_parameter("followcoverage_action_name", followcoverage_action_name_);

  this->get_parameter("goal_tolerance", goal_tolerance_);

  Kp_ = this->get_parameter("Kp").as_double();
  Ki_ = this->get_parameter("Ki").as_double();
  Kd_ = this->get_parameter("Kd").as_double();
  
  // 初始化发布器
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  projected_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(projected_marker_topic_, 10);
  tracking_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/tracking_marker", 10);

  RCLCPP_INFO(get_logger(), "Configuration completed.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlComponent::on_activate(const rclcpp_lifecycle::State &)
{
  cmd_vel_pub_->on_activate();
  projected_marker_pub_->on_activate();
  tracking_marker_pub_->on_activate();

  // 初始化服务、动作服务器
  get_pose_service_ = create_service<robot_interfaces::srv::GetRobotPose>(
    pose_service_name_,
    std::bind(&RobotControlComponent::handle_get_pose, this, std::placeholders::_1, std::placeholders::_2));
  
  set_speed_service_ = create_service<robot_interfaces::srv::SetRobotSpeed>(
    speed_service_name_,
    std::bind(&RobotControlComponent::handle_set_speed, this, std::placeholders::_1, std::placeholders::_2));
  
  rotation_action_server_ = rclcpp_action::create_server<Rotation>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    rotation_action_name_,
    std::bind(&RobotControlComponent::handle_rotation_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RobotControlComponent::handle_rotation_cancel, this, std::placeholders::_1),
    std::bind(&RobotControlComponent::handle_rotation_accepted, this, std::placeholders::_1));

  movedistance_action_server_ = rclcpp_action::create_server<MoveDistance>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    movedistnace_action_name_,
    std::bind(&RobotControlComponent::handle_movedistance_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RobotControlComponent::handle_movedistance_cancel, this, std::placeholders::_1),
    std::bind(&RobotControlComponent::handle_movedistance_accepted, this, std::placeholders::_1));

  followpath_action_server_ = rclcpp_action::create_server<FollowPath>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    followpath_action_name_,
    std::bind(&RobotControlComponent::handle_followpath_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RobotControlComponent::handle_followpath_cancel, this, std::placeholders::_1),
    std::bind(&RobotControlComponent::handle_followpath_accepted, this, std::placeholders::_1)
  );

  followcoverage_action_server_ = rclcpp_action::create_server<FollowCoverage>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    followcoverage_action_name_,
    std::bind(&RobotControlComponent::handle_followcoverage_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RobotControlComponent::handle_followcoverage_cancel, this, std::placeholders::_1),
    std::bind(&RobotControlComponent::handle_followcoverage_accepted, this, std::placeholders::_1)
  );
  // 初始化订阅器
  // camera_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
  //   camera_pose_topic_, 10, std::bind(&RobotControlComponent::pose_callback, this, std::placeholders::_1));
  aruco_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    aruco_pose_topic_, 10, std::bind(&RobotControlComponent::pose_callback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 10,
    std::bind(&RobotControlComponent::imu_callback, this, std::placeholders::_1));

  // 注册参数回调（运行时调参用）
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&RobotControlComponent::paramCallback, this, std::placeholders::_1));
    
  RCLCPP_INFO(get_logger(), "Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlComponent::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");

  cmd_vel_pub_->on_deactivate();
  projected_marker_pub_->on_deactivate();
  tracking_marker_pub_->on_deactivate();

  get_pose_service_.reset();
  set_speed_service_.reset();
  rotation_action_server_.reset();
  followpath_action_server_.reset();
  followcoverage_action_server_.reset();
  // camera_pose_sub_.reset();
  aruco_pose_sub_.reset();
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
  projected_marker_pub_.reset();
  tracking_marker_pub_.reset();

  get_pose_service_.reset();
  set_speed_service_.reset();
  rotation_action_server_.reset();
  followpath_action_server_.reset();
  followcoverage_action_server_.reset();
  // camera_pose_sub_.reset();
  aruco_pose_sub_.reset();
  imu_sub_.reset();
  
  RCLCPP_INFO(get_logger(), "Cleaned up complited.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotControlComponent::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shut down.");

  cmd_vel_pub_.reset();
  projected_marker_pub_.reset();
  tracking_marker_pub_.reset();
  
  get_pose_service_.reset();
  set_speed_service_.reset();
  rotation_action_server_.reset();
  followpath_action_server_.reset();
  followcoverage_action_server_.reset();
  // camera_pose_sub_.reset();
  aruco_pose_sub_.reset();
  imu_sub_.reset();
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// 参数回调
rcl_interfaces::msg::SetParametersResult RobotControlComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : params) {
    if (param.get_name() == "Kp") {
      Kp_ = param.as_double();
      RCLCPP_INFO(get_logger(), "Kp changed to: %.3f", Kp_);
    } else if (param.get_name() == "Ki") {
      Ki_ = param.as_double();
      RCLCPP_INFO(get_logger(), "Ki changed to: %.3f", Ki_);
    } else if (param.get_name() == "Kd") {
      Kd_ = param.as_double();
      RCLCPP_INFO(get_logger(), "Kd changed to: %.3f", Kd_);
    }
  }
  return result;
}

void RobotControlComponent::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 发布 map->camera_link
  // geometry_msgs::msg::TransformStamped map_to_camera;

  // map_to_camera.header.stamp = msg->header.stamp;
  // map_to_camera.header.frame_id = map_frame_;
  // map_to_camera.child_frame_id = camera_frame_;

  // map_to_camera.transform.translation.x = msg->pose.position.x;
  // map_to_camera.transform.translation.y = msg->pose.position.y;
  // map_to_camera.transform.translation.z = msg->pose.position.z;

  // map_to_camera.transform.rotation = msg->pose.orientation;
  
  // tf_broadcaster_->sendTransform(map_to_camera);

  // 发布 map->aruco_marker_frame
  geometry_msgs::msg::TransformStamped map_to_aruco;

  map_to_aruco.header.stamp = msg->header.stamp;
  map_to_aruco.header.frame_id = map_frame_;
  map_to_aruco.child_frame_id = aruco_marker_frame_;

  map_to_aruco.transform.translation.x = msg->pose.position.x;
  map_to_aruco.transform.translation.y = msg->pose.position.y;
  map_to_aruco.transform.translation.z = msg->pose.position.z;

  map_to_aruco.transform.rotation = msg->pose.orientation;
  
  tf_broadcaster_->sendTransform(map_to_aruco);

  // 计算map下的机器人位姿
  // geometry_msgs::msg::TransformStamped camera_to_base;
  geometry_msgs::msg::TransformStamped aruco_to_base;

  try {
    // 查找 base_link 在 camera_link 下的变换
    // camera_to_base = tf_buffer_->lookupTransform(
    //   camera_frame_,  // 目标坐标系
    aruco_to_base = tf_buffer_->lookupTransform(
      aruco_marker_frame_,  // 目标坐标系
      robot_frame_,    // 源坐标系
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Could not transform base_link from camera_link: %s", ex.what());
    return;
  }

  // 把相机的 Pose 转换成底盘的 Pose
  geometry_msgs::msg::PoseStamped base_pose;
  // tf2::doTransform(*msg, base_pose, camera_to_base);
  tf2::doTransform(*msg, base_pose, aruco_to_base);

  // 存储机器人当前位姿
  current_pose_ = base_pose;

  // 将机器人的位姿投影到2D平面
  // --- 投影到 z=0 平面 ---
  geometry_msgs::msg::PoseStamped projected_pose = base_pose;

  // 1. 位置投影
  projected_pose.pose.position.z = 0.0;

  // 2. 提取 yaw
  tf2::Quaternion q(
      base_pose.pose.orientation.x,
      base_pose.pose.orientation.y,
      base_pose.pose.orientation.z,
      base_pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // 3. 只保留 yaw
  tf2::Quaternion q2;
  q2.setRPY(0, 0, yaw);
  projected_pose.pose.orientation.x = q2.x();
  projected_pose.pose.orientation.y = q2.y();
  projected_pose.pose.orientation.z = q2.z();
  projected_pose.pose.orientation.w = q2.w();
  
  // 可视化
  visualizeProjectedPose(projected_marker_pub_, projected_pose, map_frame_, this->get_clock());
}

void RobotControlComponent::visualizeProjectedPose(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
    const geometry_msgs::msg::PoseStamped &pose_in,
    const std::string &frame_id,
    rclcpp::Clock::SharedPtr clock)
{
    // 1. 复制输入位姿
    geometry_msgs::msg::PoseStamped pose_out = pose_in;

    // 2. 投影到 z=0 平面
    pose_out.pose.position.z = 0.0;

    tf2::Quaternion q(
        pose_in.pose.orientation.x,
        pose_in.pose.orientation.y,
        pose_in.pose.orientation.z,
        pose_in.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    tf2::Quaternion q2;
    q2.setRPY(0, 0, yaw);
    pose_out.pose.orientation.x = q2.x();
    pose_out.pose.orientation.y = q2.y();
    pose_out.pose.orientation.z = q2.z();
    pose_out.pose.orientation.w = q2.w();

    // 3. 转成 RViz Marker (箭头)
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = clock->now();
    marker.ns = "robot_projection";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = pose_out.pose;

    marker.scale.x = 0.5;  // 箭头长度
    marker.scale.y = 0.1;  // 箭头宽度
    marker.scale.z = 0.1;  // 箭头高度

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // 4. 发布
    marker_pub->publish(marker);
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

nav_msgs::msg::Path RobotControlComponent::transformPathToBaseLink(const nav_msgs::msg::Path & path)
{
    nav_msgs::msg::Path transformed_path;
    transformed_path.header.frame_id = "base_link";

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
        tf_msg = tf_buffer_->lookupTransform(
            "base_link",
            path.header.frame_id,  // e.g. "map"
            tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform path: %s", ex.what());
        return transformed_path;
    }

    for (auto & pose : path.poses) {
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf2::doTransform(pose, transformed_pose, tf_msg);
        transformed_path.poses.push_back(transformed_pose);
    }
    return transformed_path;
}

} //namespace robot_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_control::RobotControlComponent)