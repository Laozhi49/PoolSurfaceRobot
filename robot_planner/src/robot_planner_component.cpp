#include "robot_planner/robot_planner_component.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace robot_planner {

PlannerComponent::PlannerComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("robot_planner", options) {
    // Declare parameters with defaults
    this->declare_parameter("costmap_topic", "/global_costmap");
    this->declare_parameter("pose_service_name", "get_robot_pose");
    this->declare_parameter("action_name", "compute_path_to_pose");
}

PlannerComponent::CallbackReturn
PlannerComponent::on_configure(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Configuring robot_planner...");
    
    this->get_parameter("costmap_topic", costmap_topic_);
    this->get_parameter("pose_service_name", pose_service_name_);
    this->get_parameter("action_name", action_name_);
    
    // auto map_qos = rclcpp::QoS(rclcpp::KeepAll()).transient_local().reliable();
    auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic_, map_qos,
        std::bind(&PlannerComponent::CostmapCallback, this, std::placeholders::_1));


    // Pose client
    pose_client_ = this->create_client<robot_interfaces::srv::GetRobotPose>(pose_service_name_);

    // Action server
    action_server_ = rclcpp_action::create_server<ComputePathToPose>(
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        action_name_,
        std::bind(&PlannerComponent::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PlannerComponent::handle_cancel, this, std::placeholders::_1),
        std::bind(&PlannerComponent::handle_accepted, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "path_marker", 10);

    planner_ = std::make_unique<AStar_Planner>();
    
    RCLCPP_INFO(get_logger(), "Configured.");
    return CallbackReturn::SUCCESS;
}

PlannerComponent::CallbackReturn
PlannerComponent::on_activate(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Activating robot_planner...");
    marker_pub_->on_activate();
    RCLCPP_INFO(get_logger(), "Activated.");
    return CallbackReturn::SUCCESS;
}

PlannerComponent::CallbackReturn
PlannerComponent::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Deactivating robot_planner...");
    marker_pub_.reset();
    costmap_sub_.reset();
    pose_client_.reset();
    action_server_.reset();
    costmap_.reset();
    return CallbackReturn::SUCCESS;
}

PlannerComponent::CallbackReturn
PlannerComponent::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Cleaning up robot_planner...");
    marker_pub_.reset();
    costmap_sub_.reset();
    pose_client_.reset();
    action_server_.reset();
    costmap_.reset();
    return CallbackReturn::SUCCESS;
}

PlannerComponent::CallbackReturn
PlannerComponent::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Shutting down robot_planner...");
    marker_pub_.reset();
    costmap_sub_.reset();
    pose_client_.reset();
    action_server_.reset();
    costmap_.reset();
    return CallbackReturn::SUCCESS;
}

void PlannerComponent::CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    costmap_ = *msg;
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Map received: %u x %u, res=%.3f",
    // msg->info.width, msg->info.height, msg->info.resolution);
}

rclcpp_action::GoalResponse
PlannerComponent::handle_goal(const rclcpp_action::GoalUUID & /*uuid*/,
std::shared_ptr<const ComputePathToPose::Goal> goal) {
    (void)goal; // we accept any goal with a valid target pose
    if (!costmap_.has_value()) {
        RCLCPP_WARN(get_logger(), "Rejecting goal: map not available yet");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
PlannerComponent::handle_cancel(const std::shared_ptr<GoalHandleComputePathToPose> /*goal_handle*/) {
    RCLCPP_INFO(get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerComponent::handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle) {
    // Execute in a new thread so the server can accept new goals
    std::thread{std::bind(&PlannerComponent::execute, this, goal_handle)}.detach();
}

void PlannerComponent::execute(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle) {
    RCLCPP_INFO(get_logger(), "Planning started");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ComputePathToPose::Feedback>();
    auto result = std::make_shared<ComputePathToPose::Result>();

    // Wait for pose service if needed
    if (!pose_client_->wait_for_service(2s)) {
        RCLCPP_ERROR(get_logger(), "Pose service not available: %s", pose_service_name_.c_str());
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    // Call pose service
    auto req = std::make_shared<robot_interfaces::srv::GetRobotPose::Request>();
    auto future = pose_client_->async_send_request(req);
    if (future.wait_for(2s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Pose service timeout");
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    auto resp = future.get();
    if (!resp->success) {
        RCLCPP_ERROR(get_logger(), "Pose service returned success=false");
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    const auto start_pose = resp->current_pose; // geometry_msgs::msg::PoseStamped
    const auto target_pose = goal->target_pose;

    // Simple staged feedback progression; replace with your A* internal progress callbacks
    for (int i = 1; i <= 3; ++i) {
        if (goal_handle->is_canceling()) {
            RCLCPP_WARN(get_logger(), "Planning canceled");
            result->success = false;
            goal_handle->canceled(result);
            return;
        }
        feedback->progress = 0.25f * i; // 0.25, 0.5, 0.75
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(100ms);
    }

    // Do planning (A* TODO)
    if (!costmap_.has_value()) {
        RCLCPP_ERROR(get_logger(), "Map not available at execution time");
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    auto path = planSmoothPath(start_pose, target_pose, *costmap_);
    path.header.stamp = now();

    publishMarker(path);

    // Final feedback and result
    feedback->progress = 1.0f;
    goal_handle->publish_feedback(feedback);

    result->path = path;
    result->success = !path.poses.empty();

    if (result->success) {
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Planning succeeded with %zu poses", path.poses.size());
    } else {
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "Planning failed: empty path");
    }
}

nav_msgs::msg::Path PlannerComponent::planSmoothPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::OccupancyGrid & costmap)
{
    nav_msgs::msg::Path smooth_msg;
    // 1. A* 搜索
    auto grid_path = planner_->aStarSearch(start, goal, costmap);
    if (grid_path.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("planner"), "A* search failed, no path found.");
        return nav_msgs::msg::Path();
    }

    // smooth_msg = planner_->toPathMsg(grid_path,costmap);

    // 2. 距离场
    auto dist_field = planner_->computeDistanceField(costmap);

    // 3. 梯度下降平滑
    auto smooth_path = planner_->smoothPathWithGradientDescent(grid_path, dist_field, costmap);

    // 4. 转换为 nav_msgs::Path
    smooth_msg.header.frame_id = costmap.header.frame_id;
    smooth_msg.header.stamp = rclcpp::Clock().now();

    for (auto &p : smooth_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = smooth_msg.header;

        // 注意：这里假设 smoothPathWithGradientDescent 输出的是 (世界坐标 x,y)
        pose.pose.position.x = p.first;
        pose.pose.position.y = p.second;
        pose.pose.orientation.w = 1.0;
        smooth_msg.poses.push_back(pose);
    }

    return smooth_msg;
}

// path可视化
void PlannerComponent::publishMarker(const nav_msgs::msg::Path & path)
{
  // 转成 Marker
  visualization_msgs::msg::Marker marker;
  marker.header = path.header;
  marker.ns = "path_marker";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = 0.05;   // 线条粗细
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  for (auto &pose_stamped : path.poses) {
    marker.points.push_back(pose_stamped.pose.position);
  }

  marker_pub_->publish(marker);
}

} // namespace robot_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_planner::PlannerComponent)