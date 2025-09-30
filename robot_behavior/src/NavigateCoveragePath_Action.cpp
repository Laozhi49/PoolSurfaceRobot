#include "robot_behavior/behavior_tree_component.hpp"

namespace robot_behavior{

void BehaviorTreeComponent::onClickedPointReceived(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Clicked point: (%.2f, %.2f)", msg->point.x, msg->point.y);

  geometry_msgs::msg::Point32 p;
  p.x = msg->point.x;
  p.y = msg->point.y;
  p.z = msg->point.z;
  polygon_points_.push_back(p);

  if(polygon_points_.size() == 1)
  {
    if (current_coverage_goal_) {
      RCLCPP_WARN(get_logger(), "Canceling previous goal before sending new one");
      nav_coverage_client_->async_cancel_goal(current_coverage_goal_);
      // 不立即 reset，因为 result_callback 可能会在之后被调用；但为简单起见这里可以 reset 指针（可选）
      current_coverage_goal_.reset();
    }
  }

  if (polygon_points_.size() == 4) {
    RCLCPP_INFO(get_logger(), "Collected 4 points, sending NavigateCoveragePath goal");

    if (!nav_coverage_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "navigate_coverage_path action server not available!");
      polygon_points_.clear();
      return;
    }
    
    NavigateCoveragePath::Goal goal_msg;
    goal_msg.polygon = polygon_points_;

    auto send_goal_options = rclcpp_action::Client<NavigateCoveragePath>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<NavigateCoveragePath>::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Coverage goal was rejected");
        } else {
          RCLCPP_INFO(this->get_logger(), "Coverage goal accepted");
          this->current_coverage_goal_ = goal_handle;
        }
      };

    send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateCoveragePath>::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Coverage navigation succeeded!");
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          RCLCPP_WARN(this->get_logger(), "Coverage navigation was canceled");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Coverage navigation failed");
        }
        this->current_coverage_goal_.reset();
      };

    nav_coverage_client_->async_send_goal(goal_msg, send_goal_options);

    // 清空，方便下一次点击 4 个点
    polygon_points_.clear();
  }
}

rclcpp_action::GoalResponse BehaviorTreeComponent::handle_nav_coverage_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateCoveragePath::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received coverage goal with %zu vertices", goal->polygon.size());

    // 放到 blackboard
    blackboard_->set("polygon", goal->polygon);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 处理取消请求
rclcpp_action::CancelResponse BehaviorTreeComponent::handle_nav_coverage_cancel(
    const std::shared_ptr<GoalHandleNavigateCoveragePath> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel coverage goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 处理接收的目标（异步执行任务）
void BehaviorTreeComponent::handle_nav_coverage_accepted(
    const std::shared_ptr<GoalHandleNavigateCoveragePath> goal_handle)
{
    std::thread{std::bind(&BehaviorTreeComponent::execute_nav_coverage_path, this, goal_handle)}.detach();
}

// 执行逻辑（这里模拟导航过程）
void BehaviorTreeComponent::execute_nav_coverage_path(
    const std::shared_ptr<GoalHandleNavigateCoveragePath> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Start executing BT for NavigateCoveragePath");

    auto result = std::make_shared<NavigateCoveragePath::Result>();
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        nav_coverage_path_tree_->haltTree();
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Coverage goal canceled");
        return;
      }

      status_ = nav_coverage_path_tree_->tickOnce();

      if (status_ == BT::NodeStatus::SUCCESS) {
        nav_coverage_path_tree_->haltTree();
        RCLCPP_INFO(get_logger(), "Coverage BT finished successfully!");
        result->success = true;
        goal_handle->succeed(result);
        return;
      } else if (status_ == BT::NodeStatus::FAILURE) {
        nav_coverage_path_tree_->haltTree();
        RCLCPP_ERROR(get_logger(), "Coverage BT failed!");
        result->success = false;
        goal_handle->abort(result);
        return;
      }

      loop_rate.sleep();
    }
}

} //namespace robot_behavior