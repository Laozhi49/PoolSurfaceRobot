#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class UltrasonicController : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit UltrasonicController(const std::string & node_name)
  : rclcpp_lifecycle::LifecycleNode(node_name)
  {
    // 声明参数
    this->declare_parameter("publish_rate", 10.0);
    this->declare_parameter("max_speed", 0.5);
    this->declare_parameter("safety_distance", 0.3);
  }

  // 节点配置时的回调
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    // 创建订阅者
    ultrasonic_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "ultrasonic_data", 10,
      std::bind(&UltrasonicController::ultrasonicCallback, this, std::placeholders::_1));
    
    // 创建生命周期发布者
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);
    
    RCLCPP_INFO(this->get_logger(), "Node configured");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 节点激活时的回调
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    // 激活发布者
    cmd_vel_pub_->on_activate();
    
    // 获取参数
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("max_speed", max_speed_);
    this->get_parameter("safety_distance", safety_distance_);
    
    // 创建定时器
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&UltrasonicController::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Node activated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 节点停用时的回调
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    // 停用发布者
    cmd_vel_pub_->on_deactivate();
    
    // 取消定时器
    timer_->cancel();
    
    RCLCPP_INFO(this->get_logger(), "Node deactivated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 节点清理时的回调
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    // 重置所有指针
    ultrasonic_sub_.reset();
    cmd_vel_pub_.reset();
    timer_.reset();
    
    RCLCPP_INFO(this->get_logger(), "Node cleaned up");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 节点关闭时的回调
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(this->get_logger(), "Node shutting down");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  // 超声波数据回调
  void ultrasonicCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // 存储最新的超声波数据
    std::lock_guard<std::mutex> lock(data_mutex_);
    ultrasonic_data_ = *msg;
    
    // 简单的调试输出
    RCLCPP_DEBUG(this->get_logger(), "Received ultrasonic data with %zu elements", msg->data.size());
  }

  // 控制循环
  void controlLoop()
  {
    // 获取最新的超声波数据
    std_msgs::msg::Float32MultiArray current_data;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      current_data = ultrasonic_data_;
    }
    
    // 创建速度命令
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    
    // 简单的避障逻辑
    if (!current_data.data.empty()) {
      // 找到最小的距离
      float min_distance = *std::min_element(current_data.data.begin(), current_data.data.end());
      
      if (min_distance < safety_distance_) {
        // 太近了，停止并转向
        cmd_vel->linear.x = 0.0;
        cmd_vel->angular.z = max_speed_;
      } else {
        // 安全距离内，继续前进
        cmd_vel->linear.x = max_speed_;
        cmd_vel->angular.z = 0.0;
      }
    } else {
      // 没有数据，停止
      cmd_vel->linear.x = 0.0;
      cmd_vel->angular.z = 0.0;
    }
    
    // 发布速度命令
    if (cmd_vel_pub_->is_activated()) {
      cmd_vel_pub_->publish(std::move(cmd_vel));
    }
  }

  // 成员变量
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ultrasonic_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std_msgs::msg::Float32MultiArray ultrasonic_data_;
  std::mutex data_mutex_;
  
  double publish_rate_;
  double max_speed_;
  double safety_distance_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<UltrasonicController>("ultrasonic_controller");
  
  rclcpp::spin(node->get_node_base_interface());
  
  rclcpp::shutdown();
  return 0;
}