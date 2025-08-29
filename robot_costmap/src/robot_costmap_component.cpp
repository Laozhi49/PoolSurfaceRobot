#include "robot_costmap/robot_costmap_component.hpp"
#include <cmath>

namespace robot_costmap
{

RobotCostmapComponent::RobotCostmapComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("robot_costmap_component", options)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotCostmapComponent::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring RobotCostmapComponent...");

  // 从参数服务器读取参数
  this->declare_parameter<std::string>("map_topic", "/map");
  this->declare_parameter<std::string>("global_costmap_topic", "/global_costmap");
  this->declare_parameter<double>("publish_frequency", 10.0);
  this->declare_parameter<double>("inflation_radius", 0.2);

  map_topic_ = this->get_parameter("map_topic").as_string();
  global_costmap_topic_ = this->get_parameter("global_costmap_topic").as_string();
  publish_frequency_ = this->get_parameter("publish_frequency").as_double();
  inflation_radius_ = this->get_parameter("inflation_radius").as_double();

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, 10,
    std::bind(&RobotCostmapComponent::mapCallback, this, std::placeholders::_1)
  );

  rclcpp::QoS pub_qos = rclcpp::QoS(1).transient_local().reliable();
  
  global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      global_costmap_topic_, pub_qos);

  // global_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
  //   global_costmap_topic_, 10
  // );

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotCostmapComponent::on_activate(const rclcpp_lifecycle::State &)
{
  global_costmap_pub_->on_activate();

  auto period = std::chrono::duration<double>(1.0 / publish_frequency_);
  timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&RobotCostmapComponent::publishCostmap, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotCostmapComponent::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  global_costmap_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void RobotCostmapComponent::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = msg;
}

void RobotCostmapComponent::publishCostmap()
{
  if (!latest_map_) return;

  auto costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>(*latest_map_);

  // 膨胀处理
  inflateCostmap(*costmap, inflation_radius_);

  global_costmap_pub_->publish(*costmap);
}

void RobotCostmapComponent::inflateCostmap(nav_msgs::msg::OccupancyGrid & costmap, double inflation_radius)
{
  unsigned int width = costmap.info.width;
  unsigned int height = costmap.info.height;
  double resolution = costmap.info.resolution;

  std::vector<int8_t> inflated_data = costmap.data;

  int inflation_cells = static_cast<int>(inflation_radius / resolution);

  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      int idx = y * width + x;
      if (costmap.data[idx] >= 100) { // 原障碍
        // 对周围格子进行膨胀
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height))
              continue;
            int nidx = ny * width + nx;
            double distance = std::hypot(dx, dy) * resolution;
            if (distance <= inflation_radius) {
              inflated_data[nidx] = std::max(inflated_data[nidx], int8_t(100));
            }
          }
        }
      }
    }
  }

  costmap.data = inflated_data;
}

} // namespace robot_costmap

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_costmap::RobotCostmapComponent)
