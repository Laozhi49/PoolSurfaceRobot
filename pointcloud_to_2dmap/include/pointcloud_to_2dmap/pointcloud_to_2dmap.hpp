#ifndef POINTCLOUD_TO_2DMAP_HPP
#define POINTCLOUD_TO_2DMAP_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

namespace pointcloud_to_2dmap {

class PointCloudTo2DMap : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PointCloudTo2DMap(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  
  // Lifecycle node callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const geometry_msgs::msg::Pose &robot_pose);
  void updateMap();
  
  // Parameters
  std::string pointcloud_topic_;
  std::string pose_topic_;
  std::string map_frame_;
  std::string robot_frame_;
  double map_resolution_;
  double map_width_;
  double map_height_;
  double robot_height_;
  double obstacle_threshold_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  
  // Publisher
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  
  // Data
  nav_msgs::msg::OccupancyGrid map_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  bool pose_received_;
  bool pointcloud_received_;
};

} // namespace pointcloud_to_2dmap

#endif  // POINTCLOUD_TO_2DMAP_HPP