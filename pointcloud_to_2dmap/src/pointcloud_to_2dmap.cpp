#include "pointcloud_to_2dmap/pointcloud_to_2dmap.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace pointcloud_to_2dmap {

PointCloudTo2DMap::PointCloudTo2DMap(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("pointcloud_to_2dmap", options),
  camerapose_received_(false),
  pointcloud_received_(false)
{
  // Declare parameters
  this->declare_parameter("pointcloud_topic", "/orbslam3/map_points");
  this->declare_parameter("camerapose_topic", "/orbslam3/camera_pose");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("robot_frame", "map");
  this->declare_parameter("map_topic", "/map");
  this->declare_parameter("map_resolution", 0.05);
  this->declare_parameter("map_width", 20.0);
  this->declare_parameter("map_height", 20.0);
  this->declare_parameter("robot_height", 0.5);
  this->declare_parameter("camera_height", 0.4);
  this->declare_parameter("obstacle_threshold", 0.2);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PointCloudTo2DMap::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring...");
  
  // Get parameters
  this->get_parameter("pointcloud_topic", pointcloud_topic_);
  this->get_parameter("camerapose_topic", camerapose_topic_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("robot_frame", robot_frame_);
  this->get_parameter("map_topic", map_topic_);
  this->get_parameter("map_resolution", map_resolution_);
  this->get_parameter("map_width", map_width_);
  this->get_parameter("map_height", map_height_);
  this->get_parameter("robot_height", robot_height_);
  this->get_parameter("camera_height", camera_height_);
  this->get_parameter("obstacle_threshold", obstacle_threshold_);
  
  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize map
  map_.header.frame_id = map_frame_;
  map_.info.resolution = map_resolution_;
  map_.info.width = static_cast<unsigned int>(map_width_ / map_resolution_);
  map_.info.height = static_cast<unsigned int>(map_height_ / map_resolution_);
  map_.info.origin.position.x = -map_width_ / 2.0;
  map_.info.origin.position.y = -map_height_ / 2.0;
  map_.info.origin.position.z = 0.0;
  map_.info.origin.orientation.w = 1.0;
  map_.data.resize(map_.info.width * map_.info.height, -1);  // -1 for unknown
  
  map_.data.assign(map_.info.width * map_.info.height, 0);  // 全部初始化为空闲

  // Create publisher (not activated yet)
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::SystemDefaultsQoS());
  
  RCLCPP_INFO(get_logger(), "Configuration completed.");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PointCloudTo2DMap::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating...");
  
  // Create subscribers
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PointCloudTo2DMap::pointCloudCallback, this, _1));
  
  camerapose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    camerapose_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&PointCloudTo2DMap::poseCallback, this, _1));
  
  // Activate publisher
  map_pub_->on_activate();
  
  RCLCPP_INFO(get_logger(), "Activation completed.");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PointCloudTo2DMap::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");
  
  // Deactivate publisher
  map_pub_->on_deactivate();
  
  // Reset subscribers
  pointcloud_sub_.reset();
  camerapose_sub_.reset();
  
  RCLCPP_INFO(get_logger(), "Deactivation completed.");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PointCloudTo2DMap::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");
  
  // Reset everything
  map_pub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  
  camerapose_received_ = false;
  pointcloud_received_ = false;
  
  RCLCPP_INFO(get_logger(), "Cleanup completed.");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PointCloudTo2DMap::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "Shutting down from state %s.",
    state.label().c_str());
  
  // Reset everything
  map_pub_.reset();
  pointcloud_sub_.reset();
  camerapose_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// 点云数据回调
void PointCloudTo2DMap::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!camerapose_received_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "No pose received yet. Skipping point cloud.");
    return;
  }
  
  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  
  // Process point cloud
  processPointCloud(cloud, latest_camerapose_.pose);
  pointcloud_received_ = true;
  
  // Update and publish map
  updateMap();
}

// 摄像头位姿回调
void PointCloudTo2DMap::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  try {
    // Transform pose to map frame
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, msg->header.frame_id, msg->header.stamp, 1s);
    
    geometry_msgs::msg::PoseStamped transformed_pose;
    tf2::doTransform(*msg, transformed_pose, transform);
    
    latest_camerapose_ = transformed_pose;
    camerapose_received_ = true;
    
    if (pointcloud_received_) {
      updateMap();
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "TF exception: %s", ex.what());
  }
}

void PointCloudTo2DMap::processPointCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  const geometry_msgs::msg::Pose &camera_pose)
{
  // Filter points based on robot height (obstacle detection)
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  for (const auto &point : cloud->points) {
    // Check if point is within the height range to be considered an obstacle
    if (point.z > (camera_pose.position.z - camera_height_) &&
        point.z < (camera_pose.position.z + robot_height_ - camera_height_)) {
      filtered_cloud->points.push_back(point);
    }
  }
  
  // Transform points to map frame
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, cloud->header.frame_id, pcl_conversions::fromPCL(cloud->header.stamp), 1s);
    
    Eigen::Isometry3d eigen_transform = tf2::transformToEigen(transform);
    pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, eigen_transform.matrix());
    
    // Update map with filtered points
    for (const auto &point : filtered_cloud->points) {
      // Convert point to map coordinates
      int mx = static_cast<int>((point.x - map_.info.origin.position.x) / map_.info.resolution);
      int my = static_cast<int>((point.y - map_.info.origin.position.y) / map_.info.resolution);
      
      // Check if point is within map bounds
      if (mx >= 0 && mx < static_cast<int>(map_.info.width) &&
          my >= 0 && my < static_cast<int>(map_.info.height)) {
        // Mark as occupied (100)
        map_.data[my * map_.info.width + mx] = 100;
      }
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "TF exception: %s", ex.what());
  }
}

void PointCloudTo2DMap::updateMap()
{
  // Update map header
  map_.header.stamp = this->now();
  
  // Publish map
  map_pub_->publish(map_);

  // 清空map
  // map_.data.assign(map_.info.width * map_.info.height, 0); 
}

} // namespace pointcloud_to_2dmap

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_2dmap::PointCloudTo2DMap)