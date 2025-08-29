#ifndef IMU_PROCESSOR_NODE_HPP
#define IMU_PROCESSOR_NODE_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define GzOFFSET -0.01702723f

class IMUProcessorNode : public rclcpp::Node
{
public:
  IMUProcessorNode();

private:
  Eigen::Matrix3f R_;
  // 互补滤波算法实现
  void IMUupdate(float ax, float ay, float az, float gx, float gy, float gz);
  
  // ROS回调函数
  void imu_raw_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  void imu_calibrate(float gz);
  
  void display_euler();
  // ROS接口
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr raw_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  
  // 算法参数
  struct {
    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    float exInt = 0.0f;
    float eyInt = 0.0f;
    float ezInt = 0.0f;
    const float Kp = 10.0f;
    const float Ki = 0.000f;
    const float halfT = 0.005f;
  } filter_state_;

  // 校准参数
  float gz_offset;
  int   cnt;
  bool  calibrate_flag = false;
  float gyroZMax, gyroZMin;
  float gz_diff;

  // 参数变量
  std::array<double, 9> angular_vel_cov_;
  std::array<double, 9> linear_accel_cov_;
  std::array<double, 9> orientation_cov_;
  std::string frame_id_;
};

#endif  // IMU_PROCESSOR_NODE_HPP