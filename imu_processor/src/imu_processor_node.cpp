#include "imu_processor/imu_processor_node.hpp"
#include "tf2/LinearMath/Quaternion.h"

IMUProcessorNode::IMUProcessorNode()
: Node("imu_processor_node")
{
  // 声明参数并设置默认值
  this->declare_parameter<std::string>("frame_id", "imu_link");
  
  // 角速度协方差
  this->declare_parameter<std::vector<double>>(
    "angular_velocity_covariance",
    std::vector<double>{1.3e-6, 0, 0, 0, 1.3e-6, 0, 0, 0, 1.3e-6});
  
  // 线加速度协方差
  this->declare_parameter<std::vector<double>>(
    "linear_acceleration_covariance",
    std::vector<double>{4e-4, 0, 0, 0, 4e-4, 0, 0, 0, 4e-4});
  
  // 姿态协方差
  this->declare_parameter<std::vector<double>>(
    "orientation_covariance",
    std::vector<double>{0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05});

  // 声明参数，默认值是 false
  this->declare_parameter<bool>("calibrate_flag", false);

  // 获取参数值
  frame_id_ = this->get_parameter("frame_id").as_string();
  
  auto gyro_cov = this->get_parameter("angular_velocity_covariance").as_double_array();
  auto accel_cov = this->get_parameter("linear_acceleration_covariance").as_double_array();
  auto orient_cov = this->get_parameter("orientation_covariance").as_double_array();
  
  // 转换为std::array
  std::copy_n(gyro_cov.begin(), 9, angular_vel_cov_.begin());
  std::copy_n(accel_cov.begin(), 9, linear_accel_cov_.begin());
  std::copy_n(orient_cov.begin(), 9, orientation_cov_.begin());
  
  this->get_parameter("calibrate_flag", calibrate_flag);
  if(calibrate_flag==true)
    gz_diff = GzOFFSET;


  // 创建订阅和发布
  raw_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "imu_raw_data", 10,
    std::bind(&IMUProcessorNode::imu_raw_callback, this, std::placeholders::_1));
    
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

  /**
   * Eigen::AngleAxisf(angle, axis) 表示一个 绕某个单位轴旋转一定角度 的旋转。
   * 
   * 如果要绕两个轴旋转，可以采用旋转矩阵相乘的方式
   * Eigen::AngleAxisf rot_z(M_PI/2, Eigen::Vector3f::UnitZ());
   * Eigen::AngleAxisf rot_x(M_PI,   Eigen::Vector3f::UnitX());
   * 注意：右乘是先执行右边，再执行左边
   * Eigen::Matrix3f R = (rot_z * rot_x).toRotationMatrix();
   * 所以这表示先绕 X 转 180°再绕 Z 转 90°
   **/
  R_ = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ());

  

  RCLCPP_INFO(get_logger(), "initial node");
}

//互补滤波函数
//输入参数：g表陀螺仪角速度(弧度/s)，a表加计（m/s2或g都可以，会归一化）
void IMUProcessorNode::IMUupdate(float ax, float ay, float az, float gx, float gy, float gz)
{
  float q0temp, q1temp, q2temp, q3temp;
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;

  // 检查加速度计数据有效性
  if(ax * ay * az == 0) {
    RCLCPP_WARN(get_logger(), "Invalid accelerometer data!");
    return;
  }

  // 归一化加速度计数据
  norm = sqrt(ax * ax + ay * ay + az * az);
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  // 计算当前姿态下的重力分量
  float q0q0 = filter_state_.q0 * filter_state_.q0;
  float q0q1 = filter_state_.q0 * filter_state_.q1;
  float q0q2 = filter_state_.q0 * filter_state_.q2;
  float q1q1 = filter_state_.q1 * filter_state_.q1;
  float q1q3 = filter_state_.q1 * filter_state_.q3;
  float q2q2 = filter_state_.q2 * filter_state_.q2;
  float q2q3 = filter_state_.q2 * filter_state_.q3;
  float q3q3 = filter_state_.q3 * filter_state_.q3;

  vx = 2 * (q1q3 - q0q2);
  vy = 2 * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  // 计算误差
  ex = (ay * vz - az * vy);
  ey = (az * vx - ax * vz);
  ez = (ax * vy - ay * vx);

  // 误差积分
  filter_state_.exInt += ex * filter_state_.Ki;
  filter_state_.eyInt += ey * filter_state_.Ki;
  filter_state_.ezInt += ez * filter_state_.Ki;

  // 补偿陀螺仪数据
  gx += filter_state_.Kp * ex + filter_state_.exInt;
  gy += filter_state_.Kp * ey + filter_state_.eyInt;
  // gz += filter_state_.Kp * ez + filter_state_.ezInt;

  // 四元数更新
  q0temp = filter_state_.q0;
  q1temp = filter_state_.q1;
  q2temp = filter_state_.q2;
  q3temp = filter_state_.q3;

  filter_state_.q0 = q0temp + (-q1temp * gx - q2temp * gy - q3temp * gz) * filter_state_.halfT;
  filter_state_.q1 = q1temp + (q0temp * gx + q2temp * gz - q3temp * gy) * filter_state_.halfT;
  filter_state_.q2 = q2temp + (q0temp * gy - q1temp * gz + q3temp * gx) * filter_state_.halfT;
  filter_state_.q3 = q3temp + (q0temp * gz + q1temp * gy - q2temp * gx) * filter_state_.halfT;

  // 归一化四元数
  norm = sqrt(filter_state_.q0 * filter_state_.q0 + 
             filter_state_.q1 * filter_state_.q1 + 
             filter_state_.q2 * filter_state_.q2 + 
             filter_state_.q3 * filter_state_.q3);
  
  filter_state_.q0 /= norm;
  filter_state_.q1 /= norm;
  filter_state_.q2 /= norm;
  filter_state_.q3 /= norm;
}

void IMUProcessorNode::imu_calibrate(float gz)
{
  static const uint16_t CaliTimes = 1000;

  gz_offset += gz;

  // 记录极差（仅Z轴）
  if (cnt == 0) {
      gyroZMax = gyroZMin = gz;
  } else {
      if (gz > gyroZMax) gyroZMax = gz;
      if (gz < gyroZMin) gyroZMin = gz;
  }

  cnt += 1;
  gz_diff = gyroZMax - gyroZMin;
  RCLCPP_INFO(get_logger(), "gz_diff = %.8f",gz_diff);
  // 检查数据稳定性（仅Z轴）
  if (gz_diff > 0.03f) {
      // 数据不稳定，重新开始校准
      cnt = 0;
      gz_offset = 0;
      gyroZMax = gyroZMin = 0;
  }

  if(cnt >= CaliTimes)
  {
    // 计算Z轴零偏平均值
    gz_offset /= (float)CaliTimes;
    
    // 简单验证（可选）
    if (fabsf(gz_offset) > 0.05f) {
        // 零偏过大，使用默认值
        gz_offset = GzOFFSET;
    }
    calibrate_flag = true;
    RCLCPP_INFO(get_logger(), "gz_offset = %.8f",gz_offset);
  }
}

void IMUProcessorNode::imu_raw_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  // 检查数据长度
  if (msg->data.size() != 6) {
    RCLCPP_WARN(get_logger(), "Invalid IMU raw data length, expected 6 got %zu", msg->data.size());
    return;
  }
  // 加速度: g → m/s²
  constexpr double G_TO_MS2 = 9.80665;
  
  // 提取原始数据
  float ax = msg->data[0] * G_TO_MS2;
  float ay = msg->data[1] * G_TO_MS2;
  float az = msg->data[2] * G_TO_MS2;

  // 角速度: dps → rad/s
  constexpr double DEG2RAD = M_PI / 180.0;
  float gx = msg->data[3] * DEG2RAD;
  float gy = msg->data[4] * DEG2RAD;
  float gz = msg->data[5] * DEG2RAD;
  
  // 旋转校正陀螺仪坐标系
  // 原始加速度、角速度
  Eigen::Vector3f acc_raw(ax, ay, az);
  Eigen::Vector3f gyro_raw(gx, gy, gz);

  // 应用旋转修正
  Eigen::Vector3f acc_fixed = R_ * acc_raw;
  Eigen::Vector3f gyro_fixed = R_ * gyro_raw;

  // 替换原始值
  ax = acc_fixed.x();
  ay = acc_fixed.y();
  az = acc_fixed.z();

  gx = gyro_fixed.x();
  gy = gyro_fixed.y();
  gz = gyro_fixed.z();

  if(calibrate_flag==false)
    imu_calibrate(gz);
  else 
  {
    gz = gz - gz_offset;
    if(fabsf(gz)<0.015f) gz = 0.000f;

    // 运行互补滤波算法
    IMUupdate(ax, ay, az, gx, gy, gz);
    
    // 创建IMU消息
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = this->now();
    imu_msg->header.frame_id = frame_id_;
    
    // 设置协方差
    std::copy(angular_vel_cov_.begin(), angular_vel_cov_.end(),
              imu_msg->angular_velocity_covariance.begin());
              
    std::copy(linear_accel_cov_.begin(), linear_accel_cov_.end(),
              imu_msg->linear_acceleration_covariance.begin());
              
    std::copy(orientation_cov_.begin(), orientation_cov_.end(),
              imu_msg->orientation_covariance.begin());

    // 设置线性加速度 (m/s^2)
    imu_msg->linear_acceleration.x = ax;
    imu_msg->linear_acceleration.y = ay;
    imu_msg->linear_acceleration.z = az;
    
    // 设置角速度 (rad/s)
    imu_msg->angular_velocity.x = gx;  // 转换为弧度
    imu_msg->angular_velocity.y = gy;
    imu_msg->angular_velocity.z = gz;
    
    // 设置四元数
    imu_msg->orientation.x = filter_state_.q1;
    imu_msg->orientation.y = filter_state_.q2;
    imu_msg->orientation.z = filter_state_.q3;
    imu_msg->orientation.w = filter_state_.q0;
    
    imu_pub_->publish(std::move(imu_msg));
    // display_euler();
  }

}

// 打印欧拉角
void IMUProcessorNode::display_euler()
{
  tf2::Quaternion q(
      filter_state_.q1,
      filter_state_.q2,
      filter_state_.q3,
      filter_state_.q0
  );

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // 转成度
  roll  = roll  * 180.0 / M_PI;
  pitch = pitch * 180.0 / M_PI;
  yaw   = yaw   * 180.0 / M_PI;

  RCLCPP_INFO(rclcpp::get_logger("imu"), "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUProcessorNode>());
  rclcpp::shutdown();
  return 0;
}