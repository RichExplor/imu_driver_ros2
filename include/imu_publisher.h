#pragma once

#include "algorithm/impvqf_estimator.h"
#include "imu_parser.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <imu_driver_interfaces/msg/imu_attitude.hpp>
#include <imu_driver_interfaces/msg/imu_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float32.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class ImuPublisher {
public:
  ImuPublisher(rclcpp::Node* node, bool publish_custom, bool publish_sensor_msgs, bool publish_temperature,
               bool publish_attitude, bool euler_unit_deg, const std::string& frame_id,
               std::shared_ptr<imu_algorithm::ImpvqfEstimator> ahrs);

  void Publish(const ImuRawData& raw, const rclcpp::Time& stamp);

private:
  void fillCovariance(std::array<double, 9>& cov, bool unknown = false);

  /// @brief 原始数据 -> 内部量：加速度 m/s^2、角速度 rad/s、磁场 mGauss；并用 SDK 更新 AHRS
  void convertRawData(const ImuRawData& raw, geometry_msgs::msg::Vector3& accel,
                      geometry_msgs::msg::Vector3& gyro, geometry_msgs::msg::Vector3& mag,
                      geometry_msgs::msg::Quaternion& ahrs_quat);

  /// @brief 从设备寄存器输出的四元数/欧拉角得到设备姿态
  void deviceAttitude(const ImuRawData& raw, geometry_msgs::msg::Quaternion& orientation) const;

  /// @brief 由四元数换算欧拉角 (roll, pitch, yaw)
  void quaternionToEuler(const geometry_msgs::msg::Quaternion& q, geometry_msgs::msg::Vector3& euler) const;

private:
  rclcpp::Publisher<imu_driver_interfaces::msg::ImuData>::SharedPtr     pub_custom_;
  rclcpp::Publisher<imu_driver_interfaces::msg::ImuAttitude>::SharedPtr pub_attitude_;
  rclcpp::Publisher<imu_driver_interfaces::msg::ImuAttitude>::SharedPtr pub_attitude_ahrs_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                   pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr         pub_mag_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                  pub_temp_;

  rclcpp::Logger logger_;
  rclcpp::Clock  clock_; ///< 用于日志节流

  bool        publish_custom_;
  bool        publish_sensor_msgs_;
  bool        publish_temperature_;
  bool        publish_attitude_;
  bool        euler_unit_deg_; ///< true: 欧拉角输出度；false: 弧度
  std::string frame_id_;

  /// @brief 内部解算 AHRS（ImpVQF SDK）
  std::shared_ptr<imu_algorithm::ImpvqfEstimator> ahrs_ptr_;

  /// @brief 协方差矩阵常量（未知方向设 -1，其余为 0）
  const double COV_UNKNOWN = -1.0;
  const double COV_ZERO    = 0.0;

  /// @brief DA901S 物理量换算（手册 2.1.2 / 4.1~4.3）
  /// 内部：加速度 m/s^2、角速度 rad/s、磁场 mGauss（SDK 接受同尺度磁场）
  /// 发布：角速度 °/s、磁场 mGauss
  static constexpr double GRAVITY = 9.80665; // 标准重力加速度（m/s^2）

  static constexpr double Rad2Deg = 180.0 / M_PI;
  static constexpr double Deg2Rad = M_PI / 180.0;

  static constexpr double ACCEL_SCALE    = 16.0 * GRAVITY / 32768.0;  ///< m/s^2 per LSB
  static constexpr double GYRO_SCALE_RAD = 2000.0 * Deg2Rad / 32768.0; ///< rad/s per LSB
  static constexpr double MAG_SCALE_MGAUSS = 1.0;                     ///< mGauss per LSB
};
