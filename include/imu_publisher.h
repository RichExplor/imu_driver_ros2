#pragma once

#include "algorithm/attitude_estimator.h"
#include "imu_parser.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <imu_driver_interfaces/msg/imu_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class ImuPublisher {
public:
  ImuPublisher(rclcpp::Node* node, bool publish_custom, bool publish_sensor_msgs, const std::string& frame_id,
               std::shared_ptr<imu_algorithm::AttitudeEstimator> estimator);

  void Publish(const ImuRawData& raw, const rclcpp::Time& stamp);

private:
  void fillCovariance(std::array<double, 9>& cov, bool unknown = false);

  /// @brief 将原始数据转换为 SI 单位，并应用缩放系数
  void convertRawData(const ImuRawData& raw, const rclcpp::Time& stamp, geometry_msgs::msg::Vector3& accel,
                      geometry_msgs::msg::Vector3& gyro, geometry_msgs::msg::Vector3& mag,
                      geometry_msgs::msg::Quaternion& quat);

  /// @brief 使用姿态解算器计算当前姿态四元数
  void attitudeEstimate(const geometry_msgs::msg::Vector3& accel, const geometry_msgs::msg::Vector3& gyro,
                        const geometry_msgs::msg::Vector3& mag, geometry_msgs::msg::Quaternion& orientation,
                        const rclcpp::Time& stamp);

private:
  rclcpp::Publisher<imu_driver_interfaces::msg::ImuData>::SharedPtr pub_custom_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr               pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr     pub_mag_;

  rclcpp::Logger logger_;

  bool        publish_custom_;
  bool        publish_sensor_msgs_;
  std::string frame_id_;

  /// @brief 姿态解算器
  std::shared_ptr<imu_algorithm::AttitudeEstimator> estimator_ptr_;

  /// @brief 上一帧时间戳，用于计算 dt
  bool         first_frame_;
  rclcpp::Time last_stamp_;

  /// @brief 协方差矩阵常量（未知方向设 -1，其余为 0）
  const double COV_UNKNOWN = -1.0;
  const double COV_ZERO    = 0.0;

  /// @brief 转换原始数据为 SI 单位（m/s^2, rad/s, T）
  static constexpr double GRAVITY = 9.80665; // 标准重力加速度

  // 新协议原始 DATA 需乘以 1e-6 得到物理量（除温度）
  static constexpr double SCALE_ACCEL           = 1e-6; // 加速度: DATA * SCALE_ACCEL -> m/s^2
  static constexpr double SCALE_GYRO            = 1e-6; // 角速度: DATA * SCALE_GYRO -> deg/s
  static constexpr double SCALE_MAG             = 1e-6; // 磁力归一化: DATA * SCALE_MAG -> 单位向量分量
  static constexpr double SCALE_MAG_STRENGTH    = 1e-3; // 磁场强度: DATA * SCALE_MAG_STRENGTH -> mGauss
  static constexpr double SCALE_NOT_DIMENSIONAL = 1e-6; // 无量纲缩放因子

  // 磁场强度转 Tesla：DATA * 0.001 (mGauss) -> Tesla = DATA * 0.001 * 1e-4
  static constexpr double SCALE_MAG_STRENGTH_TO_TESLA = 1e-7;

  /// @brief 角度转换常量
  static constexpr double Rad2Deg = 180.0 / M_PI;
  static constexpr double Deg2Rad = M_PI / 180.0;
};
