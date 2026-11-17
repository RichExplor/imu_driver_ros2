#pragma once

#include "algorithm/impvqf_estimator.h"
#include "imu_parser.h"
#include "imu_publisher.h"
#include "register_parser.h"
#include "serial_port.h"
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>

/// @brief IMU 驱动顶层节点，组合串口、解析、姿态解算、发布四个模块
class ImuDriverNode : public rclcpp::Node {
public:
  ImuDriverNode();

  bool Init();

  void Run();

  void Shutdown();

private:
  void loadParams();

  void publishDiagnostics();

private:
  std::unique_ptr<SerialPort>                       serial_ptr_;
  std::unique_ptr<ImuParser>                        parser_ptr_;
  std::unique_ptr<RegisterParser>                   register_parser_ptr_;
  std::unique_ptr<ImuPublisher>                     publisher_ptr_;
  std::shared_ptr<imu_algorithm::ImpvqfEstimator>   ahrs_ptr_;

  /// @brief 最近一次寄存器读取的设备姿态/温度缓存
  ImuRawData device_cache_;

  // 串口参数
  std::string port_;
  int         baud_;
  int         timeout_ms_;
  int         poll_rate_hz_; ///< 主机轮询频率（应答模式下主动查询传感器）

  // 发布参数
  bool        publish_custom_;
  bool        publish_sensor_msgs_;
  bool        publish_temperature_;
  bool        publish_attitude_;
  std::string frame_id_;
  std::string euler_unit_; ///< 欧拉角输出单位："deg"（默认）或 "rad"

  // 寄存器协议参数
  bool        enable_device_attitude_; ///< 是否读取设备板载姿态/温度寄存器
  int         register_poll_divider_;  ///< 每 N 次传感器轮询读取一次寄存器

  // 内部 AHRS（ImpVQF SDK）配置
  bool     enable_attitude_estimation_; ///< 是否启用内部 SDK AHRS
  double   ahrs_tau_acc_;               ///< 倾角滤波时间常数 (s)
  double   ahrs_tau_mag_;               ///< 磁航向修正时间常数 (s)
  uint32_t ahrs_options_;               ///< SDK 选项位掩码

  // 诊断统计
  size_t       total_frames_;
  rclcpp::Time last_diag_time_;
};
