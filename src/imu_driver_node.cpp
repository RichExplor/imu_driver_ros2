#include "imu_driver_node.h"
#include <rclcpp/rclcpp.hpp>

ImuDriverNode::ImuDriverNode() : Node("imu_ros_publisher"), total_frames_(0), last_diag_time_(this->now()) {
  loadParams();
}

void ImuDriverNode::loadParams() {
  // 串口参数
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud", 115200);
  this->declare_parameter<int>("timeout_ms", 100);

  // 发布参数
  this->declare_parameter<bool>("publish_custom", true);
  this->declare_parameter<bool>("publish_sensor_msgs", false);
  this->declare_parameter<std::string>("frame_id", "imu_link");

  // 姿态解算参数
  this->declare_parameter<bool>("enable_attitude_estimation", true);
  this->declare_parameter<std::string>("algorithm_type", "complementary");
  this->declare_parameter<std::string>("axis_mode", "9");
  this->declare_parameter<double>("alpha_acc", 0.02);
  this->declare_parameter<double>("alpha_mag", 0.01);

  // 读取参数
  port_       = this->get_parameter("port").as_string();
  baud_       = this->get_parameter("baud").as_int();
  timeout_ms_ = this->get_parameter("timeout_ms").as_int();

  publish_custom_      = this->get_parameter("publish_custom").as_bool();
  publish_sensor_msgs_ = this->get_parameter("publish_sensor_msgs").as_bool();
  frame_id_            = this->get_parameter("frame_id").as_string();

  enable_attitude_estimation_ = this->get_parameter("enable_attitude_estimation").as_bool();
  algorithm_type_             = this->get_parameter("algorithm_type").as_string();
  axis_mode_                  = this->get_parameter("axis_mode").as_string();
  alpha_acc_                  = this->get_parameter("alpha_acc").as_double();
  alpha_mag_                  = this->get_parameter("alpha_mag").as_double();

  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  port = %s", port_.c_str());
  RCLCPP_INFO(this->get_logger(), "  baud = %d", baud_);
  RCLCPP_INFO(this->get_logger(), "  timeout_ms = %d", timeout_ms_);
  RCLCPP_INFO(this->get_logger(), "  publish_custom = %s", publish_custom_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  publish_sensor_msgs = %s", publish_sensor_msgs_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  frame_id = %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "  enable_attitude_estimation = %s", enable_attitude_estimation_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  algorithm_type = %s", algorithm_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "  axis_mode = %s", axis_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "  alpha_acc = %.6f", alpha_acc_);
  RCLCPP_INFO(this->get_logger(), "  alpha_mag = %.6f", alpha_mag_);
}

bool ImuDriverNode::Init() {
  serial_ptr_ = std::make_unique<SerialPort>(port_, baud_, timeout_ms_, this->get_logger());
  if (!serial_ptr_->Open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
    return false;
  }

  parser_ptr_ = std::make_unique<ImuParser>();

  if (enable_attitude_estimation_) {
    auto algo_type = imu_algorithm::AttitudeEstimator::AlgorithmFromString(algorithm_type_);
    auto axis_mode = imu_algorithm::AttitudeEstimator::AxisModeFromString(axis_mode_);
    attitude_estimator_ptr_ =
        std::make_shared<imu_algorithm::AttitudeEstimator>(algo_type, axis_mode, alpha_acc_, alpha_mag_);

    RCLCPP_INFO(this->get_logger(),
                "Attitude estimator created: algorithm=%s, axis_mode=%s, "
                "alpha_acc=%.4f, alpha_mag=%.4f",
                attitude_estimator_ptr_->GetAlgorithmName(), attitude_estimator_ptr_->GetAxisModeName(), alpha_acc_,
                alpha_mag_);
  } else {
    attitude_estimator_ptr_ = nullptr;
    RCLCPP_INFO(this->get_logger(), "Attitude estimation disabled, publishing "
                                    "raw data with identity quaternion");
  }

  publisher_ptr_ =
      std::make_unique<ImuPublisher>(this, publish_custom_, publish_sensor_msgs_, frame_id_, attitude_estimator_ptr_);

  last_diag_time_ = this->now();
  return true;
}

void ImuDriverNode::Run() {
  uint8_t data_buffer[ImuParser::READ_BUF_SIZE];

  while (rclcpp::ok()) {
    size_t n = serial_ptr_->Read(data_buffer, sizeof(data_buffer));
    if (n == 0) {
      // 处理 ROS2 回调（参数更新等）
      rclcpp::spin_some(this->shared_from_this());
      continue;
    }

    parser_ptr_->Feed(data_buffer, n);

    ImuRawData raw;
    while (parser_ptr_->Parse(raw)) {
      if (total_frames_ == std::numeric_limits<size_t>::max())
        total_frames_ = 0;
      else
        ++total_frames_;

      publisher_ptr_->Publish(raw, this->now());
    }

    // 定期发布诊断信息
    publishDiagnostics();

    // 处理 ROS2 回调
    rclcpp::spin_some(this->shared_from_this());
  }
}

void ImuDriverNode::Shutdown() {
  if (serial_ptr_) {
    serial_ptr_->Close();
  }
  RCLCPP_INFO(this->get_logger(), "IMU driver node shutdown. Total frames: %zu, Checksum fails: %zu", total_frames_,
              parser_ptr_ ? parser_ptr_->ChecksumFailCount() : 0);
}

void ImuDriverNode::publishDiagnostics() {
  const rclcpp::Time now           = this->now();
  const double       diag_interval = 60.0; // 每 60 秒输出一次诊断

  if ((now - last_diag_time_).seconds() < diag_interval) {
    return;
  }

  size_t fails = parser_ptr_ ? parser_ptr_->ChecksumFailCount() : 0;
  RCLCPP_INFO(this->get_logger(), "Diagnostics: frames_ok=%zu, checksum_fails=%zu", total_frames_, fails);

  last_diag_time_ = now;
}
