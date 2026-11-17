#include "imu_driver_node.h"
#include <chrono>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace {
// DA901S 寄存器起始地址与读取数量：0x46(RollL) ~ 0x55(Q3H)，共 16 个寄存器
constexpr uint8_t REG_ATTITUDE_START = 0x46;
constexpr uint8_t REG_ATTITUDE_COUNT = 16;
constexpr size_t  REG_ATTITUDE_BYTES = 16;
} // namespace

ImuDriverNode::ImuDriverNode() : Node("imu_ros_publisher"), total_frames_(0), last_diag_time_(this->now()) {
  loadParams();
}

void ImuDriverNode::loadParams() {
  // 串口参数（DA901S 默认 230400、应答模式主机轮询）
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud", 230400);
  this->declare_parameter<int>("timeout_ms", 20);
  this->declare_parameter<int>("poll_rate_hz", 200);

  // 发布参数
  this->declare_parameter<bool>("publish_custom", false);
  this->declare_parameter<bool>("publish_sensor_msgs", true);
  this->declare_parameter<bool>("publish_temperature", true);
  this->declare_parameter<bool>("publish_attitude", true);
  this->declare_parameter<std::string>("frame_id", "imu_link");
  this->declare_parameter<std::string>("euler_unit", "deg");

  // 寄存器协议参数
  this->declare_parameter<bool>("enable_device_attitude", true);
  this->declare_parameter<int>("register_poll_divider", 10);

  // 内部 AHRS（ImpVQF SDK）配置：采样周期由 poll_rate_hz 决定
  this->declare_parameter<bool>("enable_attitude_estimation", true);
  this->declare_parameter<double>("ahrs_tau_acc", 3.0);
  this->declare_parameter<double>("ahrs_tau_mag", 9.0);
  this->declare_parameter<int>("ahrs_options", 15);

  // 读取参数
  port_         = this->get_parameter("port").as_string();
  baud_         = this->get_parameter("baud").as_int();
  timeout_ms_   = this->get_parameter("timeout_ms").as_int();
  poll_rate_hz_ = this->get_parameter("poll_rate_hz").as_int();
  if (poll_rate_hz_ <= 0)
    poll_rate_hz_ = 200;

  publish_custom_      = this->get_parameter("publish_custom").as_bool();
  publish_sensor_msgs_ = this->get_parameter("publish_sensor_msgs").as_bool();
  publish_temperature_ = this->get_parameter("publish_temperature").as_bool();
  publish_attitude_    = this->get_parameter("publish_attitude").as_bool();
  frame_id_            = this->get_parameter("frame_id").as_string();
  euler_unit_          = this->get_parameter("euler_unit").as_string();

  enable_device_attitude_ = this->get_parameter("enable_device_attitude").as_bool();
  register_poll_divider_  = this->get_parameter("register_poll_divider").as_int();
  if (register_poll_divider_ <= 0)
    register_poll_divider_ = 1;

  enable_attitude_estimation_ = this->get_parameter("enable_attitude_estimation").as_bool();
  ahrs_tau_acc_               = this->get_parameter("ahrs_tau_acc").as_double();
  ahrs_tau_mag_               = this->get_parameter("ahrs_tau_mag").as_double();
  ahrs_options_               = static_cast<uint32_t>(this->get_parameter("ahrs_options").as_int());

  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  port = %s", port_.c_str());
  RCLCPP_INFO(this->get_logger(), "  baud = %d", baud_);
  RCLCPP_INFO(this->get_logger(), "  timeout_ms = %d", timeout_ms_);
  RCLCPP_INFO(this->get_logger(), "  poll_rate_hz = %d", poll_rate_hz_);
  RCLCPP_INFO(this->get_logger(), "  publish_custom = %s", publish_custom_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  publish_sensor_msgs = %s", publish_sensor_msgs_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  publish_temperature = %s", publish_temperature_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  publish_attitude = %s", publish_attitude_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  euler_unit = %s", euler_unit_.c_str());
  RCLCPP_INFO(this->get_logger(), "  frame_id = %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "  enable_device_attitude = %s", enable_device_attitude_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  register_poll_divider = %d", register_poll_divider_);
  RCLCPP_INFO(this->get_logger(), "  enable_attitude_estimation = %s", enable_attitude_estimation_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  ahrs_tau_acc = %.3f, ahrs_tau_mag = %.3f", ahrs_tau_acc_, ahrs_tau_mag_);
  RCLCPP_INFO(this->get_logger(), "  ahrs_options = %u", ahrs_options_);
}

bool ImuDriverNode::Init() {
  serial_ptr_ = std::make_unique<SerialPort>(port_, baud_, timeout_ms_, this->get_logger());
  if (!serial_ptr_->Open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
    return false;
  }

  parser_ptr_          = std::make_unique<ImuParser>();
  register_parser_ptr_ = std::make_unique<RegisterParser>();

  if (enable_attitude_estimation_) {
    imu_algorithm::ImpvqfEstimator::Config cfg;
    cfg.sample_rate_hz = static_cast<double>(poll_rate_hz_);
    cfg.tau_acc        = ahrs_tau_acc_;
    cfg.tau_mag        = ahrs_tau_mag_;
    cfg.options        = ahrs_options_;

    ahrs_ptr_ = std::make_shared<imu_algorithm::ImpvqfEstimator>();
    if (!ahrs_ptr_->Init(cfg)) {
      RCLCPP_ERROR(this->get_logger(), "ImpVQF AHRS init failed: status=%d", ahrs_ptr_->LastStatus());
      ahrs_ptr_.reset();
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "ImpVQF AHRS ready: rate=%.1f Hz, tau_acc=%.2f, tau_mag=%.2f, options=%u",
                cfg.sample_rate_hz, cfg.tau_acc, cfg.tau_mag, cfg.options);
  } else {
    ahrs_ptr_ = nullptr;
    RCLCPP_INFO(this->get_logger(), "Internal SDK AHRS disabled");
  }

  publisher_ptr_ = std::make_unique<ImuPublisher>(this, publish_custom_, publish_sensor_msgs_, publish_temperature_,
                                                  publish_attitude_, euler_unit_ != "rad", frame_id_, ahrs_ptr_);

  last_diag_time_ = this->now();
  return true;
}

void ImuDriverNode::Run() {
  uint8_t data_buffer[ImuParser::READ_BUF_SIZE];

  // DA901S 默认为应答模式：主机每次先发查询指令，设备才返回一帧数据
  const auto poll_period = std::chrono::microseconds(1000000 / static_cast<long>(poll_rate_hz_));
  auto       next_poll   = std::chrono::steady_clock::now();
  const auto read_window = std::chrono::milliseconds(timeout_ms_ > 0 ? timeout_ms_ : 20);

  size_t        poll_counter = 0;
  RegisterFrame reg_frame;

  while (rclcpp::ok()) {
    if (!serial_ptr_->Write(ImuParser::QUERY_FRAME.data(), ImuParser::QUERY_FRAME.size())) {
      std::this_thread::sleep_for(poll_period);
      continue;
    }

    // 在一次轮询窗口内持续接收，直到解析出完整一帧（串口可能分多次返回字节）
    ImuRawData raw;
    bool       got_frame  = false;
    auto       read_start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
      size_t n = serial_ptr_->Read(data_buffer, sizeof(data_buffer));
      if (n > 0) {
        parser_ptr_->Feed(data_buffer, n);
        if (parser_ptr_->Parse(raw)) {
          got_frame = true;
          break;
        }
      }
      if (std::chrono::steady_clock::now() - read_start >= read_window)
        break;
    }

    if (got_frame) {
      ++poll_counter;

      // 按分频读取设备板载姿态/温度寄存器（0x46 ~ 0x55）
      if (enable_device_attitude_ && (poll_counter % static_cast<size_t>(register_poll_divider_) == 0)) {
        auto req = RegisterParser::BuildReadRequest(REG_ATTITUDE_START, REG_ATTITUDE_COUNT);
        if (serial_ptr_->Write(req.data(), req.size())) {
          auto   reg_start = std::chrono::steady_clock::now();
          size_t attempts  = 0;
          while (rclcpp::ok() && attempts++ < 3) {
            size_t n = serial_ptr_->Read(data_buffer, sizeof(data_buffer));
            if (n > 0) {
              register_parser_ptr_->Feed(data_buffer, n);
              if (register_parser_ptr_->Parse(reg_frame)) {
                if (reg_frame.start_reg == REG_ATTITUDE_START && reg_frame.values.size() >= REG_ATTITUDE_BYTES) {
                  auto read_i16 = [&](size_t off) -> int16_t {
                    uint16_t v = static_cast<uint16_t>(reg_frame.values[off]) |
                                 (static_cast<uint16_t>(reg_frame.values[off + 1]) << 8);
                    return static_cast<int16_t>(v);
                  };
                  device_cache_.dev_roll        = read_i16(0);
                  device_cache_.dev_pitch       = read_i16(2);
                  device_cache_.dev_yaw         = read_i16(4);
                  device_cache_.dev_temp        = read_i16(6);
                  device_cache_.dev_q0          = read_i16(8);
                  device_cache_.dev_q1          = read_i16(10);
                  device_cache_.dev_q2          = read_i16(12);
                  device_cache_.dev_q3          = read_i16(14);
                  device_cache_.has_dev_euler   = true;
                  device_cache_.has_dev_temp    = true;
                  device_cache_.has_dev_quat    = true;
                }
                break;
              }
            }
            if (std::chrono::steady_clock::now() - reg_start >= read_window)
              break;
          }
        }
      }

      // 把最近一次寄存器数据附加到本帧一起发布
      raw.dev_roll      = device_cache_.dev_roll;
      raw.dev_pitch     = device_cache_.dev_pitch;
      raw.dev_yaw       = device_cache_.dev_yaw;
      raw.dev_temp      = device_cache_.dev_temp;
      raw.dev_q0        = device_cache_.dev_q0;
      raw.dev_q1        = device_cache_.dev_q1;
      raw.dev_q2        = device_cache_.dev_q2;
      raw.dev_q3        = device_cache_.dev_q3;
      raw.has_dev_euler = device_cache_.has_dev_euler;
      raw.has_dev_temp  = device_cache_.has_dev_temp;
      raw.has_dev_quat  = device_cache_.has_dev_quat;

      if (total_frames_ == std::numeric_limits<size_t>::max())
        total_frames_ = 0;
      else
        ++total_frames_;

      publisher_ptr_->Publish(raw, this->now());
    }

    // 定期发布诊断信息
    publishDiagnostics();

    // 处理 ROS2 回调。SIGTERM 信号处理器可能刚关闭上下文，捕获该异常避免退出时崩溃
    try {
      rclcpp::spin_some(this->shared_from_this());
    } catch (const rclcpp::exceptions::RCLError&) {
      if (!rclcpp::ok())
        break;
    }

    // 按设定频率节流轮询
    next_poll += poll_period;
    auto now = std::chrono::steady_clock::now();
    if (next_poll > now)
      std::this_thread::sleep_until(next_poll);
    else
      next_poll = now;
  }
}

void ImuDriverNode::Shutdown() {
  if (serial_ptr_) {
    serial_ptr_->Close();
  }
  RCLCPP_INFO(this->get_logger(), "IMU driver node shutdown. Total frames: %zu, Checksum fails: %zu, Reg fails: %zu",
              total_frames_, parser_ptr_ ? parser_ptr_->ChecksumFailCount() : 0,
              register_parser_ptr_ ? register_parser_ptr_->ChecksumFailCount() : 0);
}

void ImuDriverNode::publishDiagnostics() {
  const rclcpp::Time now           = this->now();
  const double       diag_interval = 60.0; // 每 60 秒输出一次诊断

  if ((now - last_diag_time_).seconds() < diag_interval) {
    return;
  }

  size_t fails = parser_ptr_ ? parser_ptr_->ChecksumFailCount() : 0;
  RCLCPP_INFO(this->get_logger(), "Diagnostics: frames_ok=%zu, checksum_fails=%zu, reg_checksum_fails=%zu, ahrs_status=%d",
              total_frames_, fails, register_parser_ptr_ ? register_parser_ptr_->ChecksumFailCount() : 0,
              ahrs_ptr_ ? ahrs_ptr_->LastStatus() : 0);

  last_diag_time_ = now;
}
