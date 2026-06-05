#pragma once

#include <boost/asio.hpp>
#include <cstdint>
#include <memory>
#include <rclcpp/logger.hpp>
#include <string>

class SerialPort {
public:
  SerialPort(const std::string& port, int baud, int timeout_ms = 100,
             const rclcpp::Logger& logger = rclcpp::get_logger("serial_port"));

  ~SerialPort();

  bool Open();

  void Close();

  bool IsOpen() const;

  /// @brief 带超时的同步读取
  /// @param buf 接收缓冲区
  /// @param max_len 缓冲区最大长度
  /// @return 实际读取的字节数，超时或错误返回 0
  size_t Read(uint8_t* buf, size_t max_len);

private:
  boost::asio::io_service io_;
  std::string             port_;
  int                     baud_;
  int                     timeout_ms_;

  std::unique_ptr<boost::asio::serial_port> serial_ptr_;
  rclcpp::Logger                            logger_;
};
