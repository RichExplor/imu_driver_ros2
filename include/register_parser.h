#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

/// @brief 一段连续寄存器的读取结果
struct RegisterFrame {
  uint8_t              start_reg = 0;  ///< 起始寄存器地址
  std::vector<uint8_t> values;         ///< 按地址递增排列的寄存器值
  bool                 valid    = false;
};

/// @brief DA901S 寄存器协议解析器（仅读取）
///
/// 请求：FF AA 27 REG NUM SUM_L SUM_H
///       NUM = 要读取的寄存器个数，SUM = 请求各字节算术和（低 16 位，低字节在前）
/// 响应：4E 4A NUM REG data[NUM-1] SUM_L SUM_H
///       响应中的 NUM = 1（REG 字节）+ 数据字节数
///       SUM = 从帧头到最后一个 data 字节的算术和（低 16 位，低字节在前）
class RegisterParser {
public:
  void Feed(const uint8_t* data, size_t len);
  bool Parse(RegisterFrame& out);

  /// @brief 构造读寄存器请求
  static std::array<uint8_t, 7> BuildReadRequest(uint8_t reg, uint8_t num);

  size_t ChecksumFailCount() const {
    return checksum_fails_;
  }

  static constexpr uint8_t HEADER_BYTE0 = 0x4E;
  static constexpr uint8_t HEADER_BYTE1 = 0x4A;

private:
  bool validateChecksum(size_t pos, size_t data_count) const;

  std::vector<uint8_t> buf_;
  size_t               checksum_fails_ = 0;
};
