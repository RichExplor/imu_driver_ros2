#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

/// @brief DA901S 固定协议传感器原始数据（小端 int16）
struct ImuRawData {
  int16_t ax, ay, az; ///< 加速度原始值，物理值 = raw / 32768 * 16 g
  int16_t wx, wy, wz; ///< 角速度原始值，物理值 = raw / 32768 * 2000 °/s
  int16_t hx, hy, hz; ///< 磁场原始值，1 LSB = 1 mGauss

  // 来自寄存器协议的设备姿态与温度（板载解算结果，低频更新）
  int16_t dev_roll, dev_pitch, dev_yaw; ///< 欧拉角原始值，角度 = raw/32768*180°
  int16_t dev_q0, dev_q1, dev_q2, dev_q3; ///< 四元数原始值，分量 = raw/32768
  int16_t dev_temp;                       ///< 温度原始值，摄氏度 = raw/100

  bool has_accel;
  bool has_gyro;
  bool has_mag;
  bool has_dev_euler;
  bool has_dev_quat;
  bool has_dev_temp;
  bool valid; ///< 校验是否通过

  ImuRawData()
      : ax(0), ay(0), az(0), wx(0), wy(0), wz(0), hx(0), hy(0), hz(0), dev_roll(0), dev_pitch(0), dev_yaw(0),
        dev_q0(0), dev_q1(0), dev_q2(0), dev_q3(0), dev_temp(0), has_accel(false), has_gyro(false), has_mag(false),
        has_dev_euler(false), has_dev_quat(false), has_dev_temp(false), valid(false) {
  }
};

/// @brief DA901S 固定协议解析器
///
/// 帧格式（共 24 字节，小端）：
///   0x4E 0x4A | 0x13 | CMD | AxL AxH ... HzL HzH | SUM_L SUM_H
///   SUM = (帧头..HzH 全部字节算术和) & 0xFFFF，低字节在前
class ImuParser {
public:
  ImuParser();

  /// @brief 向解析器输入新数据
  void Feed(const uint8_t* data, size_t len);

  /// @brief 尝试解析一帧传感器数据
  /// @return 成功解析返回 true（out 填充），否则返回 false
  bool Parse(ImuRawData& out);

  size_t ChecksumFailCount() const {
    return checksum_fails_;
  }

  // 协议常量
  static constexpr uint8_t HEADER_BYTE0 = 0x4E;
  static constexpr uint8_t HEADER_BYTE1 = 0x4A;
  static constexpr uint8_t LENGTH_BYTE  = 0x13; ///< 1 字节命令 + 18 字节数据
  static constexpr uint8_t CMD_SENSOR   = 0x01; ///< 传感器数据返回

  static constexpr size_t HEADER_SIZE   = 2;
  static constexpr size_t FRAME_SIZE    = 24; ///< 2 帧头 + 1 长度 + 1 命令 + 18 数据 + 2 校验
  static constexpr size_t READ_BUF_SIZE = 512; ///< 单次读取缓冲区大小

  /// @brief 主机查询指令：FF AA 27 34 00（读取全部传感器数据）
  static constexpr std::array<uint8_t, 5> QUERY_FRAME{{0xFF, 0xAA, 0x27, 0x34, 0x00}};

private:
  /// @brief 验证从 pos 开始的帧的校验和
  bool validateChecksum(size_t pos) const;

  /// @brief 从缓冲区读取小端 int16
  int16_t readI16(size_t offset) const;

  std::vector<uint8_t>                buf_;
  static const std::array<uint8_t, 2> HEADER_;
  size_t                              checksum_fails_;
};
