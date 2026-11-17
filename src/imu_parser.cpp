#include "imu_parser.h"
#include <algorithm>
#include <limits>

const std::array<uint8_t, 2> ImuParser::HEADER_ = {ImuParser::HEADER_BYTE0, ImuParser::HEADER_BYTE1};

ImuParser::ImuParser() : checksum_fails_(0) {
  buf_.reserve(READ_BUF_SIZE);
}

void ImuParser::Feed(const uint8_t* data, size_t len) {
  buf_.insert(buf_.end(), data, data + len);
}

bool ImuParser::Parse(ImuRawData& out) {
  // 至少凑够一整帧再尝试解析
  while (buf_.size() >= FRAME_SIZE) {
    // 搜索帧头 4E 4A
    auto it = std::search(buf_.begin(), buf_.end(), HEADER_.begin(), HEADER_.end());
    if (it == buf_.end()) {
      // 保留最后 1 字节，可能是被截断的帧头首字节
      buf_.clear();
      return false;
    }

    size_t pos = static_cast<size_t>(std::distance(buf_.begin(), it));

    // 帧头后至少需要有「长度 + 命令」两个字节
    if (buf_.size() - pos < HEADER_SIZE + 2) {
      return false;
    }

    // 长度域必须是 0x13
    if (buf_[pos + 2] != LENGTH_BYTE) {
      buf_.erase(buf_.begin(), buf_.begin() + pos + 1);
      continue;
    }

    // 等待整帧到齐
    if (buf_.size() - pos < FRAME_SIZE) {
      return false;
    }

    if (!validateChecksum(pos)) {
      if (checksum_fails_ == std::numeric_limits<size_t>::max())
        checksum_fails_ = 0;
      else
        ++checksum_fails_;

      buf_.erase(buf_.begin(), buf_.begin() + pos + 1);
      continue;
    }

    uint8_t cmd = buf_[pos + 3];
    if (cmd != CMD_SENSOR) {
      // 零偏帧（0x00）或其他帧：丢弃，不算有效数据帧
      buf_.erase(buf_.begin(), buf_.begin() + pos + FRAME_SIZE);
      continue;
    }

    // 解析 9 个 int16 数据字段（Ax, Ay, Az, Wx, Wy, Wz, Hx, Hy, Hz）
    size_t data_off = pos + HEADER_SIZE + 2; // 帧头(2) + 长度(1) + 命令(1)
    out            = ImuRawData();
    out.valid      = true;
    out.ax         = readI16(data_off + 0);
    out.ay         = readI16(data_off + 2);
    out.az         = readI16(data_off + 4);
    out.wx         = readI16(data_off + 6);
    out.wy         = readI16(data_off + 8);
    out.wz         = readI16(data_off + 10);
    out.hx         = readI16(data_off + 12);
    out.hy         = readI16(data_off + 14);
    out.hz         = readI16(data_off + 16);
    out.has_accel  = true;
    out.has_gyro   = true;
    out.has_mag    = true;

    buf_.erase(buf_.begin(), buf_.begin() + pos + FRAME_SIZE);
    return true;
  }

  return false;
}

bool ImuParser::validateChecksum(size_t pos) const {
  // SUM = (0x4E + 0x4A + 0x13 + 0x01 + ... + HzL + HzH) & 0xFFFF
  // 校验和覆盖帧头到数据域结束（共 22 字节），SUM_L 在低地址、SUM_H 在高地址
  uint32_t sum = 0;
  for (size_t i = pos; i < pos + FRAME_SIZE - 2; ++i) {
    sum += buf_[i];
  }

  uint8_t expected_l = static_cast<uint8_t>(sum & 0xFF);
  uint8_t expected_h = static_cast<uint8_t>((sum >> 8) & 0xFF);
  return buf_[pos + FRAME_SIZE - 2] == expected_l && buf_[pos + FRAME_SIZE - 1] == expected_h;
}

int16_t ImuParser::readI16(size_t offset) const {
  return static_cast<int16_t>(static_cast<uint16_t>(buf_[offset]) | (static_cast<uint16_t>(buf_[offset + 1]) << 8));
}
