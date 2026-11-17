#include "register_parser.h"
#include <algorithm>
#include <limits>

void RegisterParser::Feed(const uint8_t* data, size_t len) {
  buf_.insert(buf_.end(), data, data + len);
}

bool RegisterParser::Parse(RegisterFrame& out) {
  // 最小响应：帧头(2) + NUM(1) + REG(1) + 至少 1 字节数据 + 校验(2)
  while (buf_.size() >= 2 + 1 + 1 + 1 + 2) {
    const std::array<uint8_t, 2> header = {HEADER_BYTE0, HEADER_BYTE1};
    auto                         it     = std::search(buf_.begin(), buf_.end(), header.begin(), header.end());
    if (it == buf_.end()) {
      buf_.clear();
      return false;
    }

    size_t pos = static_cast<size_t>(std::distance(buf_.begin(), it));
    if (buf_.size() - pos < 4) {
      return false;
    }

    // 响应长度域 NUM = 1 + 数据字节数
    uint8_t    num_field = buf_[pos + 2];
    if (num_field < 2) {
      buf_.erase(buf_.begin(), buf_.begin() + pos + 1);
      continue;
    }
    size_t data_count = static_cast<size_t>(num_field) - 1;
    size_t frame_len  = 4 + data_count + 2; // 帧头(2) + NUM(1) + REG(1) + data + 校验(2)

    if (buf_.size() - pos < frame_len) {
      return false;
    }

    if (!validateChecksum(pos, data_count)) {
      if (checksum_fails_ == std::numeric_limits<size_t>::max())
        checksum_fails_ = 0;
      else
        ++checksum_fails_;
      buf_.erase(buf_.begin(), buf_.begin() + pos + 1);
      continue;
    }

    out            = RegisterFrame();
    out.valid      = true;
    out.start_reg  = buf_[pos + 3];
    out.values.assign(buf_.begin() + pos + 4, buf_.begin() + pos + 4 + data_count);

    buf_.erase(buf_.begin(), buf_.begin() + pos + frame_len);
    return true;
  }

  return false;
}

bool RegisterParser::validateChecksum(size_t pos, size_t data_count) const {
  // 累加帧头、NUM、REG 及全部 data 字节（共 4 + data_count 字节）
  size_t   sum_end = pos + 4 + data_count;
  uint32_t sum     = 0;
  for (size_t i = pos; i < sum_end; ++i) {
    sum += buf_[i];
  }

  return buf_[sum_end] == static_cast<uint8_t>(sum & 0xFF) &&
         buf_[sum_end + 1] == static_cast<uint8_t>((sum >> 8) & 0xFF);
}

std::array<uint8_t, 7> RegisterParser::BuildReadRequest(uint8_t reg, uint8_t num) {
  std::array<uint8_t, 7> req{0xFF, 0xAA, 0x27, reg, num, 0x00, 0x00};
  uint32_t               sum = static_cast<uint32_t>(req[0]) + req[1] + req[2] + reg + num;
  req[5]                     = static_cast<uint8_t>(sum & 0xFF);
  req[6]                     = static_cast<uint8_t>((sum >> 8) & 0xFF);
  return req;
}
