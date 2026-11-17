#pragma once

#include <array>
#include <cstdint>
#include <Eigen/Geometry>
#include "impvqf.h"

namespace imu_algorithm {

/// @brief ImpVQF SDK（静态库）的 C++ 封装，用作内部解算 AHRS
///
/// 输入：角速度 rad/s、加速度 m/s^2、磁场 mGauss（三轴同尺度即可）
/// 输出：quat9d / quat6d，均为机体系到地球系的 Hamilton 四元数 (w,x,y,z)
class ImpvqfEstimator {
public:
  struct Config {
    double   sample_rate_hz = 200.0; ///< 采样率，决定 gyr/acc/mag 周期
    double   tau_acc        = 3.0;   ///< SDK 默认倾角滤波时间常数 (s)
    double   tau_mag        = 9.0;   ///< SDK 默认磁航向修正时间常数 (s)
    uint32_t options        = 15;    ///< SDK 默认：运动/静止零偏 + 磁扰拒绝 + 梯形输入
  };

  bool Init(const Config& config);

  bool Update(const Eigen::Vector3d& gyr_rad_s, const Eigen::Vector3d& acc_m_s2,
              const Eigen::Vector3d& mag_mgauss);

  Eigen::Quaterniond Quaternion9D() const;
  Eigen::Quaterniond Quaternion6D() const;

  std::array<double, 3> BiasRadS() const;
  uint32_t              Flags() const;

  int32_t LastStatus() const {
    return last_status_;
  }
  bool Initialized() const {
    return initialized_;
  }

private:
  ImpVqfContext context_{};
  ImpVqfOutput  output_{};
  bool          initialized_ = false;
  int32_t       last_status_ = 0;
};

} // namespace imu_algorithm
