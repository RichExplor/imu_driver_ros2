#include "algorithm/impvqf_estimator.h"
#include <cmath>

namespace imu_algorithm {
namespace {

bool finite3(const Eigen::Vector3d& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

Eigen::Quaterniond toQuat(const ImpVqfReal q[4]) {
  return Eigen::Quaterniond(static_cast<double>(q[0]), static_cast<double>(q[1]), static_cast<double>(q[2]),
                            static_cast<double>(q[3]));
}

} // namespace

bool ImpvqfEstimator::Init(const Config& config) {
  static_assert(sizeof(ImpVqfContext) == IMPVQF_CONTEXT_BYTES, "ImpVqfContext size mismatch with SDK header");

  if (!std::isfinite(config.sample_rate_hz) || config.sample_rate_hz <= 0 || !std::isfinite(config.tau_acc) ||
      config.tau_acc <= 0 || !std::isfinite(config.tau_mag) || config.tau_mag <= 0) {
    last_status_ = IMPVQF_ERROR_CONFIG;
    return false;
  }

  ImpVqfConfig cfg;
  last_status_ = impvqf_config_default(&cfg, sizeof(cfg));
  if (last_status_ != IMPVQF_OK) {
    return false;
  }

  const ImpVqfReal ts     = static_cast<ImpVqfReal>(1.0 / config.sample_rate_hz);
  cfg.gyr_ts              = ts;
  cfg.acc_ts              = ts;
  cfg.mag_ts              = ts;
  cfg.tau_acc             = static_cast<ImpVqfReal>(config.tau_acc);
  cfg.tau_mag             = static_cast<ImpVqfReal>(config.tau_mag);
  cfg.options             = config.options;

  last_status_ = impvqf_init(&context_, &cfg);
  initialized_ = (last_status_ == IMPVQF_OK);
  return initialized_;
}

bool ImpvqfEstimator::Update(const Eigen::Vector3d& gyr_rad_s, const Eigen::Vector3d& acc_m_s2,
                             const Eigen::Vector3d& mag_mgauss) {
  if (!initialized_) {
    last_status_ = IMPVQF_ERROR_STATE;
    return false;
  }
  if (!finite3(gyr_rad_s) || !finite3(acc_m_s2) || !finite3(mag_mgauss)) {
    last_status_ = IMPVQF_ERROR_NUMERIC;
    return false;
  }

  const ImpVqfReal gyr[3] = {static_cast<ImpVqfReal>(gyr_rad_s.x()), static_cast<ImpVqfReal>(gyr_rad_s.y()),
                             static_cast<ImpVqfReal>(gyr_rad_s.z())};
  const ImpVqfReal acc[3] = {static_cast<ImpVqfReal>(acc_m_s2.x()), static_cast<ImpVqfReal>(acc_m_s2.y()),
                             static_cast<ImpVqfReal>(acc_m_s2.z())};
  const ImpVqfReal mag[3] = {static_cast<ImpVqfReal>(mag_mgauss.x()), static_cast<ImpVqfReal>(mag_mgauss.y()),
                             static_cast<ImpVqfReal>(mag_mgauss.z())};

  last_status_ = impvqf_update(&context_, gyr, acc, mag);
  if (last_status_ != IMPVQF_OK) {
    return false;
  }

  last_status_ = impvqf_get_output(&context_, &output_, sizeof(output_));
  return last_status_ == IMPVQF_OK;
}

Eigen::Quaterniond ImpvqfEstimator::Quaternion9D() const {
  return initialized_ ? toQuat(output_.quat9d) : Eigen::Quaterniond::Identity();
}

Eigen::Quaterniond ImpvqfEstimator::Quaternion6D() const {
  return initialized_ ? toQuat(output_.quat6d) : Eigen::Quaterniond::Identity();
}

std::array<double, 3> ImpvqfEstimator::BiasRadS() const {
  std::array<double, 3> bias{0.0, 0.0, 0.0};
  if (initialized_) {
    for (size_t i = 0; i < 3; ++i) {
      bias[i] = static_cast<double>(output_.bias_rad_s[i]);
    }
  }
  return bias;
}

uint32_t ImpvqfEstimator::Flags() const {
  return initialized_ ? output_.flags : 0U;
}

} // namespace imu_algorithm
