#include "imu_publisher.h"

ImuPublisher::ImuPublisher(rclcpp::Node* node, bool publish_custom, bool publish_sensor_msgs,
                           bool publish_temperature, bool publish_attitude, bool euler_unit_deg,
                           const std::string& frame_id, std::shared_ptr<imu_algorithm::ImpvqfEstimator> ahrs)
    : logger_(node->get_logger()), publish_custom_(publish_custom), publish_sensor_msgs_(publish_sensor_msgs),
      publish_temperature_(publish_temperature), publish_attitude_(publish_attitude), euler_unit_deg_(euler_unit_deg),
      frame_id_(frame_id), ahrs_ptr_(ahrs) {

  if (publish_custom_) {
    pub_custom_ = node->create_publisher<imu_driver_interfaces::msg::ImuData>("/DA901S/imu/data_raw", 10);
    RCLCPP_INFO(logger_, "Publishing custom ImuData on /DA901S/imu/data_raw");
  }

  if (publish_sensor_msgs) {
    pub_imu_ = node->create_publisher<sensor_msgs::msg::Imu>("/DA901S/imu/data", 10);
    pub_mag_ = node->create_publisher<sensor_msgs::msg::MagneticField>("/DA901S/imu/mag", 10);
    RCLCPP_INFO(logger_, "Publishing sensor_msgs/Imu on /DA901S/imu/data, "
                         "sensor_msgs/MagneticField on /DA901S/imu/mag");
  }

  if (publish_temperature_) {
    pub_temp_ = node->create_publisher<std_msgs::msg::Float32>("/DA901S/imu/temperature", 10);
    RCLCPP_INFO(logger_, "Publishing temperature (deg C) on /DA901S/imu/temperature");
  }

  if (publish_attitude_) {
    pub_attitude_ = node->create_publisher<imu_driver_interfaces::msg::ImuAttitude>("/DA901S/imu/attitude", 10);
    pub_attitude_ahrs_ =
        node->create_publisher<imu_driver_interfaces::msg::ImuAttitude>("/DA901S/imu/attitude_ahrs", 10);
    RCLCPP_INFO(logger_, "Publishing device attitude on /DA901S/imu/attitude and "
                         "SDK AHRS attitude on /DA901S/imu/attitude_ahrs (euler in %s)",
                euler_unit_deg_ ? "deg" : "rad");
  }

  if (ahrs_ptr_ && ahrs_ptr_->Initialized()) {
    RCLCPP_INFO(logger_, "ImpVQF AHRS initialized: version=%s, storage=%zu bytes", impvqf_version(),
                impvqf_context_size());
  }
}

void ImuPublisher::Publish(const ImuRawData& raw, const rclcpp::Time& stamp) {
  // 1. 内部量 + SDK AHRS 更新
  geometry_msgs::msg::Vector3    accel;
  geometry_msgs::msg::Vector3    gyro;
  geometry_msgs::msg::Vector3    mag;
  geometry_msgs::msg::Quaternion ahrs_quat;
  convertRawData(raw, accel, gyro, mag, ahrs_quat);

  // 2. 设备寄存器姿态（与 SDK 姿态分开发布）
  geometry_msgs::msg::Quaternion device_quat;
  deviceAttitude(raw, device_quat);

  // 3. 按需求转换发布单位：角速度 °/s、磁场 mGauss
  geometry_msgs::msg::Vector3 angular_velocity_out;
  angular_velocity_out.x = gyro.x * Rad2Deg;
  angular_velocity_out.y = gyro.y * Rad2Deg;
  angular_velocity_out.z = gyro.z * Rad2Deg;

  // 4. 发布自定义消息和标准消息（朝向统一使用内部 SDK AHRS）
  if (publish_custom_) {
    auto custom_msg            = imu_driver_interfaces::msg::ImuData();
    custom_msg.header.stamp    = stamp;
    custom_msg.header.frame_id = frame_id_;

    custom_msg.orientation         = ahrs_quat;
    custom_msg.linear_acceleration = accel;
    custom_msg.angular_velocity    = angular_velocity_out;
    custom_msg.magnetic_field      = mag;

    custom_msg.valid = raw.valid;
    pub_custom_->publish(custom_msg);
  }

  if (publish_sensor_msgs_) {
    auto imu_msg            = sensor_msgs::msg::Imu();
    imu_msg.header.stamp    = stamp;
    imu_msg.header.frame_id = frame_id_;

    imu_msg.orientation = ahrs_quat;
    fillCovariance(imu_msg.orientation_covariance, !ahrs_ptr_ || !ahrs_ptr_->Initialized());

    imu_msg.linear_acceleration = accel;
    fillCovariance(imu_msg.linear_acceleration_covariance, false);

    imu_msg.angular_velocity = angular_velocity_out;
    fillCovariance(imu_msg.angular_velocity_covariance, false);

    auto mag_msg            = sensor_msgs::msg::MagneticField();
    mag_msg.header.stamp    = stamp;
    mag_msg.header.frame_id = frame_id_;
    mag_msg.magnetic_field  = mag;
    fillCovariance(mag_msg.magnetic_field_covariance, false);

    pub_imu_->publish(imu_msg);
    pub_mag_->publish(mag_msg);
  }

  // 5. 两个姿态话题：设备读取姿态 & 内部 SDK AHRS 姿态
  if (publish_attitude_) {
    auto device_msg            = imu_driver_interfaces::msg::ImuAttitude();
    device_msg.header.stamp    = stamp;
    device_msg.header.frame_id = frame_id_;
    device_msg.orientation     = device_quat;
    quaternionToEuler(device_quat, device_msg.euler);
    pub_attitude_->publish(device_msg);

    auto ahrs_msg            = imu_driver_interfaces::msg::ImuAttitude();
    ahrs_msg.header.stamp    = stamp;
    ahrs_msg.header.frame_id = frame_id_;
    ahrs_msg.orientation     = ahrs_quat;
    quaternionToEuler(ahrs_quat, ahrs_msg.euler);
    pub_attitude_ahrs_->publish(ahrs_msg);
  }

  // 6. 板载温度（°C）
  if (publish_temperature_ && raw.has_dev_temp) {
    auto temp_msg = std_msgs::msg::Float32();
    temp_msg.data = static_cast<float>(raw.dev_temp) / 100.0f;
    pub_temp_->publish(temp_msg);
  }
}

void ImuPublisher::fillCovariance(std::array<double, 9>& cov, bool unknown) {
  // 全部置零
  cov.fill(COV_ZERO);

  if (unknown) {
    cov[0] = COV_UNKNOWN;
  }
}

void ImuPublisher::convertRawData(const ImuRawData& raw, geometry_msgs::msg::Vector3& accel,
                                  geometry_msgs::msg::Vector3& gyro, geometry_msgs::msg::Vector3& mag,
                                  geometry_msgs::msg::Quaternion& ahrs_quat) {
  // 1. 线性加速度：raw / 32768 * 16 g -> m/s^2
  if (raw.has_accel) {
    accel.x = static_cast<double>(raw.ax) * ACCEL_SCALE;
    accel.y = static_cast<double>(raw.ay) * ACCEL_SCALE;
    accel.z = static_cast<double>(raw.az) * ACCEL_SCALE;
  } else {
    accel.x = accel.y = accel.z = 0.0;
  }

  // 2. 角速度：raw / 32768 * 2000 °/s -> rad/s（SDK 输入）
  if (raw.has_gyro) {
    gyro.x = static_cast<double>(raw.wx) * GYRO_SCALE_RAD;
    gyro.y = static_cast<double>(raw.wy) * GYRO_SCALE_RAD;
    gyro.z = static_cast<double>(raw.wz) * GYRO_SCALE_RAD;
  } else {
    gyro.x = gyro.y = gyro.z = 0.0;
  }

  // 3. 磁场：raw * 1 mGauss/LSB（SDK 接受同尺度磁场）
  if (raw.has_mag) {
    mag.x = static_cast<double>(raw.hx) * MAG_SCALE_MGAUSS;
    mag.y = static_cast<double>(raw.hy) * MAG_SCALE_MGAUSS;
    mag.z = static_cast<double>(raw.hz) * MAG_SCALE_MGAUSS;
  } else {
    mag.x = mag.y = mag.z = 0.0;
  }

  // 4. 用 SDK 更新 AHRS，取 quat9d 作为内部解算姿态
  ahrs_quat.x = ahrs_quat.y = ahrs_quat.z = 0.0;
  ahrs_quat.w                             = 1.0;
  if (ahrs_ptr_ && ahrs_ptr_->Initialized() && raw.has_accel && raw.has_gyro && raw.has_mag) {
    Eigen::Vector3d gyr_vec(gyro.x, gyro.y, gyro.z);
    Eigen::Vector3d acc_vec(accel.x, accel.y, accel.z);
    Eigen::Vector3d mag_vec(mag.x, mag.y, mag.z);
    if (ahrs_ptr_->Update(gyr_vec, acc_vec, mag_vec)) {
      const Eigen::Quaterniond q = ahrs_ptr_->Quaternion9D();
      ahrs_quat.x                = q.x();
      ahrs_quat.y                = q.y();
      ahrs_quat.z                = q.z();
      ahrs_quat.w                = q.w();
    } else {
      RCLCPP_WARN_THROTTLE(logger_, clock_, 5000, "ImpVQF update failed: status=%d", ahrs_ptr_->LastStatus());
    }
  }
}

void ImuPublisher::deviceAttitude(const ImuRawData& raw, geometry_msgs::msg::Quaternion& orientation) const {
  orientation.x = orientation.y = orientation.z = 0.0;
  orientation.w                                 = 1.0;

  if (raw.has_dev_quat) {
    // 手册 4.5：q = raw / 32768，q0 为标量(w)
    orientation.w = static_cast<double>(raw.dev_q0) / 32768.0;
    orientation.x = static_cast<double>(raw.dev_q1) / 32768.0;
    orientation.y = static_cast<double>(raw.dev_q2) / 32768.0;
    orientation.z = static_cast<double>(raw.dev_q3) / 32768.0;
  } else if (raw.has_dev_euler) {
    // 手册 4.4：角度 = raw / 32768 * 180°
    double            roll  = static_cast<double>(raw.dev_roll) / 32768.0 * M_PI;
    double            pitch = static_cast<double>(raw.dev_pitch) / 32768.0 * M_PI;
    double            yaw   = static_cast<double>(raw.dev_yaw) / 32768.0 * M_PI;
    Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = rz * ry * rx;
    orientation.x        = q.x();
    orientation.y        = q.y();
    orientation.z        = q.z();
    orientation.w        = q.w();
  }
}

void ImuPublisher::quaternionToEuler(const geometry_msgs::msg::Quaternion& q,
                                     geometry_msgs::msg::Vector3&          euler) const {
  // ZYX 顺序（yaw-pitch-roll），与姿态合成 rz * ry * rx 一致
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  quat.normalize();
  Eigen::Vector3d ypr = quat.toRotationMatrix().eulerAngles(2, 1, 0);

  const double scale = euler_unit_deg_ ? Rad2Deg : 1.0;
  euler.x            = ypr[2] * scale; // roll
  euler.y            = ypr[1] * scale; // pitch
  euler.z            = ypr[0] * scale; // yaw
}
