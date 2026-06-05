#include "imu_driver_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ImuDriverNode>();

  if (!node->Init()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize IMU driver node");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "IMU driver node started");
  node->Run();
  node->Shutdown();

  rclcpp::shutdown();
  return 0;
}
