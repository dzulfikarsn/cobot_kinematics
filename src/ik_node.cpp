#include "cobot_kinematics/ik.hpp"

#include <memory>

#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CobotIK>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}