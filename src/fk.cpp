#include "cobot_kinematics/fk.hpp"

#include "kdl_parser/kdl_parser.hpp"

CobotFK::CobotFK()
: Node("cobot_fk")
{
  // Parse URDF to KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdfPath, tree)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to parse URDF into KDL Tree!");
    return;
  }

  // Get the tree chain
  if (!tree.getChain("base_link", "link4", chain)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain!");
    return;
  }

  fkSolver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);

  // ROS2 System
  jointSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&CobotFK::jointCallback, this, std::placeholders::_1));
}

void CobotFK::jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() < chain.getNrOfJoints()) {
    RCLCPP_WARN(this->get_logger(),
      "JointState doesn't have enough joints: got %ld, expected %d",
      msg->position.size(), chain.getNrOfJoints());
    
    return;
  }

  KDL::JntArray jointPositions(chain.getNrOfJoints());
  for (size_t i = 0; i < jointPositions.rows(); ++i) {
    jointPositions(i) = msg->position[i];
  }

  KDL::Frame eeFrame;
  if (fkSolver->JntToCart(jointPositions, eeFrame) >= 0) {
    double x = eeFrame.p.x();
    double y = eeFrame.p.y();
    double z = eeFrame.p.z();
    RCLCPP_INFO(this->get_logger(), "End-effector: [%.3f, %.3f, %.3f]", x, y, z);

    double roll, pitch, yaw;
    eeFrame.M.GetRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "Orientation: [%.3f, %.3f, %.3f]", roll, pitch, yaw);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to compute FK");
  }
}