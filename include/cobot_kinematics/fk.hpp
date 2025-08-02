#ifndef COBOT_KINEMATICS__FK_HPP_
#define COBOT_KINEMATICS__FK_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/frames.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"

class CobotFK : public rclcpp::Node
{
public:
  CobotFK();

private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSub_;

private:
  std::string urdfPath{"/home/jull/cobot_ws/src/cobot_description/urdf/cobot_kinematics.urdf"};
  KDL::Chain chain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fkSolver;
};

#endif  // COBOT_KINEMATICS__FK_HPP_