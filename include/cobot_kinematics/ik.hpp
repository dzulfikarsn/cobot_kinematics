#ifndef COBOT_KINEMATICS__IK_HPP_
#define COBOT_KINEMATICS__IK_HPP_

#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "kdl/chain.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/jntarray.hpp"

class CobotIK : public rclcpp::Node
{
public:
  CobotIK();

private:
  void callback();
  void solveIK();

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::string urdfPath{"/home/jull/ros2_ws/src/cobot_description/urdf/cobot.urdf"};
  KDL::Chain chain;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ikSolver;
  KDL::JntArray lastSolution;

  std::array<double, 3> pos{};
  std::array<double, 4> orien{};  // quaternion
};

#endif  // COBOT_KINEMATICS__IK_HPP_