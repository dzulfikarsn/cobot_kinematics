#ifndef COBOT_KINEMATICS__IK_HPP_
#define COBOT_KINEMATICS__IK_HPP_

#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// interface packages
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

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
  void publishAngleService(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  // get transform from the Robot Model
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  rclcpp::TimerBase::SharedPtr callbackTimer_;  // for callback() function

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pubAngleService_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr anglePub_;
  
  std::string urdfPath{"/home/jull/cobot_ws/src/cobot_description/urdf/cobot.urdf"};
  KDL::Chain chain;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ikSolver;
  KDL::JntArray lastSolution;  // last joint position
  KDL::JntArray solution;  // solution

  std::array<double, 3> pos{};
  std::array<double, 4> orien{};  // quaternion
};

#endif  // COBOT_KINEMATICS__IK_HPP_