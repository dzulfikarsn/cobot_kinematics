#include "cobot_kinematics/ik.hpp"

#include <array>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

CobotIK::CobotIK()
: Node("cobot_ik"),
  tfBuffer_(this->get_clock()),
  tfListener_(tfBuffer_)
{
  // Parse URDF to KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdfPath, tree)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to parse URDF into KDL Tree!");
    return;
  }

  // Get the tree chain
  if (!tree.getChain("base_link", "link5", chain)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to get KDL chain!");
    return;
  }

  if (chain.getNrOfJoints() == 0) {
    RCLCPP_FATAL(this->get_logger(), "KDL chain has 0 joint!");
    return;
  }

  ikSolver = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain);
  lastSolution = KDL::JntArray(chain.getNrOfJoints());

  timer_ = this->create_wall_timer(
    100ms, std::bind(&CobotIK::callback, this));
}

void CobotIK::callback() {
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tfBuffer_.lookupTransform("base_link", "link5", tf2::TimePointZero);

    pos = { tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z };
    
    orien = { tf.transform.rotation.x,
              tf.transform.rotation.y,
              tf.transform.rotation.z,
              tf.transform.rotation.w };
    
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    return;
  }

  solveIK();
}

void CobotIK::solveIK() {
  KDL::Frame targetPose;
  targetPose.p = KDL::Vector(pos.at(0), pos.at(1), pos.at(2));
  targetPose.M = KDL::Rotation::Quaternion(orien.at(0), orien.at(1), orien.at(2), orien.at(3));

  KDL::JntArray q_init = lastSolution;
  KDL::JntArray q_result(chain.getNrOfJoints());

  int result = ikSolver->CartToJnt(q_init, targetPose, q_result);

  if (result >= 0) {
    RCLCPP_INFO(this->get_logger(), "IK solution found:");
    for (size_t i = 0; i < q_result.rows(); ++i) {
      RCLCPP_INFO(this->get_logger(), " joint[%ld] = %.5f", i, q_result(i));
    }
    lastSolution = q_result;
  } else {
    RCLCPP_ERROR(this->get_logger(), "IK failed (code = %d)", result);
  }
}