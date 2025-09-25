#include "cobot_kinematics/ik.hpp"

#include <array>

#include "rclcpp/rclcpp.hpp"
#include "tf2/time.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// interface packages
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "kdl/chain.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"

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
  if (!tree.getChain("base_link", "end_effector", chain)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to get KDL chain!");
    return;
  }

  if (chain.getNrOfJoints() == 0) {
    RCLCPP_FATAL(this->get_logger(), "KDL chain has 0 joint!");
    return;
  }

  ikSolver = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain);

  lastSolution = KDL::JntArray(chain.getNrOfJoints());

  callbackTimer_ = this->create_wall_timer(
    100ms, std::bind(&CobotIK::callback, this));

  pubAngleService_ = this->create_service<std_srvs::srv::Trigger>(
    "pub_angle", std::bind(&CobotIK::publishAngleService, this, std::placeholders::_1, std::placeholders::_2));
  anglePub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 1);
}

void CobotIK::callback() {
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tfBuffer_.lookupTransform("base_link", "end_effector", tf2::TimePointZero);

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

  RCLCPP_INFO(
    this->get_logger(),
    "Pos x: %f y: %f z: %f\nOrien x: %f y: %f z: %f w: %f",
    pos[0], pos[1], pos[2], orien[0], orien[1], orien[2], orien[3]);

  solveIK();
}

void CobotIK::solveIK() {
  KDL::Frame targetPose(
    KDL::Rotation::Quaternion(0.5, 0.5, -0.5, 0.5),
    KDL::Vector(0.218, 0.004, 0.1));
  // KDL::Frame targetPose;  // pose of the target (position and orientation)
  // targetPose.p = KDL::Vector(pos.at(0), pos.at(1), pos.at(2));
  // targetPose.M = KDL::Rotation::Quaternion(orien.at(0), orien.at(1), orien.at(2), orien.at(3));

  solution = KDL::JntArray(chain.getNrOfJoints());

  int result = ikSolver->CartToJnt(lastSolution, targetPose, solution);

  if (result >= 0) {
    RCLCPP_INFO(this->get_logger(), "IK solution found:");
    for (size_t i=0; i<solution.rows(); i++) {
      RCLCPP_INFO(this->get_logger(), " joint[%ld] = %.5f", i, solution(i));
    }
    lastSolution = solution;
  } else {
    RCLCPP_ERROR(this->get_logger(), "IK failed (code = %d)", result);
  }
}

void CobotIK::publishAngleService(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res)  // req = request, res = response
{
  // publisher sessionpoint.positions.push_back(solution(i));

  trajectory_msgs::msg::JointTrajectoryPoint point;
  for (size_t i=0; i<solution.rows(); i++) {
    if (i==1) {
      point.positions.push_back(solution(i) + 0.1667);  // TODO: INI OFFSET TOLONG BUATKAN YAML UNTUK OFFSET BIAR LURUS KE ATAS SEMUA
    } else {
      point.positions.push_back(solution(i));
    }
  }
  point.positions.push_back(-1.0);  // for gripper
  point.time_from_start.sec = 1;  // second

  trajectory_msgs::msg::JointTrajectory msg;

  msg.header.stamp = this->get_clock()->now();
  
  msg.joint_names = {"joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "gripper"};

  msg.points.push_back(point);


  anglePub_->publish(msg);

  // service session

  res->success = true;
  res->message = "Joint angle command published.";
}