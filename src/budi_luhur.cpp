#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "kdl/chain.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"

class Show : public rclcpp::Node
{
public:
  Show();

private:
  void callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pubTrajectoryServ_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr anglePub_;
};

Show::Show()
: Node("trajectory_node")
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

  KDL::Frame frame(
    KDL::Rotation::Quaternion(0.5, 0.5, -0.5, 0.5),
    KDL::Vector(0.218241, 0.003845, 0.115223));
  target.push_back(frame);

  KDL::Frame zframe(
    KDL::Rotation::Quaternion(0.5, 0.5, -0.5, 0.5),
    KDL::Vector(0.218241, 0.003845, 0.1));
  // trajectory_msgs::msg::JointTrajectoryPoint p0;
  // p0.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // p0.time_from_start.sec = 0;

  // trajectory_msgs::msg::JointTrajectoryPoint p1;
  // p1.positions = {0.0, 0.795, -0.776, 0.0, 0.534, 0.0};
  // p1.time_from_start.sec = 1;

  // trajectory_msgs::msg::JointTrajectoryPoint p12;
  // p12.positions = {0.0, 0.795, -0.776, 0.0, 0.534, 1.0};
  // p12.time_from_start.sec = 2;

  // trajectory_msgs::msg::JointTrajectoryPoint p2;
  // p2.positions = {0.0, 1.03604, -1.07859, 0.0, 0.98982, 0.0};
  // p2.time_from_start.sec = 4;

  // trajectory_msgs::msg::JointTrajectoryPoint p22;
  // p22.positions = {0.0, 1.03604, -1.07859, 0.0, 0.98982, 1.0};
  // p22.time_from_start.sec = 5;

  // trajectory_msgs::msg::JointTrajectoryPoint p3;
  // p3.positions = {1.57066, 0.795, -0.776, 0.0, 0.534, 0.0};
  // p3.time_from_start.sec = 7;

  // trajectory_msgs::msg::JointTrajectoryPoint p32;
  // p32.positions = {1.57066, 0.795, -0.776, 0.0, 0.534, 1.0};
  // p32.time_from_start.sec = 8;

  // trajectory_msgs::msg::JointTrajectoryPoint p4;
  // p4.positions = {1.57066, 1.03489, -1.07180, 0.0, 0.99790, 0.0};
  // p4.time_from_start.sec = 10;

  // trajectory_msgs::msg::JointTrajectoryPoint p42;
  // p42.positions = {1.57066, 1.03489, -1.07180, 0.0, 0.99790, 1.0};
  // p42.time_from_start.sec = 11;

  // trajectory_msgs::msg::JointTrajectoryPoint p5;
  // p5.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // p5.time_from_start.sec = 13;

  // msg.joint_names = {"joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "gripper"};
  // msg.points.push_back(p0);
  // msg.points.push_back(p1);
  // msg.points.push_back(p12);
  // msg.points.push_back(p2);
  // msg.points.push_back(p22);
  // msg.points.push_back(p3);
  // msg.points.push_back(p32);
  // msg.points.push_back(p4);
  // msg.points.push_back(p42);
  // msg.points.push_back(p5);

  pubTrajectoryServ_ = this->create_service<std_srvs::srv::Trigger>(
    "pub_angle_show", std::bind(&Show::callback, this, std::placeholders::_1, std::placeholders::_2));
  anglePub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 1);
}

void Show::callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  trajectory_msgs::msg::JointTrajectory msg;

  for (const auto& it : target) {
    solution = KDL::JntArray(chain.getNrOfJoints());
    int result = ikSolver->CartToJnt(lastSolution, it, solution);

    if (result >= 0) {
      RCLCPP_INFO(this->get_logger(), "IK solution found:");
      for (size_t i=0; i<solution.rows(); i++) {
        RCLCPP_INFO(this->get_logger(), " joint[%ld] = %.5f", i, solution(i));
      }
      lastSolution = solution;

      trajectory_msgs::msg::JointTrajectoryPoint point;
      for (size_t i=0; i<solution.rows(); i++) {
        if (i==1) {
          point.positions.push_back(solution(i) + 0.1667);  // TODO: INI OFFSET TOLONG BUATKAN YAML UNTUK OFFSET BIAR LURUS KE ATAS SEMUA
        } else {
          point.positions.push_back(solution(i));
        }
      }
      point.positions.push_back(0.0);  // for gripper
      point.time_from_start.sec = 1;  // second
      
      msg.points.push_back(point);
    } else {
      RCLCPP_ERROR(this->get_logger(), "IK failed (code = %d)", result);
    }
  }

  msg.header.stamp = this->get_clock()->now();
  msg.joint_names = {"joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "gripper"};

  anglePub_->publish(msg);

  res->success = true;
  res->message = "Joint angle command published.";
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Show>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}