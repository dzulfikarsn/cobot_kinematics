#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class Show : public rclcpp::Node
{
public:
  Show();

private:
  void startExecution();
  void callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pubTrajectoryServ_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr anglePub_;
  rclcpp::TimerBase::SharedPtr timer_;

  trajectory_msgs::msg::JointTrajectory msg;
};

Show::Show()
: Node("trajectory_node")
{
  using namespace std::chrono_literals;

  pubTrajectoryServ_ = this->create_service<std_srvs::srv::Trigger>(
    "pub_angle_show", std::bind(&Show::callback, this, std::placeholders::_1, std::placeholders::_2));
  anglePub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 1);

  timer_ = this->create_wall_timer(
    1s, std::bind(&Show::startExecution, this));
  
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.0, 0.0 + 0.1667, 0.0, 0.0, 0.0, 0.0};
  point.time_from_start.sec = 0;
  msg.points.push_back(point);

  point.positions = {M_PI_4, -0.75455 + 0.1667, 0.89365, 0.00030, -1.49336, 0.0};
  point.time_from_start.sec = 2;
  msg.points.push_back(point);

  point.positions = {M_PI_4, -0.75455 + 0.1667, 0.89365, 0.00030, -1.49336, 0.0};
  point.time_from_start.sec = 4;
  msg.points.push_back(point);

  point.positions = {M_PI_4, -1.03115 + 0.1667, 1.13626, 0.00042, -0.97414, 0.0};
  point.time_from_start.sec = 6;
  msg.points.push_back(point);

  point.positions = {M_PI_4, -1.03115 + 0.1667, 1.13626, 0.00042, -0.97414, 0.0};
  point.time_from_start.sec = 8;
  msg.points.push_back(point);

  point.positions = {M_PI_4, -1.03115 + 0.1667, 1.13626, 0.00042, -0.97414, -1.0};
  point.time_from_start.sec = 10;
  msg.points.push_back(point);

  point.positions = {M_PI_4, -0.75455 + 0.1667, 0.89365, 0.00030, -1.49336, -1.0};
  point.time_from_start.sec = 12;
  msg.points.push_back(point);

  point.positions = {M_PI_4, -0.75455 + 0.1667, 0.89365, 0.00030, -1.49336, -1.0};
  point.time_from_start.sec = 14;
  msg.points.push_back(point);

  point.positions = {0.0, -0.25859 + 0.1667, 1.55261, -0.00000, -1.34861, -1.0};
  point.time_from_start.sec = 16;
  msg.points.push_back(point);

  point.positions = {0.0, -0.25859 + 0.1667, 1.55261, -0.00000, -1.34861, -1.0};
  point.time_from_start.sec = 18;
  msg.points.push_back(point);

  point.positions = {0.0, -0.25859 + 0.1667, 1.55261, -0.00000, -1.34861, 0.0};
  point.time_from_start.sec = 20;
  msg.points.push_back(point);

  point.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  point.time_from_start.sec = 13;

  msg.joint_names = {"joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "gripper"};
}

void Show::startExecution() {
  timer_->cancel();
}

void Show::callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
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