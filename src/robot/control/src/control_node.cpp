#include <chrono>
#include <functional>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger()))
{
  robot::ControlCore::Params params;
  params.lookahead_distance_m =
    this->declare_parameter<double>("lookahead_distance_m", params.lookahead_distance_m);
  params.linear_speed_mps =
    this->declare_parameter<double>("linear_speed_mps", params.linear_speed_mps);
  params.max_angular_speed =
    this->declare_parameter<double>("max_angular_speed", params.max_angular_speed);
  params.goal_tolerance_m =
    this->declare_parameter<double>("goal_tolerance_m", params.goal_tolerance_m);
  const double controller_rate_hz = this->declare_parameter<double>("controller_rate_hz", 15.0);

  if (params.lookahead_distance_m <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "lookahead_distance_m <= 0, using default 1.0");
    params.lookahead_distance_m = 1.0;
  }
  if (params.linear_speed_mps <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "linear_speed_mps <= 0, using default 0.9");
    params.linear_speed_mps = 0.9;
  }
  if (params.max_angular_speed <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "max_angular_speed <= 0, using default 2.0");
    params.max_angular_speed = 2.0;
  }

  control_.setParams(params);
  logParams(params);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  const double timer_period_s = (controller_rate_hz > 0.0) ? (1.0 / controller_rate_hz) : 0.1;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_s),
    std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  control_.setPath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  control_.setOdom(*msg);
}

void ControlNode::timerCallback()
{
  const auto cmd = control_.computeCommand();
  cmd_pub_->publish(cmd);
}

void ControlNode::logParams(const robot::ControlCore::Params& params) const
{
  RCLCPP_INFO(this->get_logger(),
    "Control params: lookahead=%.2f linear_speed=%.2f max_angular=%.2f goal_tol=%.2f",
    params.lookahead_distance_m, params.linear_speed_mps,
    params.max_angular_speed, params.goal_tolerance_m);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
