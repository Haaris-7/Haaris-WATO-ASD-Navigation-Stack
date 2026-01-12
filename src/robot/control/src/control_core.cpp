#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void ControlCore::setParams(const Params& params)
{
  params_ = params;
  params_set_ = true;
}

const ControlCore::Params& ControlCore::params() const
{
  return params_;
}

void ControlCore::setPath(const nav_msgs::msg::Path& path)
{
  path_points_.clear();
  path_points_.reserve(path.poses.size());
  for (const auto& pose : path.poses) {
    path_points_.push_back(pose.pose.position);
  }
  have_path_ = !path_points_.empty();
}

void ControlCore::setOdom(const nav_msgs::msg::Odometry& odom)
{
  odom_ = odom;
  have_odom_ = true;
}

double ControlCore::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double ControlCore::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

geometry_msgs::msg::Twist ControlCore::computeCommand()
{
  geometry_msgs::msg::Twist cmd;
  if (!params_set_) {
    RCLCPP_WARN(logger_, "Control params not set; using defaults.");
  }
  if (!have_odom_ || !have_path_) {
    return cmd;
  }
  if (path_points_.empty()) {
    return cmd;
  }

  const double x = odom_.pose.pose.position.x;
  const double y = odom_.pose.pose.position.y;
  const double yaw = yawFromQuaternion(odom_.pose.pose.orientation);

  const auto& goal = path_points_.back();
  const double goal_dx = goal.x - x;
  const double goal_dy = goal.y - y;
  const double goal_dist = std::hypot(goal_dx, goal_dy);
  if (goal_dist <= params_.goal_tolerance_m) {
    path_points_.clear();
    have_path_ = false;
    return cmd;
  }

  int nearest_idx = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path_points_.size(); ++i) {
    const double dx = path_points_[i].x - x;
    const double dy = path_points_[i].y - y;
    const double dist = std::hypot(dx, dy);
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_idx = static_cast<int>(i);
    }
  }

  geometry_msgs::msg::Point target = path_points_.back();
  for (size_t i = static_cast<size_t>(nearest_idx); i < path_points_.size(); ++i) {
    const double dx = path_points_[i].x - x;
    const double dy = path_points_[i].y - y;
    const double dist = std::hypot(dx, dy);
    if (dist >= params_.lookahead_distance_m) {
      target = path_points_[i];
      break;
    }
  }

  const double angle_to_target = std::atan2(target.y - y, target.x - x);
  const double alpha = normalizeAngle(angle_to_target - yaw);
  if (params_.lookahead_distance_m <= 1e-3) {
    return cmd;
  }

  constexpr double kRotateInPlaceThresholdRad = 1.2;
  const double curvature = 2.0 * std::sin(alpha) / params_.lookahead_distance_m;
  if (std::abs(alpha) >= kRotateInPlaceThresholdRad) {
    cmd.linear.x = 0.0;
    cmd.angular.z = (alpha > 0.0 ? 1.0 : -1.0) * params_.max_angular_speed;
    return cmd;
  }

  cmd.linear.x = params_.linear_speed_mps;
  cmd.angular.z = curvature * cmd.linear.x;
  cmd.angular.z = std::clamp(cmd.angular.z, -params_.max_angular_speed, params_.max_angular_speed);

  return cmd;
}

}  
