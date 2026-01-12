#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger()))
{
  robot::PlannerCore::Params params;
  params.occ_threshold = this->declare_parameter<int>("occ_threshold", params.occ_threshold);
  params.connectivity = this->declare_parameter<int>("connectivity", params.connectivity);
  params.heuristic_type = this->declare_parameter<std::string>("heuristic_type", params.heuristic_type);
  params.frame_id = this->declare_parameter<std::string>("map_frame_id", params.frame_id);
  params.cost_weight = this->declare_parameter<double>("cost_weight", params.cost_weight);
  planner_timer_ms_ = this->declare_parameter<int>("planner_timer_ms", planner_timer_ms_);
  goal_reached_threshold_m_ =
    this->declare_parameter<double>("goal_reached_threshold_m", goal_reached_threshold_m_);
  plan_timeout_s_ = this->declare_parameter<double>("plan_timeout_s", plan_timeout_s_);
  max_failed_plans_ = this->declare_parameter<int>("max_failed_plans", max_failed_plans_);
  map_frame_id_ = params.frame_id;

  if (params.connectivity != 4 && params.connectivity != 8) {
    RCLCPP_WARN(this->get_logger(), "connectivity must be 4 or 8, using 8");
    params.connectivity = 8;
  }
  if (params.heuristic_type != "euclidean" && params.heuristic_type != "manhattan") {
    RCLCPP_WARN(this->get_logger(), "heuristic_type must be euclidean or manhattan, using euclidean");
    params.heuristic_type = "euclidean";
  }
  if (planner_timer_ms_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "planner_timer_ms <= 0, using 500");
    planner_timer_ms_ = 500;
  }
  if (max_failed_plans_ < 0) {
    RCLCPP_WARN(this->get_logger(), "max_failed_plans < 0, using 0 (no auto-reset)");
    max_failed_plans_ = 0;
  }
  if (params.cost_weight < 0.0) {
    RCLCPP_WARN(this->get_logger(), "cost_weight < 0, using 0.0");
    params.cost_weight = 0.0;
  }

  planner_.setParams(params);
  logParams(params);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(planner_timer_ms_),
    std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = *msg;
  have_map_ = true;
  map_updated_ = true;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  have_goal_ = true;
  need_replan_ = true;
  failed_plan_count_ = 0;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  if (!goal_.header.frame_id.empty() && goal_.header.frame_id != map_frame_id_) {
    RCLCPP_WARN(this->get_logger(),
      "Goal frame_id '%s' does not match map frame '%s'.",
      goal_.header.frame_id.c_str(), map_frame_id_.c_str());
  }
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_ = *msg;
  have_odom_ = true;
}

void PlannerNode::timerCallback()
{
  if (state_ == State::WAITING_FOR_GOAL) {
    return;
  }
  if (!have_map_ || !have_goal_ || !have_odom_) {
    return;
  }

  const double dx = goal_.point.x - odom_.pose.pose.position.x;
  const double dy = goal_.point.y - odom_.pose.pose.position.y;
  const double distance_to_goal = std::hypot(dx, dy);
  if (distance_to_goal <= goal_reached_threshold_m_) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = map_frame_id_;
    empty_path.header.stamp = this->now();
    path_pub_->publish(empty_path);
    state_ = State::WAITING_FOR_GOAL;
    have_plan_ = false;
    RCLCPP_INFO(this->get_logger(), "Goal reached; waiting for next goal.");
    return;
  }

  const rclcpp::Time now = this->now();
  bool should_replan = need_replan_ || map_updated_ || !have_plan_;
  if (have_plan_ && plan_timeout_s_ > 0.0) {
    if ((now - last_plan_time_).seconds() >= plan_timeout_s_) {
      should_replan = true;
    }
  }

  if (!should_replan) {
    return;
  }

  robot::PlannerCore::Params params = planner_.params();
  params.frame_id = map_.header.frame_id.empty() ? map_frame_id_ : map_.header.frame_id;
  planner_.setParams(params);

  geometry_msgs::msg::Point start;
  start.x = odom_.pose.pose.position.x;
  start.y = odom_.pose.pose.position.y;
  start.z = 0.0;

  nav_msgs::msg::Path path = planner_.planPath(map_, start, goal_.point);
  path.header.stamp = now;
  path_pub_->publish(path);
  if (path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Planner failed to find a path.");
    failed_plan_count_++;
    have_plan_ = false;
    if (max_failed_plans_ > 0 && failed_plan_count_ >= max_failed_plans_) {
      RCLCPP_WARN(this->get_logger(), "Max failed plans reached; clearing goal and waiting.");
      have_goal_ = false;
      state_ = State::WAITING_FOR_GOAL;
    }
    last_plan_time_ = now;
    map_updated_ = false;
    need_replan_ = false;
    return;
  }

  last_plan_time_ = now;
  have_plan_ = true;
  failed_plan_count_ = 0;
  map_updated_ = false;
  need_replan_ = false;
}

void PlannerNode::logParams(const robot::PlannerCore::Params& params) const
{
  RCLCPP_INFO(this->get_logger(),
    "Planner params: occ_threshold=%d connectivity=%d heuristic=%s frame_id=%s timer_ms=%d goal_thresh=%.2f cost_weight=%.2f max_failed_plans=%d",
    params.occ_threshold, params.connectivity, params.heuristic_type.c_str(),
    params.frame_id.c_str(), planner_timer_ms_, goal_reached_threshold_m_,
    params.cost_weight, max_failed_plans_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
