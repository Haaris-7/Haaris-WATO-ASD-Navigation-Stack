#include <chrono>
#include <functional>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
  : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  robot::MapMemoryCore::Params params;
  params.map_resolution = this->declare_parameter<double>("map_resolution", params.map_resolution);
  params.map_width_m = this->declare_parameter<double>("map_width_m", params.map_width_m);
  params.map_height_m = this->declare_parameter<double>("map_height_m", params.map_height_m);
  params.map_frame_id = this->declare_parameter<std::string>("map_frame_id", params.map_frame_id);
  params.fusion_distance_threshold_m =
    this->declare_parameter<double>("fusion_distance_threshold_m", params.fusion_distance_threshold_m);
  const double fusion_timer_hz = this->declare_parameter<double>("fusion_timer_hz", 1.0);

  if (params.map_resolution <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "map_resolution <= 0, using default 0.15");
    params.map_resolution = 0.15;
  }
  if (params.map_width_m <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "map_width_m <= 0, using default 40.0");
    params.map_width_m = 40.0;
  }
  if (params.map_height_m <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "map_height_m <= 0, using default 40.0");
    params.map_height_m = 40.0;
  }

  map_memory_.setParams(params);
  map_memory_.initializeMap();
  logParams(params);

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  map_pub_->publish(map_memory_.map());

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  const double timer_period_s = (fusion_timer_hz > 0.0) ? (1.0 / fusion_timer_hz) : 1.0;
  fusion_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_s),
    std::bind(&MapMemoryNode::fusionTimerCallback, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_memory_.setCostmap(*msg);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  map_memory_.setOdom(*msg);
}

void MapMemoryNode::fusionTimerCallback()
{
  if (map_memory_.fuseLatest()) {
    nav_msgs::msg::OccupancyGrid map = map_memory_.map();
    map.header.stamp = this->now();
    map_pub_->publish(map);
  }
}

void MapMemoryNode::logParams(const robot::MapMemoryCore::Params& params) const
{
  RCLCPP_INFO(this->get_logger(),
    "Map params: resolution=%.3f width=%.2f height=%.2f frame_id=%s fusion_dist=%.2f",
    params.map_resolution, params.map_width_m, params.map_height_m,
    params.map_frame_id.c_str(), params.fusion_distance_threshold_m);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
