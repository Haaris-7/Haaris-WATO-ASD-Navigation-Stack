#include <functional>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
  : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  robot::CostmapCore::Params params;
  params.resolution_m = this->declare_parameter<double>("costmap_resolution", params.resolution_m);
  params.width_m = this->declare_parameter<double>("costmap_width_m", params.width_m);
  params.height_m = this->declare_parameter<double>("costmap_height_m", params.height_m);
  params.inflation_radius_m = this->declare_parameter<double>("inflation_radius_m", params.inflation_radius_m);
  params.inflation_max_cost = this->declare_parameter<int>("inflation_max_cost", params.inflation_max_cost);
  params.frame_id = this->declare_parameter<std::string>("costmap_frame_id", params.frame_id);

  if (params.resolution_m <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "costmap_resolution <= 0, using default 0.1");
    params.resolution_m = 0.1;
  }
  if (params.width_m <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "costmap_width_m <= 0, using default 40.0");
    params.width_m = 40.0;
  }
  if (params.height_m <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "costmap_height_m <= 0, using default 40.0");
    params.height_m = 40.0;
  }
  if (params.inflation_radius_m < 0.0) {
    RCLCPP_WARN(this->get_logger(), "inflation_radius_m < 0, using default 1.2");
    params.inflation_radius_m = 1.2;
  }
  if (params.inflation_max_cost <= 0) {
    RCLCPP_WARN(this->get_logger(), "inflation_max_cost <= 0, using default 100");
    params.inflation_max_cost = 100;
  }

  costmap_.setParams(params);
  logParams(params);

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", rclcpp::SensorDataQoS(),
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const auto costmap = costmap_.buildCostmap(*msg);
  costmap_pub_->publish(costmap);
}

void CostmapNode::logParams(const robot::CostmapCore::Params& params) const
{
  RCLCPP_INFO(this->get_logger(),
    "Costmap params: resolution=%.3f width=%.2f height=%.2f inflation_radius=%.2f inflation_max_cost=%d frame_id=%s",
    params.resolution_m, params.width_m, params.height_m,
    params.inflation_radius_m, params.inflation_max_cost, params.frame_id.c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
