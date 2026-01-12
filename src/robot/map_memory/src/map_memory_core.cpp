#include "map_memory_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::setParams(const Params& params)
{
  params_ = params;
  params_set_ = true;
}

const MapMemoryCore::Params& MapMemoryCore::params() const
{
  return params_;
}

int MapMemoryCore::widthCells() const
{
  return static_cast<int>(std::ceil(params_.map_width_m / params_.map_resolution));
}

int MapMemoryCore::heightCells() const
{
  return static_cast<int>(std::ceil(params_.map_height_m / params_.map_resolution));
}

double MapMemoryCore::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void MapMemoryCore::initializeMap()
{
  if (!params_set_) {
    RCLCPP_WARN(logger_, "Map params not set; using defaults.");
    setParams(params_);
  }

  const int width = widthCells();
  const int height = heightCells();

  global_map_.header.frame_id = params_.map_frame_id;
  global_map_.info.resolution = params_.map_resolution;
  global_map_.info.width = static_cast<uint32_t>(width);
  global_map_.info.height = static_cast<uint32_t>(height);
  global_map_.info.origin.position.x = -0.5 * params_.map_width_m;
  global_map_.info.origin.position.y = -0.5 * params_.map_height_m;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(width * height, -1);
  map_initialized_ = true;
}

void MapMemoryCore::setCostmap(const nav_msgs::msg::OccupancyGrid& costmap)
{
  latest_costmap_ = costmap;
  have_costmap_ = true;
}

void MapMemoryCore::setOdom(const nav_msgs::msg::Odometry& odom)
{
  current_pose_.x = odom.pose.pose.position.x;
  current_pose_.y = odom.pose.pose.position.y;
  current_pose_.yaw = yawFromQuaternion(odom.pose.pose.orientation);
  have_odom_ = true;
}

bool MapMemoryCore::canFuse() const
{
  if (!map_initialized_ || !have_costmap_ || !have_odom_) {
    return false;
  }
  if (!has_last_fusion_) {
    return true;
  }
  const double dx = current_pose_.x - last_fusion_pose_.x;
  const double dy = current_pose_.y - last_fusion_pose_.y;
  return std::hypot(dx, dy) >= params_.fusion_distance_threshold_m;
}

bool MapMemoryCore::fuseLatest()
{
  if (!canFuse()) {
    return false;
  }

  const int map_width = static_cast<int>(global_map_.info.width);
  const int map_height = static_cast<int>(global_map_.info.height);
  if (map_width <= 0 || map_height <= 0 || global_map_.data.empty()) {
    return false;
  }

  const int costmap_width = static_cast<int>(latest_costmap_.info.width);
  const int costmap_height = static_cast<int>(latest_costmap_.info.height);
  if (costmap_width <= 0 || costmap_height <= 0 || latest_costmap_.data.empty()) {
    return false;
  }

  const double map_origin_x = global_map_.info.origin.position.x;
  const double map_origin_y = global_map_.info.origin.position.y;

  const double costmap_origin_x = latest_costmap_.info.origin.position.x;
  const double costmap_origin_y = latest_costmap_.info.origin.position.y;
  const double costmap_res = latest_costmap_.info.resolution;

  const double cos_yaw = std::cos(current_pose_.yaw);
  const double sin_yaw = std::sin(current_pose_.yaw);

  for (int cy = 0; cy < costmap_height; ++cy) {
    for (int cx = 0; cx < costmap_width; ++cx) {
      const int cidx = cy * costmap_width + cx;
      const int8_t cell_value = latest_costmap_.data[cidx];
      if (cell_value < 0) {
        continue;
      }

      const double local_x = costmap_origin_x + (cx + 0.5) * costmap_res;
      const double local_y = costmap_origin_y + (cy + 0.5) * costmap_res;
      const double world_x = current_pose_.x + (cos_yaw * local_x - sin_yaw * local_y);
      const double world_y = current_pose_.y + (sin_yaw * local_x + cos_yaw * local_y);

      const int gx = static_cast<int>(std::floor((world_x - map_origin_x) / params_.map_resolution));
      const int gy = static_cast<int>(std::floor((world_y - map_origin_y) / params_.map_resolution));
      if (gx < 0 || gy < 0 || gx >= map_width || gy >= map_height) {
        continue;
      }
      const int gidx = gy * map_width + gx;
      global_map_.data[gidx] = std::clamp(cell_value, static_cast<int8_t>(0), static_cast<int8_t>(100));
    }
  }

  last_fusion_pose_ = current_pose_;
  has_last_fusion_ = true;
  return true;
}

const nav_msgs::msg::OccupancyGrid& MapMemoryCore::map() const
{
  return global_map_;
}

} 
