#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::setParams(const Params& params)
{
  params_ = params;
  params_set_ = true;
  rebuildInflationCache();
}

const CostmapCore::Params& CostmapCore::params() const
{
  return params_;
}

int CostmapCore::widthCells() const
{
  return static_cast<int>(std::ceil(params_.width_m / params_.resolution_m));
}

int CostmapCore::heightCells() const
{
  return static_cast<int>(std::ceil(params_.height_m / params_.resolution_m));
}

bool CostmapCore::worldToGrid(double x, double y, int& ix, int& iy) const
{
  const double origin_x = -0.5 * params_.width_m;
  const double origin_y = -0.5 * params_.height_m;
  ix = static_cast<int>(std::floor((x - origin_x) / params_.resolution_m));
  iy = static_cast<int>(std::floor((y - origin_y) / params_.resolution_m));
  return ix >= 0 && iy >= 0 && ix < widthCells() && iy < heightCells();
}

void CostmapCore::rebuildInflationCache()
{
  inflation_offsets_.clear();
  if (params_.inflation_radius_m <= 0.0 || params_.resolution_m <= 0.0) {
    return;
  }

  const int radius_cells = static_cast<int>(std::ceil(params_.inflation_radius_m / params_.resolution_m));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double dist = std::hypot(dx * params_.resolution_m, dy * params_.resolution_m);
      if (dist <= params_.inflation_radius_m) {
        const double ratio = 1.0 - (dist / params_.inflation_radius_m);
        const int cost = static_cast<int>(std::round(params_.inflation_max_cost * ratio));
        inflation_offsets_.push_back({dx, dy, std::clamp(cost, 0, params_.inflation_max_cost)});
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::buildCostmap(const sensor_msgs::msg::LaserScan& scan)
{
  nav_msgs::msg::OccupancyGrid costmap;
  if (!params_set_) {
    RCLCPP_WARN(logger_, "Costmap params not set; using defaults.");
    setParams(params_);
  }

  const int width = widthCells();
  const int height = heightCells();

  costmap.header = scan.header;
  costmap.header.frame_id = params_.frame_id;
  costmap.info.resolution = params_.resolution_m;
  costmap.info.width = static_cast<uint32_t>(width);
  costmap.info.height = static_cast<uint32_t>(height);
  costmap.info.origin.position.x = -0.5 * params_.width_m;
  costmap.info.origin.position.y = -0.5 * params_.height_m;
  costmap.info.origin.position.z = 0.0;
  costmap.info.origin.orientation.w = 1.0;
  costmap.data.assign(width * height, 0);

  std::vector<int> obstacle_indices;
  obstacle_indices.reserve(scan.ranges.size());

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range) || range <= scan.range_min || range >= scan.range_max) {
      continue;
    }

    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);

    int ix = 0;
    int iy = 0;
    if (!worldToGrid(x, y, ix, iy)) {
      continue;
    }
    const int idx = iy * width + ix;
    costmap.data[idx] = static_cast<int8_t>(params_.inflation_max_cost);
    obstacle_indices.push_back(idx);
  }

  if (inflation_offsets_.empty()) {
    return costmap;
  }

  for (const int obs_idx : obstacle_indices) {
    const int obs_x = obs_idx % width;
    const int obs_y = obs_idx / width;
    for (const auto& offset : inflation_offsets_) {
      const int nx = obs_x + offset.dx;
      const int ny = obs_y + offset.dy;
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      const int nidx = ny * width + nx;
      if (offset.cost > costmap.data[nidx]) {
        costmap.data[nidx] = static_cast<int8_t>(offset.cost);
      }
    }
  }

  return costmap;
}

}
