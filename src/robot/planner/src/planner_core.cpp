#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void PlannerCore::setParams(const Params& params)
{
  params_ = params;
  params_set_ = true;
}

const PlannerCore::Params& PlannerCore::params() const
{
  return params_;
}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy, int& gx, int& gy) const
{
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  gx = static_cast<int>(std::floor((wx - origin_x) / map.info.resolution));
  gy = static_cast<int>(std::floor((wy - origin_y) / map.info.resolution));
  return gx >= 0 && gy >= 0 &&
    gx < static_cast<int>(map.info.width) &&
    gy < static_cast<int>(map.info.height);
}

geometry_msgs::msg::Point PlannerCore::gridToWorld(
  const nav_msgs::msg::OccupancyGrid& map, int gx, int gy) const
{
  geometry_msgs::msg::Point p;
  p.x = map.info.origin.position.x + (gx + 0.5) * map.info.resolution;
  p.y = map.info.origin.position.y + (gy + 0.5) * map.info.resolution;
  p.z = 0.0;
  return p;
}

double PlannerCore::heuristic(int x0, int y0, int x1, int y1, double resolution) const
{
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  if (params_.heuristic_type == "manhattan") {
    return (dx + dy) * resolution;
  }
  return std::hypot(dx, dy) * resolution;
}

bool PlannerCore::isObstacle(int8_t value) const
{
  return value >= params_.occ_threshold;
}

double PlannerCore::cellCost(int8_t value) const
{
  if (value < 0) {
    return 0.0;
  }
  const double normalized = static_cast<double>(value) / 100.0;
  return normalized * params_.cost_weight;
}

nav_msgs::msg::Path PlannerCore::planPath(
  const nav_msgs::msg::OccupancyGrid& map,
  const geometry_msgs::msg::Point& start,
  const geometry_msgs::msg::Point& goal) const
{
  nav_msgs::msg::Path path;
  if (!params_set_) {
    RCLCPP_WARN(logger_, "Planner params not set; using defaults.");
  }

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  if (width <= 0 || height <= 0 || map.data.size() < static_cast<size_t>(width * height)) {
    RCLCPP_WARN(logger_, "Planner received invalid map.");
    return path;
  }

  int start_x = 0;
  int start_y = 0;
  int goal_x = 0;
  int goal_y = 0;
  if (!worldToGrid(map, start.x, start.y, start_x, start_y) ||
      !worldToGrid(map, goal.x, goal.y, goal_x, goal_y)) {
    RCLCPP_WARN(logger_, "Start or goal outside map bounds.");
    return path;
  }

  const int start_idx = start_y * width + start_x;
  const int goal_idx = goal_y * width + goal_x;

  if (isObstacle(map.data[start_idx]) || isObstacle(map.data[goal_idx])) {
    RCLCPP_WARN(logger_, "Start or goal in obstacle cell.");
    return path;
  }

  struct QueueNode {
    int idx;
    double f;
  };
  struct CompareNode {
    bool operator()(const QueueNode& a, const QueueNode& b) const
    {
      return a.f > b.f;
    }
  };

  std::priority_queue<QueueNode, std::vector<QueueNode>, CompareNode> open_set;
  const int total_cells = width * height;
  std::vector<double> g_score(total_cells, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(total_cells, -1);
  std::vector<bool> closed(total_cells, false);

  g_score[start_idx] = 0.0;
  open_set.push({start_idx, heuristic(start_x, start_y, goal_x, goal_y, map.info.resolution)});

  const std::vector<std::pair<int, int>> neighbors_4 = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}
  };
  const std::vector<std::pair<int, int>> neighbors_8 = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };
  const auto& neighbors = (params_.connectivity == 4) ? neighbors_4 : neighbors_8;

  bool found = false;
  while (!open_set.empty()) {
    const auto current = open_set.top();
    open_set.pop();
    if (closed[current.idx]) {
      continue;
    }
    closed[current.idx] = true;
    if (current.idx == goal_idx) {
      found = true;
      break;
    }

    const int cx = current.idx % width;
    const int cy = current.idx / width;

    for (const auto& step : neighbors) {
      const int nx = cx + step.first;
      const int ny = cy + step.second;
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      const int nidx = ny * width + nx;
      if (closed[nidx]) {
        continue;
      }
      const int8_t cell_value = map.data[nidx];
      if (isObstacle(cell_value)) {
        continue;
      }

      const double step_cost = std::hypot(step.first, step.second) * map.info.resolution;
      const double tentative_g = g_score[current.idx] + step_cost + cellCost(cell_value);
      if (tentative_g < g_score[nidx]) {
        came_from[nidx] = current.idx;
        g_score[nidx] = tentative_g;
        const double f = tentative_g + heuristic(nx, ny, goal_x, goal_y, map.info.resolution);
        open_set.push({nidx, f});
      }
    }
  }

  if (!found) {
    return path;
  }

  std::vector<int> indices;
  int current = goal_idx;
  while (current != -1) {
    indices.push_back(current);
    if (current == start_idx) {
      break;
    }
    current = came_from[current];
  }
  if (indices.empty() || indices.back() != start_idx) {
    return path;
  }
  std::reverse(indices.begin(), indices.end());

  path.header.frame_id = params_.frame_id.empty() ? map.header.frame_id : params_.frame_id;
  path.poses.reserve(indices.size());
  for (const int idx : indices) {
    const int gx = idx % width;
    const int gy = idx / width;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.pose.position = gridToWorld(map, gx, gy);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

} 
