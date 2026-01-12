#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    struct Params {
      int occ_threshold{40};
      int connectivity{8};
      std::string heuristic_type{"euclidean"};
      std::string frame_id{"sim_world"};
      double cost_weight{12.0};
    };

    void setParams(const Params& params);
    const Params& params() const;

    nav_msgs::msg::Path planPath(
      const nav_msgs::msg::OccupancyGrid& map,
      const geometry_msgs::msg::Point& start,
      const geometry_msgs::msg::Point& goal) const;

  private:
    rclcpp::Logger logger_;
    Params params_;
    bool params_set_{false};

    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy, int& gx, int& gy) const;
    geometry_msgs::msg::Point gridToWorld(const nav_msgs::msg::OccupancyGrid& map, int gx, int gy) const;
    double heuristic(int x0, int y0, int x1, int y1, double resolution) const;
    bool isObstacle(int8_t value) const;
    double cellCost(int8_t value) const;
};

}  

#endif  
