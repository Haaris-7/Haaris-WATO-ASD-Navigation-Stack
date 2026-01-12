#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    struct Params {
      double resolution_m{0.1};
      double width_m{40.0};
      double height_m{40.0};
      double inflation_radius_m{1.2};
      int inflation_max_cost{100};
      std::string frame_id{"robot/chassis/lidar"};
    };

    void setParams(const Params& params);
    const Params& params() const;

    nav_msgs::msg::OccupancyGrid buildCostmap(const sensor_msgs::msg::LaserScan& scan);

  private:
    rclcpp::Logger logger_;
    Params params_;
    bool params_set_{false};

    struct InflationOffset {
      int dx;
      int dy;
      int cost;
    };
    std::vector<InflationOffset> inflation_offsets_;

    void rebuildInflationCache();
    bool worldToGrid(double x, double y, int& ix, int& iy) const;
    int widthCells() const;
    int heightCells() const;

};

}  

#endif  
