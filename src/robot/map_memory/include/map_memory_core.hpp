#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    struct Params {
      double map_resolution{0.15};
      double map_width_m{40.0};
      double map_height_m{40.0};
      std::string map_frame_id{"sim_world"};
      double fusion_distance_threshold_m{0.5};
    };

    void setParams(const Params& params);
    const Params& params() const;

    void setCostmap(const nav_msgs::msg::OccupancyGrid& costmap);
    void setOdom(const nav_msgs::msg::Odometry& odom);
    void initializeMap();
    bool canFuse() const;
    bool fuseLatest();
    const nav_msgs::msg::OccupancyGrid& map() const;

  private:
    rclcpp::Logger logger_;
    Params params_;
    bool params_set_{false};

    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool have_costmap_{false};
    bool have_odom_{false};
    bool map_initialized_{false};

    struct Pose2D {
      double x{0.0};
      double y{0.0};
      double yaw{0.0};
    };
    Pose2D current_pose_;
    Pose2D last_fusion_pose_;
    bool has_last_fusion_{false};

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
    int widthCells() const;
    int heightCells() const;
};

}  

#endif  
