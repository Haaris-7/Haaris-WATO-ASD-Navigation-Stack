#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    struct Params {
      double lookahead_distance_m{1.0};
      double linear_speed_mps{0.9};
      double max_angular_speed{2.0};
      double goal_tolerance_m{0.35};
    };

    void setParams(const Params& params);
    const Params& params() const;

    void setPath(const nav_msgs::msg::Path& path);
    void setOdom(const nav_msgs::msg::Odometry& odom);
    geometry_msgs::msg::Twist computeCommand();
  
  private:
    rclcpp::Logger logger_;
    Params params_;
    bool params_set_{false};

    std::vector<geometry_msgs::msg::Point> path_points_;
    nav_msgs::msg::Odometry odom_;
    bool have_path_{false};
    bool have_odom_{false};

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
    static double normalizeAngle(double angle);
};

} 

#endif 
