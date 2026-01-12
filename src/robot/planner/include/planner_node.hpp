#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State {
      WAITING_FOR_GOAL,
      WAITING_FOR_ROBOT_TO_REACH_GOAL
    };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void logParams(const robot::PlannerCore::Params& params) const;

    robot::PlannerCore planner_;
    State state_{State::WAITING_FOR_GOAL};

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid map_;
    geometry_msgs::msg::PointStamped goal_;
    nav_msgs::msg::Odometry odom_;
    bool have_map_{false};
    bool have_goal_{false};
    bool have_odom_{false};
    bool map_updated_{false};
    bool need_replan_{false};
    bool have_plan_{false};
    rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};

    int planner_timer_ms_{200};
    double goal_reached_threshold_m_{0.4};
    double plan_timeout_s_{1.0};
    int max_failed_plans_{3};
    int failed_plan_count_{0};
    std::string map_frame_id_{"sim_world"};
};

#endif 
