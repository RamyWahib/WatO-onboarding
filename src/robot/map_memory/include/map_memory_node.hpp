#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    // functions
    void readCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void readOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void integrateCostmap(const nav_msgs::msg::Odometry::SharedPtr odom);
    void updateMapIfMoved();

    // variables
    bool first_pose_ = true;
    double last_update_x_ = 0.0;
    double last_update_y_ = 0.0;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latestCostmap;

    
  private:
    robot::MapMemoryCore map_memory_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};   

#endif 
