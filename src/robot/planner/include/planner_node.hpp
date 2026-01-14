#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <queue>

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
  struct CellIndex
    {
      int x;
      int y;
    
      CellIndex(int xx, int yy) : x(xx), y(yy) {}
      CellIndex() : x(0), y(0) {}
    
      bool operator==(const CellIndex &other) const
      {
        return (x == other.x && y == other.y);
      }
    
      bool operator!=(const CellIndex &other) const
      {
        return (x != other.x || y != other.y);
      }
    };
  struct CellIndexHash
      {
        std::size_t operator()(const CellIndex &idx) const
      {
        // A simple hash combining x and y
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
      }
    };
 
// Structure representing a node in the A* open set
  struct AStarNode
  {
    CellIndex index;
    double f_score;  // f = g + h
 
    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
  };

    PlannerNode();
    void readMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void readOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void readPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void planPath();
    void timerFunction();
    CellIndex worldToGrid(double x, double y);
    bool map_recieved;

    bool goal_reached = false;
    nav_msgs::msg::OccupancyGrid map;
    geometry_msgs::msg::PointStamped current_goal_;
    bool goal_received_ = false;
    double current_x_ = 0.0;
    double current_y_ = 0.0;

    double goal_x_ = 0.0;
    double goal_y_ = 0.0;

    // Comparator for the priority queue (min-heap by f_score)
    struct CompareF
      {
      bool operator()(const AStarNode &a, const AStarNode &b)
      {
    // We want the node with the smallest f_score on top
      return a.f_score > b.f_score;
      }
      };
    
    

  private:
    robot::PlannerCore planner_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

#endif 
