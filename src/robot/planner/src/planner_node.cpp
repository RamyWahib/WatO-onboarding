#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription <nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::readMap, this, std::placeholders::_1));
  odometry_sub_ = this->create_subscription <nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::readOdometry, this, std::placeholders::_1));
  point_sub_ = this->create_subscription <geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::readPoint, this, std::placeholders::_1));
  path_pub_ = this->create_publisher <nav_msgs::msg::Path>("/path", 10);
  update_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PlannerNode::timerFunction, this));
  map_recieved = false;
}

void PlannerNode::readMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  map = *msg;
  map_recieved = true;
}
void PlannerNode::readOdometry(const nav_msgs::msg::Odometry::SharedPtr msg){
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
}
void PlannerNode::readPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg){
  // DEBUG: show goal and map info
  goal_reached = false;
    RCLCPP_INFO(this->get_logger(), "Goal received: (%.2f, %.2f)", msg->point.x, msg->point.y);

    if (map.info.width == 0 || map.info.height == 0) {
        RCLCPP_WARN(this->get_logger(), "Map not received yet!");
        return;
    }

    auto goal_grid = worldToGrid(msg->point.x, msg->point.y);
    RCLCPP_INFO(this->get_logger(), "Goal grid: (%d, %d), map size: (%d, %d)", 
                goal_grid.x, goal_grid.y, map.info.width, map.info.height);

    // Ignore zero points
    if (msg->point.x == 0.0 && msg->point.y == 0.0) {
        return;
    }
  
  // Ignore zero points
    if (msg->point.x == 0.0 && msg->point.y == 0.0) {
        return;
    }

    // Store the current goal
    current_goal_ = *msg;
    goal_received_ = true;

    // Update goal coordinates for easier access
    goal_x_ = current_goal_.point.x;
    goal_y_ = current_goal_.point.y;

    RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", goal_x_, goal_y_);
}

void PlannerNode::timerFunction(){
  if(!goal_received_ || goal_reached || !map_recieved)
  {
    return;
  }
   // 3. Check if robot is close enough to the goal
    double dx = current_x_ - goal_x_;
    double dy = current_y_ - goal_y_;
    double distance = std::sqrt(dx * dx + dy * dy);

    constexpr double GOAL_TOLERANCE = 0.1; // meters

    if (distance < GOAL_TOLERANCE) {
        goal_reached = true;
        RCLCPP_INFO(this->get_logger(), "goal reached");
        return;
    }

    // 4. Otherwise, this is where planning happens
    planPath();
}

PlannerNode::CellIndex PlannerNode::worldToGrid(double x, double y)
{
  int gx = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
    int gy = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);

    // Clip to map bounds to avoid out-of-range errors
    gx = std::clamp(gx, 0, static_cast<int>(map.info.width) - 1);
    gy = std::clamp(gy, 0, static_cast<int>(map.info.height) - 1);


    return CellIndex(gx, gy);
}

void PlannerNode::planPath()
{
  

  CellIndex start = worldToGrid(current_x_, current_y_);
  CellIndex goal  = worldToGrid(goal_x_, goal_y_);


  const int width  = map.info.width;
  const int height = map.info.height;

  auto isValid = [&](const CellIndex &c) {
    if (c.x < 0 || c.y < 0 || c.x >= width || c.y >= height)
      return false;

    int idx = c.y * width + c.x;
    return map.data[idx] < 50;
  };

  int start_idx = start.y * width + start.x;
  int goal_idx  = goal.y * width + goal.x;

  RCLCPP_INFO(this->get_logger(),
  "Start cell value: %d | Goal cell value: %d",
  map.data[start_idx], map.data[goal_idx]);

  if (!isValid(start)) {
    RCLCPP_WARN(this->get_logger(), "Start cell invalid (%d, %d)", start.x, start.y);
    return;
  }

  if (!isValid(goal)) {
    RCLCPP_WARN(this->get_logger(), "Goal cell invalid (%d, %d)", goal.x, goal.y);
    return;
  }
  
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  auto heuristic = [&](const CellIndex &a, const CellIndex &b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };
  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start, goal));

  // here
  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal) {
      // -------- Path reconstruction --------
      nav_msgs::msg::Path path;
      path.header.frame_id = map.header.frame_id;
      path.header.stamp = this->now();

      CellIndex c = current;
      while (c != start) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;

        pose.pose.position.x =
          map.info.origin.position.x +
          (c.x + 0.5) * map.info.resolution;

        pose.pose.position.y =
          map.info.origin.position.y +
          (c.y + 0.5) * map.info.resolution;

        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);
        c = came_from[c];
      }

      std::reverse(path.poses.begin(), path.poses.end());
      path_pub_->publish(path);

      RCLCPP_INFO(this->get_logger(), "Path found!");
      return;
    }

    // 4-connected neighbors
    std::vector<CellIndex> neighbors = {
      {current.x + 1, current.y},
      {current.x - 1, current.y},
      {current.x, current.y + 1},
      {current.x, current.y - 1}
    };

    for (const auto &n : neighbors) {
      if (!isValid(n)) continue;

      double tentative_g = g_score[current] + 1.0;

      if (!g_score.count(n) || tentative_g < g_score[n]) {
        came_from[n] = current;
        g_score[n] = tentative_g;
        double f = tentative_g + heuristic(n, goal);
        open_set.emplace(n, f);
      }
    }
  }

  RCLCPP_WARN(this->get_logger(), "No path found");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
