#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  // Initialize the constructs and their parameters
  string_sub_ = this->create_subscription <sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::readMessage, this, std::placeholders::_1));
  occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",10);
}


void CostmapNode::readMessage(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  float resolution = 0.1;
  int width = 100;
  int height = 100;
  int default_value = 0;

  float inflation_radius = 1.0;
  int max_cost = 100;
  int inflation_cells = static_cast<int>(inflation_radius / resolution);

  std::vector<std::vector<int>> occupancy_grid(
    height, std::vector<int>(width, default_value));

  // --- Mark obstacles ---
  for (size_t i = 0; i < msg->ranges.size(); i++)
  {
    float range = msg->ranges[i];

    if (range < msg->range_min || range > msg->range_max)
      continue;

    float angle = msg->angle_min + i * msg->angle_increment;

    float x = range * std::cos(angle);
    float y = range * std::sin(angle);

    int col = static_cast<int>(x / resolution + width / 2);
    int row = static_cast<int>(y / resolution + height / 2);

    if (row >= 0 && row < height && col >= 0 && col < width)
      occupancy_grid[row][col] = max_cost;
  }

  // --- Inflate obstacles ---
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      if (occupancy_grid[y][x] == max_cost)
      {
        for (int dy = -inflation_cells; dy <= inflation_cells; dy++)
        {
          for (int dx = -inflation_cells; dx <= inflation_cells; dx++)
          {
            int ny = y + dy;
            int nx = x + dx;

            if (ny < 0 || ny >= height || nx < 0 || nx >= width)
              continue;

            float distance = std::sqrt(dx * dx + dy * dy) * resolution;

            if (distance <= inflation_radius)
            {
              int cost = static_cast<int>(
                max_cost * (1.0f - distance / inflation_radius));

              occupancy_grid[ny][nx] =
                std::max(occupancy_grid[ny][nx], cost);
            }
          }
        }
      }
    }
  }
 nav_msgs::msg::OccupancyGrid grid_msg;

  // --- Header ---
  grid_msg.header.stamp = this->get_clock()->now();
  grid_msg.header.frame_id = "map";

  // --- Metadata ---
  grid_msg.info.resolution = resolution;
  grid_msg.info.width = width;
  grid_msg.info.height = height;

  // Origin: bottom-left corner of the map
  grid_msg.info.origin.position.x = -width * resolution / 2.0;
  grid_msg.info.origin.position.y = -height * resolution / 2.0;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  // --- Data (flatten 2D → 1D) ---
  grid_msg.data.resize(width * height);

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int index = y * width + x;
      grid_msg.data[index] = occupancy_grid[y][x];
    }
  }
  occupancy_pub_->publish(grid_msg);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;

  // 1️⃣ Define costmap parameters
  
}