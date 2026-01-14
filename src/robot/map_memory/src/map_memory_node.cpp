#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription <nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::readCostmap, this, std::placeholders::_1));
  odometry_sub_ = this->create_subscription <nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::readOdometry, this, std::placeholders::_1));
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local();
  qos.reliable();
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);

  global_map_.info.width = 500;    // example size
  global_map_.info.height = 500;
  global_map_.info.resolution = 0.1;
  global_map_.info.origin.position.x = -25.0;
  global_map_.info.origin.position.y = -25.0;
  global_map_.info.origin.orientation.w = 1.0; // identity
  global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1);

  global_map_.header.frame_id = "map";
  global_map_.header.stamp = this->now();
  costmap_pub_->publish(global_map_);

  update_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MapMemoryNode::updateMapIfMoved, this));
}

void MapMemoryNode::readCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  MapMemoryNode::latestCostmap = *msg;

  // 🔑 force first integration
  if (!first_pose_ && latest_odom_) {
    integrateCostmap(latest_odom_);
  }
}
void MapMemoryNode::readOdometry(const nav_msgs::msg::Odometry::SharedPtr msg){
  MapMemoryNode::latest_odom_ = msg;
 
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;

  // First pose initialization
  if (MapMemoryNode::first_pose_) {
    MapMemoryNode::last_update_x_ = x;
    MapMemoryNode::last_update_y_ = y;
    MapMemoryNode::first_pose_ = false;
    return;
  }

  const double dx = x - MapMemoryNode::last_update_x_;
  const double dy = y - MapMemoryNode::last_update_y_;
  const double distance = std::sqrt(dx * dx + dy * dy);

  constexpr double UPDATE_DISTANCE = 1.5; // meters

  if (distance < UPDATE_DISTANCE) {
    return;
  }

  MapMemoryNode::last_update_x_ = x;
  MapMemoryNode::last_update_y_ = y;

  // Integrate costmap when robot moves enough
  MapMemoryNode::integrateCostmap(msg);
}

void MapMemoryNode::integrateCostmap(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  // Extract robot yaw from odometry using tf2
    tf2::Quaternion q(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); // get yaw in radians

    // Robot's current position
    double robot_x = odom->pose.pose.position.x;
    double robot_y = odom->pose.pose.position.y;

    // Loop through every cell in the latest costmap
    for (unsigned int cy = 0; cy < latestCostmap.info.height; ++cy) {
        for (unsigned int cx = 0; cx < latestCostmap.info.width; ++cx) {
            int idx = cy * latestCostmap.info.width + cx;
            int value = latestCostmap.data[idx];
            if (value < 0) value = 0; ; // ignore unknown

            // Costmap cell position relative to its origin
            double local_x = latestCostmap.info.origin.position.x + cx * latestCostmap.info.resolution;
            double local_y = latestCostmap.info.origin.position.y + cy * latestCostmap.info.resolution;

            // Rotate and translate to global frame
            double global_x = cos(yaw) * local_x - sin(yaw) * local_y + robot_x;
            double global_y = sin(yaw) * local_x + cos(yaw) * local_y + robot_y;

            // Convert to global map indices
            int gx = static_cast<int>((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
            int gy = static_cast<int>((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

            // Skip if outside global map
            if (gx < 0 || gy < 0 || gx >= static_cast<int>(global_map_.info.width) || gy >= static_cast<int>(global_map_.info.height)) {
                continue;
            }

            // Update global map with new costmap data
            global_map_.data[gy * global_map_.info.width + gx] = value;
        }
    }

    //publish updated global map
    costmap_pub_->publish(global_map_);
}
void MapMemoryNode::updateMapIfMoved()
{
    if (first_pose_) return; // no valid pose yet

    double x = latest_odom_->pose.pose.position.x;
    double y = latest_odom_->pose.pose.position.y;

    double dx = x - last_update_x_;
    double dy = y - last_update_y_;
    double distance = std::sqrt(dx*dx + dy*dy);

    constexpr double UPDATE_DISTANCE = 1.5; // meters

    if (distance < UPDATE_DISTANCE) return; // robot hasn't moved enough

    // Update last position
    last_update_x_ = x;
    last_update_y_ = y;

    // Integrate latest costmap
    integrateCostmap(latest_odom_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
