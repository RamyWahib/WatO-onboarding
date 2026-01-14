#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  lookahead_distance_ = 1.0;
  goal_tolerance = 0.1;
  linear_speed_ = 1.0;
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&ControlNode::readOdometry, this, std::placeholders::_1));
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&ControlNode::readPath, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::readOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_odom_ = msg;
}

void ControlNode::readPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = msg;
}
void ControlNode::controlLoop(){
  // Skip control if no path or odometry data is available
        if (!current_path_ || !robot_odom_) {
            return;
        }
  // Find the lookahead point
        auto lookahead_point = findLookaheadPoint();
        if (!lookahead_point) {
            return;  // No valid lookahead point found
        }

        // Compute velocity command
        auto cmd_vel = computeVelocity(*lookahead_point);
 
        // Publish the velocity command
        cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint(){
  const auto &robot_pos = robot_odom_->pose.pose.position;

  for (const auto &pose : current_path_->poses)
  {
    double dist = computeDistance(robot_pos, pose.pose.position);
    if (dist >= lookahead_distance_)
    {
      return pose;
    }
  }

  // If we didn't find one, use final goal
  if (!current_path_->poses.empty())
  {
    return current_path_->poses.back();
  }

  return std::nullopt; // Replace with a valid point when implemented
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target){
    geometry_msgs::msg::Twist cmd;

  const auto &robot_pose = robot_odom_->pose.pose;
  const auto &robot_pos = robot_pose.position;

  double yaw = extractYaw(robot_pose.orientation);

  // Transform target into robot frame
  double dx = target.pose.position.x - robot_pos.x;
  double dy = target.pose.position.y - robot_pos.y;

  double x_r =  std::cos(-yaw) * dx - std::sin(-yaw) * dy;
  double y_r =  std::sin(-yaw) * dx + std::cos(-yaw) * dy;

  double Ld = std::hypot(x_r, y_r);
  if (Ld < goal_tolerance)
  {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }

  // Pure pursuit curvature
  double curvature = (2.0 * y_r) / (Ld * Ld);

  cmd.linear.x = linear_speed_;
  cmd.angular.z = curvature * linear_speed_;

  return cmd;

}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b){
        return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    return tf2::getYaw(tf_q);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
