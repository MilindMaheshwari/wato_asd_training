#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters

  lidar_subscribtion_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

  // string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  // lidar_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lidar_pub", 10);
  
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
} 
 
// // Define the timer to publish a message every 500ms
// void CostmapNode::publishMessage() {
//   auto message = std_msgs::msg::String();
//   message.data = "Hello, ROS 2!";
//   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//   string_pub_->publish(message);
// }

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  // Lidar range_max = 20m 
  // 360 degree lidar ?
  // -pi to +pi 
  // Make grid width 36m by 36m (18m in each direction) 
  // Each cell is 0.1m x 0.1m
  // Grid is 360 cells by 360 cells
  // Robot is at the center of the grid (cell 90, 90) 

  nav_msgs::msg::OccupancyGrid RobotCentricGrid = nav_msgs::msg::OccupancyGrid();
  RobotCentricGrid.info.resolution = 0.1; // each cell is 0.1m x 0.1m
  RobotCentricGrid.info.width = 360; // 18m / 0.1m
  RobotCentricGrid.info.height = 360; // 18m / 0.1m
  RobotCentricGrid.header.stamp = this->now();
  RobotCentricGrid.header.frame_id = "robot_base";

  int inflation_radius = 1; // 1m around each obstacle is also marked as occupied, decreasing linearly

  int x_cells, y_cells;
  float cost, angle, range, x, y, distance;

  float OccupancyGrid[360][360] = {0}; // 2D array to represent the grid, initialized to 0 (free space)

  for(int i = 0; i < msg->ranges.size(); i++) {

    angle = msg->angle_min + i * msg->angle_increment;
    range = msg->ranges[i];

    if (range < msg->range_min || range > msg->range_max) { // Should be 20m max, 0.079m min
      continue; // Ignore measurements outside valid range
    }

    x = range * sin(angle);
    y = range * cos(angle);

    x_cells = (x / RobotCentricGrid.info.resolution) + 180; // shift right (since 0,0 is bottom left)
    y_cells = (y / RobotCentricGrid.info.resolution) + 180; // shift right (since 0,0 is bottom left)

    // Bounds checking
    if (x_cells >= 0 && x_cells < 360 && y_cells >= 0 && y_cells < 360) {
      OccupancyGrid[x_cells][y_cells] = 100; // Mark as occupied
    }


    // Inflate the obstacle
    for(int dx = -10; dx <= 10; dx++) { // 10 cells = 1m
      for(int dy = -10; dy <= 10; dy++) {
        int nx = x_cells + dx;
        int ny = y_cells + dy;
        // skip out-of-bounds neighbors
        if (nx < 0 || nx >= 360 || ny < 0 || ny >= 360) {
          continue;
        }
        distance = sqrt(dx*dx + dy*dy) * RobotCentricGrid.info.resolution;
        if (distance <= inflation_radius) {

          cost = 100 * (1 -(distance / inflation_radius)); // Linear decrease from 100 to 0
          
          if (OccupancyGrid[nx][ny] <= cost) { // Don't update if already higher value
            OccupancyGrid[nx][ny] = cost; 
          }
        }
      }
    }
  }

  // Flatten 2D array to 1D array for OccupancyGrid message
  for(int i = 0; i < 360; i++) {
    for(int j = 0; j < 360; j++) {
      RobotCentricGrid.data.push_back(OccupancyGrid[i][j]);
    }
  }

  // Publish the occupancy grid
  costmap_pub_->publish(RobotCentricGrid);

}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
