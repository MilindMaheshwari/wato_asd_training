#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters

  lidar_subscribtion_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

  // lidar_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lidar_pub", 10);
  
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
} 
 


void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  // Lidar range_max = 20m 
  // 360 degree lidar ?
  // -pi to +pi 
  // Make grid width 36m by 36m (18m in each direction) 
  // Each cell is 0.1m x 0.1m
  // Grid is 360 cells by 360 cells
  // Robot is at the center of the grid (cell 90, 90) 

  // NEVERMIND ENTIRE FOXGLOVE GRID IS 30m x 30m

  nav_msgs::msg::OccupancyGrid RobotCentricGrid = nav_msgs::msg::OccupancyGrid();
  RobotCentricGrid.info.resolution = 0.1; // each cell is 0.1m x 0.1m
  RobotCentricGrid.info.width = 360; // 18m / 0.1m
  RobotCentricGrid.info.height = 360; // 18m / 0.1m

  // FOR WIDHT OF 360, origin.x = -18, origin.y = -18
  

  RobotCentricGrid.info.origin.position.x = -(RobotCentricGrid.info.width/2 * RobotCentricGrid.info.resolution);  // SHOULD BE -18
  RobotCentricGrid.info.origin.position.y = -(RobotCentricGrid.info.height/2 * RobotCentricGrid.info.resolution); // SHOULD = -18, not -18.000000 

  RobotCentricGrid.header.stamp = msg->header.stamp;
  RobotCentricGrid.header.frame_id = "base_link";

  int inflation_radius = 1; // 1m around each obstacle is also marked as occupied, decreasing linearly

  int x_cells, y_cells;
  float cost, angle, range, x, y, distance;

  float OccupancyGrid[RobotCentricGrid.info.width][RobotCentricGrid.info.height] = {0}; // 2D array to represent the grid, initialized to 0 (free space)

  for(int i = 0; i < msg->ranges.size(); i++) {

    angle = msg->angle_min + i * msg->angle_increment;
    range = msg->ranges[i];

    if (range < msg->range_min || range > msg->range_max) { // Should be 20m max, 0.079m min
      continue; // Ignore measurements outside valid range
    }

    x = range * sin(angle);
    y = range * cos(angle);

    x_cells = (x / RobotCentricGrid.info.resolution) + RobotCentricGrid.info.width/2; // shift right (since 0,0 is bottom left)
    y_cells = (y / RobotCentricGrid.info.resolution) + RobotCentricGrid.info.height/2; // shift right (since 0,0 is bottom left)

    // Bounds checking
    if (x_cells >= 0 && x_cells < RobotCentricGrid.info.width && y_cells >= 0 && y_cells < RobotCentricGrid.info.height) {
      OccupancyGrid[x_cells][y_cells] = 100; // Mark as occupied
    }
    else{
      // RCLCPP_INFO(this->get_logger(), "Out of bounds: x_cells: %d, y_cells: %d", x_cells, y_cells);
      continue;
    }


    // Inflate the obstacle
    for(int dx = -10; dx <= 10; dx++) { // 10 cells = 1m
      for(int dy = -10; dy <= 10; dy++) {
        int nx = x_cells + dx;
        int ny = y_cells + dy;
        // skip out-of-bounds neighbors
        if (nx < 0 || nx >= RobotCentricGrid.info.width || ny < 0 || ny >= RobotCentricGrid.info.height) {
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
  for(int i = 0; i < RobotCentricGrid.info.width; i++) {
    for(int j = 0; j < RobotCentricGrid.info.height; j++) {
      RobotCentricGrid.data.push_back(OccupancyGrid[i][j]);
    }
  }

  // RCLCPP_INFO(this->get_logger(), "Published costmap with %d non zero cells", std::count_if(RobotCentricGrid.data.begin(), RobotCentricGrid.data.end(), [](int cell) { return cell != 0; }));
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
