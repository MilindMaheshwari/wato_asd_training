#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {

  // Initialize subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

  double curr_x = msg->pose.pose.position.x;
  double curr_y = msg->pose.pose.position.y;

  // Compute distance traveled
  double distance = std::sqrt(std::pow(curr_x - last_x, 2) + std::pow(curr_y - last_y, 2));
  if (distance >= distance_threshold) {
      last_x = curr_x;
      last_y = curr_y;
      should_update_map_ = true;

      RCLCPP_INFO(this->get_logger(), "Updated map with distance: %f", distance);
  }


}

void MapMemoryNode::updateMap() {

  if (should_update_map_ && costmap_updated_) {
    integrateCostmapIntoGlobalMap();
  }
}


void MapMemoryNode::integrateCostmapIntoGlobalMap() {

  int local_x_from_center, local_y_from_center, global_x, global_y;
  int local_x, local_y;

  global_map_ = nav_msgs::msg::OccupancyGrid();
  global_map_.info.resolution = 0.1; // each cell is 0.1m x 0.1m
  global_map_.info.width = 360; // 18m / 0.1m
  global_map_.info.height = 360; // 18m / 0.1m
  global_map_.info.origin.position.x = -(global_map_.info.width/2 * global_map_.info.resolution);  // SHOULD BE -18
  global_map_.info.origin.position.y = -(global_map_.info.height/2 * global_map_.info.resolution); // SHOULD = -18, not -18.000000 
  global_map_.header.stamp = this->now();
  global_map_.header.frame_id = "sim_world";


  local_x = (float)latest_odom_.pose.pose.position.x / latest_costmap_.info.resolution;
  local_y = (float)latest_odom_.pose.pose.position.y / latest_costmap_.info.resolution;

  for (int i = 0; i < latest_costmap_.info.height; i++) {
    for (int j = 0; j < latest_costmap_.info.width; j++) {


      local_x_from_center = j - latest_costmap_.info.width / 2;
      local_y_from_center = i - latest_costmap_.info.height / 2;

      global_x = global_center_x + local_x_from_center + local_x;
      global_y = global_center_y + local_y_from_center + local_y;

      if (global_x >= 0 && global_x < global_width && global_y >= 0 && global_y < global_height) {
        if (latest_costmap_.data[i * latest_costmap_.info.width + j] > global_grid[global_y][global_x]) {
          global_grid[global_y][global_x] = latest_costmap_.data[i * latest_costmap_.info.width + j];
        }
      }
    }
  }

  for(int i = 0; i < global_map_.info.width; i++) {
    for(int j = 0; j < global_map_.info.height; j++) {
      global_map_.data.push_back(global_grid[i][j]);
    }
  }


  map_pub_->publish(global_map_);
  should_update_map_ = false;

}


void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {

  latest_costmap_ = *msg;
  costmap_updated_ = true;

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
