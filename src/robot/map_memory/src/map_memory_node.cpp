#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  
  costmap_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odometryCallback, this, std::placeholders::_1));

  map_memory_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  last_x = 0.0;
  last_y = 0.0;
  yaw = 0.0;

  distance_threshold = 1.5; // meters

  should_update = true; // Force update on initialization

  nav_msgs::msg::OccupancyGrid world_map = nav_msgs::msg::OccupancyGrid();
  world_map.info.resolution = 0.1; // each cell is 0.1m x 0.1m
  world_map.info.width = 5400; // 540m / 0.1
  world_map.info.height = 5400; // 540m / 0.1
  world_map.header.frame_id = "map"; // WORLD CENTRIC frame
  world_map.data.resize(5400 * 5400, 0); // Initialize all cells to free space (0)
}

void MapMemoryNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double current_x = msg->pose.pose.position.x;
  double current_y = msg->pose.pose.position.y;

  double distance_moved = std::sqrt(std::pow(current_x - last_x, 2) + std::pow(current_y - last_y, 2));

  OccupancyGrid[540][540] = {0}; // 2D array to represent the WORLD CENTRIC map, initialized to 0 (free space)

  if (distance_moved >= distance_threshold) { // Update map memory if moved more than 0.5m
    last_x = current_x;
    last_y = current_y;
    yaw = tf2::getYaw(msg->pose.pose.orientation);    

    should_update = true;
  }
  else
  {
    should_update = false;
  } 

}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (should_update) {

    global_map = addToWorldMap(msg);
    global_map.header.stamp = this->now();
    global_map.header.frame_id = "map"; // WORLD CENTRIC frame

    map_memory_pub_->publish(global_map);
    should_update = false; // Reset the flag
  }
}


nav_msgs::msg::OccupancyGrid MapMemoryNode::addToWorldMap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {

  // Integrate the incoming costmap into the world-centric map memory

  int8_t unflattened_costmap[360][360] = {}; // 2D array to represent the WORLD CENTRIC map, initialized to 0 (free space)

  for(int i = 0; i < costmap->info.height; i++) {
    for(int j = 0; j < costmap->info.width; j++) {

      int index = j + (i * costmap->info.width);
      int8_t cell_value = costmap->data[index];

      unflattened_costmap[i][j] = cell_value;

    }
  }
  
  for(int i = 0; i < 360; i++) {
    for(int j = 0; j < 360; j++) {

    
      // (270, 270) is center of world map
      // 0.1m per cell

      // bot_pose_x_cells = static_cast<int>(last_x / 0.1) + 270; // Convert to cell index, (270, 270) is center of the map
      // bot_pose_y_cells = static_cast<int>(last_y / 0.1) + 270; // Convert to cell index, (270, 270) is center of the map

      // translation vectors (in metres)
      costmap_tx_metres = (i - 180) * 0.1; // Convert cell index to meters relative to robot
      costmap_ty_metres = (j - 180) * 0.1; // Convert cell index to meters relative to robot

      global_costmap_posx = (std::cos(yaw) * costmap_tx_metres - std::sin(yaw) * costmap_ty_metres) + last_x;
      global_costmap_posy = (std::sin(yaw) * costmap_tx_metres + std::cos(yaw) * costmap_ty_metres) + last_y;

      world_x_cells = static_cast<int>(global_costmap_posx / 0.1) + 270; // Convert to cell index (0.1m resolution), (270, 270) is center of the map
      world_y_cells = static_cast<int>(global_costmap_posy / 0.1) + 270; // Convert to cell index (0.1m resolution), (270, 270) is center of the map

      if (world_x_cells < 0 || world_x_cells >= 5400 || world_y_cells < 0 || world_y_cells >= 5400) {
        continue; // Skip out-of-bounds cells
      }

      if(OccupancyGrid[world_x_cells][world_y_cells] <= unflattened_costmap[i][j] && unflattened_costmap[i][j] != -1) {

        OccupancyGrid[world_x_cells][world_y_cells] = unflattened_costmap[i][j]; // Update the world map cell if new value is higher

      } // Unknown (-1) does not change the world map

    }
  }

  nav_msgs::msg::OccupancyGrid world_map = nav_msgs::msg::OccupancyGrid();
  world_map.data.resize(5400 * 5400);
  for (int i = 0; i < 5400; i++) {
    for (int j = 0; j < 5400; j++) {
      world_map.data[i * 5400 + j] = OccupancyGrid[i][j];
    }
  }

  return world_map;

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
