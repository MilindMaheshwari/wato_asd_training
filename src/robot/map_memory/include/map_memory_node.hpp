#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "nav_msgs/msg/odometry.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    
    nav_msgs::msg::OccupancyGrid addToWorldMap();
    void costmapCallback();

  private:
    robot::MapMemoryCore map_memory_;

    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_memory_pub_;

    double last_x, last_y, distance_threshold, yaw; 
    int8_t OccupancyGrid[540][540];
    bool should_update = false;
};

#endif 
