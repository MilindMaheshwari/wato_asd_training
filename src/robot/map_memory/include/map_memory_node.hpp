#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void integrateCostmapIntoGlobalMap();
    void updateMap();

  private:
    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;


    nav_msgs::msg::OccupancyGrid latest_costmap_;
    nav_msgs::msg::Odometry latest_odom_;
    nav_msgs::msg::OccupancyGrid global_map_;
    double last_x = 0.0; 
    double last_y = 0.0;
    double yaw = 0.0;
    const double distance_threshold = 1.0;
    bool costmap_updated_ = false;
    bool should_update_map_ = false; // Set to true at beginning to force first update

    int global_width = 360;
    int global_height = 360;

    int global_center_x = global_width / 2;
    int global_center_y = global_height / 2;

    int global_grid[360][360] = {};
};

#endif 
