#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void integrateCostmapIntoGlobalMap();
    void updateMap();

  private:
    void syncCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap_msg, 
                      const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);


    robot::MapMemoryCore map_memory_;


    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;



    message_filters::Subscriber<nav_msgs::msg::OccupancyGrid> costmap_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::OccupancyGrid, nav_msgs::msg::Odometry> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    nav_msgs::msg::Odometry latest_odom_;
    nav_msgs::msg::OccupancyGrid global_map_;
    double last_x = 0.0; 
    double last_y = 0.0;
    double curr_yaw = 0.0;
    double last_yaw = 0.0;

    
    double rotated_x = 0.0;
    double rotated_y = 0.0;

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
