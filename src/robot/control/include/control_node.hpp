#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;

    double lookahead_distance_ =  1.0;
    double goal_tolerance_ = 0.1;
    double linear_speed_ = 0.75;  // Constant forward speed 

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;


    void controlLoop() {
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
 
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() {

        // TODO: Implement logic to find the lookahead point on the path
        // If path has no poses, we can't find a lookahead point
        if (!current_path_->poses.size()) {
            RCLCPP_WARN(this->get_logger(), "Received empty path; skipping control update");
            return std::nullopt;
        }

        geometry_msgs::msg::Pose robot_pose = robot_odom_->pose.pose;

        for(const auto & pose_stamped : current_path_->poses) {
            double distance = computeDistance(robot_pose.position, pose_stamped.pose.position);
            if (distance >= lookahead_distance_) {
                return pose_stamped;
            }
        }

        // Return goal if lookahead not found
        return current_path_->poses.back();

    }
 
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
        // TODO: Implement logic to compute velocity commands

        // Differential drive

        if(computeDistance(robot_odom_->pose.pose.position, target.pose.position) < goal_tolerance_) {
            // Stop if within goal tolerance
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            return stop_cmd;
        }
        
        // angular velocity = linear_speed * 2y / (L^2) (y = horizontal offset to target, L = lookahead distance)

        double lat_distance = target.pose.position.y - robot_odom_->pose.pose.position.y;

        double dx  = target.pose.position.x - robot_odom_->pose.pose.position.x;
        double dy  = target.pose.position.y - robot_odom_->pose.pose.position.y;

        double local_x_diff =  cos(extractYaw(robot_odom_->pose.pose.orientation)) * dx + sin(extractYaw(robot_odom_->pose.pose.orientation)) * dy;
        double local_y_diff = -sin(extractYaw(robot_odom_->pose.pose.orientation)) * dx + cos(extractYaw(robot_odom_->pose.pose.orientation)) * dy;

        double angular_v = (2 * linear_speed_ * local_y_diff) / std::pow(lookahead_distance_  , 2);

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_speed_;
        cmd_vel.angular.z = angular_v;

        return cmd_vel;
    }
 
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
        // TODO: Implement distance calculation between two points

        return hypot(a.x - b.x, a.y - b.y);
    }
 
    double extractYaw(const geometry_msgs::msg::Quaternion &quat) {
        // TODO: Implement quaternion to yaw conversion
        return tf2::getYaw(quat);
    }
 


};

#endif
