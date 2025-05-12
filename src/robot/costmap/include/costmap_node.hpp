#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "costmap_core.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <iostream>
#include <vector>
#include <tf2/utils.h>
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishCostmap();
    void convertLaserToGrid(const std::vector<float>& ranges, float angle_min, float angle_max, float angle_increment);
    void applyInflation();
 
  private:
    robot::CostmapCore costmap_core_;
    double current_orientation_ = 0.0;
    int occupancy_grid_[10][10] = {};
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
    rclcpp::TimerBase::SharedPtr update_timer_;
};


 
#endif 