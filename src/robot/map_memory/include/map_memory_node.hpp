#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot {

/**
 * @brief Node for maintaining a persistent map memory of the environment
 * 
 * This node integrates costmap updates into a global map while handling
 * robot pose transformations and map updates.
 */
class MapMemoryNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the MapMemoryNode
     */
    MapMemoryNode();

private:
    // Core functionality
    void initializeMapMetadata();
    void integrateCostmap();
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) const;
    bool isWithinBounds(int x, int y) const;

    // Callback handlers
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();

    // ROS communication
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Map configuration
    struct MapConfig {
        static constexpr double distance_threshold = 1.5;
        static constexpr int scale_factor = 10;
        static constexpr int width = 40;
        static constexpr int height = 40;
    };

    // Robot state
    struct Pose {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    // Member variables
    MapMemoryCore map_memory_;
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    Pose current_pose_;
    double last_x_{0.0};
    double last_y_{0.0};

    // State flags
    bool costmap_updated_{false};
    bool should_update_map_{false};
};

} // namespace robot

#endif // MAP_MEMORY_NODE_HPP_ 
