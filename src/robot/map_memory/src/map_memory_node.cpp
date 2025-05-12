#include "map_memory_node.hpp"

namespace robot {

MapMemoryNode::MapMemoryNode() 
    : Node("map_memory")
    , last_x_(0.0)
    , last_y_(0.0)
    , map_memory_(MapMemoryCore(this->get_logger())) 
{
    // Setup subscriptions
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 
        10, 
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { this->costmapCallback(msg); }
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 
        10, 
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->odomCallback(msg); }
    );

    // Setup publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Setup periodic timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), 
        [this]() { this->updateMap(); }
    );

    // Initialize map metadata
    initializeMapMetadata();
}

void MapMemoryNode::initializeMapMetadata() {
    nav_msgs::msg::MapMetaData metadata;
    metadata.resolution = 1.0 / MapConfig::scale_factor;
    metadata.width = MapConfig::width * MapConfig::scale_factor;
    metadata.height = MapConfig::height * MapConfig::scale_factor;
    metadata.origin.position.x = -20.0;
    metadata.origin.position.y = -20.0;
    metadata.origin.orientation.w = 1.0;
    
    global_map_.info = metadata;
    global_map_.data.resize(MapConfig::width * MapConfig::scale_factor * MapConfig::height * MapConfig::scale_factor, 0);
}

// Process incoming costmap data
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

// Process odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_pose_.yaw = quaternionToYaw(msg->pose.pose.orientation);

    // Check if we've moved enough to warrant an update
    double dist_moved = std::hypot(current_pose_.x - last_x_, current_pose_.y - last_y_);
    if (dist_moved >= MapConfig::distance_threshold) {
        last_x_ = current_pose_.x;
        last_y_ = current_pose_.y;
        should_update_map_ = true;
    }
}

// Periodic map update handler
void MapMemoryNode::updateMap() {
    if (!should_update_map_ || !costmap_updated_) {
        return;
    }

    integrateCostmap();
    map_pub_->publish(global_map_);
    RCLCPP_INFO(this->get_logger(), "Map memory updated successfully");
    should_update_map_ = false;
}

// Convert quaternion to yaw angle
double MapMemoryNode::quaternionToYaw(const geometry_msgs::msg::Quaternion& q) const {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// Integrate latest costmap into global map
void MapMemoryNode::integrateCostmap() {
    // Calculate costmap origin in global frame
    const double costmap_origin_x_global = current_pose_.x + latest_costmap_.info.origin.position.x;
    const double costmap_origin_y_global = current_pose_.y + latest_costmap_.info.origin.position.y;

    // Define rotation center
    const double costmap_center_x = MapConfig::width * MapConfig::scale_factor / 2.0;
    const double costmap_center_y = MapConfig::height * MapConfig::scale_factor / 2.0;

    // Process each cell in the costmap
    for (unsigned int cy = 0; cy < latest_costmap_.info.height; ++cy) {
        for (unsigned int cx = 0; cx < latest_costmap_.info.width; ++cx) {
            // Calculate relative coordinates
            const double cell_x_rel = (cx * latest_costmap_.info.resolution) - costmap_center_x * latest_costmap_.info.resolution;
            const double cell_y_rel = (cy * latest_costmap_.info.resolution) - costmap_center_y * latest_costmap_.info.resolution;

            // Apply rotation
            const double rotated_x_rel = std::cos(current_pose_.yaw) * cell_x_rel - std::sin(current_pose_.yaw) * cell_y_rel;
            const double rotated_y_rel = std::sin(current_pose_.yaw) * cell_x_rel + std::cos(current_pose_.yaw) * cell_y_rel;

            // Convert to global coordinates
            const double global_x_meters = rotated_x_rel + costmap_origin_x_global + costmap_center_x * latest_costmap_.info.resolution;
            const double global_y_meters = rotated_y_rel + costmap_origin_y_global + costmap_center_y * latest_costmap_.info.resolution;

            // Convert to grid coordinates
            const int global_x = static_cast<int>((global_x_meters - global_map_.info.origin.position.x) / global_map_.info.resolution);
            const int global_y = static_cast<int>((global_y_meters - global_map_.info.origin.position.y) / global_map_.info.resolution);

            // Update global map if within bounds and cell has data
            if (isWithinBounds(global_x, global_y)) {
                const int costmap_idx = cy * latest_costmap_.info.width + cx;
                const int global_idx = global_y * global_map_.info.width + global_x;
                
                if (latest_costmap_.data[costmap_idx] > 0) {
                    global_map_.data[global_idx] = latest_costmap_.data[costmap_idx];
                }
            }
        }
    }

    // Update header information
    global_map_.header = latest_costmap_.header;
    global_map_.header.frame_id = "sim_world";
    costmap_updated_ = false;
}

// Helper function to check if coordinates are within map bounds
bool MapMemoryNode::isWithinBounds(int x, int y) const {
    return x >= 0 && x < global_map_.info.width && y >= 0 && y < global_map_.info.height;
}

} // namespace robot

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot::MapMemoryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
