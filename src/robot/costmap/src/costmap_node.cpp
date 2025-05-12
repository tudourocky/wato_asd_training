#include <chrono>
#include <memory>


#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_core_(robot::CostmapCore(this->get_logger())) 
{
    // Setup publishers and subscribers
    costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 
        10, 
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->handleOdometry(msg); }
    );
    
    // Setup timer for periodic updates
    update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), 
        [this]() { this->publishCostmap(); }
    );
    
    // Setup laser scan subscriber
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 
        10, 
        std::bind(&CostmapNode::handleLaserScan, this, std::placeholders::_1)
    );
}

void CostmapNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract yaw from quaternion
    const auto& q = msg->pose.pose.orientation;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 rotation_matrix(quaternion);
    
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);
    current_orientation_ = yaw;
}

void CostmapNode::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Reset occupancy grid
    std::fill(&occupancy_grid_[0][0], &occupancy_grid_[0][0] + 100, 0);
    
    // Process scan data
    convertLaserToGrid(
        scan->ranges,
        scan->angle_min,
        scan->angle_min,
        scan->angle_increment
    );
    
    publishCostmap();
}

void CostmapNode::convertLaserToGrid(
    const std::vector<float>& ranges,
    float angle_min,
    float angle_max,
    float angle_increment
) {
    constexpr float GRID_RESOLUTION = 0.5f;
    constexpr int ROBOT_GRID_X = 5;
    constexpr int ROBOT_GRID_Y = 5;
    constexpr int MAX_GRID_SIZE = 10;

    for (size_t i = 0; i < ranges.size() && i < 256; ++i) {
        float angle = angle_min + i * angle_increment;
        float global_angle = angle + current_orientation_;
        
        float x = ranges[i] * std::cos(global_angle);
        float y = ranges[i] * std::sin(global_angle);
        
        int grid_x = ROBOT_GRID_X + static_cast<int>(x / GRID_RESOLUTION);
        int grid_y = ROBOT_GRID_Y + static_cast<int>(y / GRID_RESOLUTION);
        
        if (grid_x >= 0 && grid_x < MAX_GRID_SIZE && grid_y >= 0 && grid_y < MAX_GRID_SIZE) {
            occupancy_grid_[grid_x][grid_y] = 100;  // Mark as occupied
        }
    }
    
    applyInflation();
}

void CostmapNode::applyInflation() {
    constexpr float GRID_RESOLUTION = 0.5f;
    constexpr int MAX_GRID_SIZE = 10;
    const int inflation_radius = static_cast<int>(5.0f / GRID_RESOLUTION);
    
    for (int x = 0; x < MAX_GRID_SIZE; ++x) {
        for (int y = 0; y < MAX_GRID_SIZE; ++y) {
            if (occupancy_grid_[x][y] == 100) {
                for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                    for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                        if (dx == 0 && dy == 0) continue;
                        
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if (nx >= 0 && nx < MAX_GRID_SIZE && ny >= 0 && ny < MAX_GRID_SIZE) {
                            float distance = std::sqrt(dx*dx + dy*dy) * GRID_RESOLUTION;
                            int cost = static_cast<int>(80.0f * (1.0f - distance/inflation_radius));
                            
                            if (cost > occupancy_grid_[nx][ny]) {
                                occupancy_grid_[nx][ny] = cost;
                            }
                        }
                    }
                }
            }
        }
    }
}

void CostmapNode::publishCostmap() {
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = "map";
    
    // Configure grid parameters
    grid_msg.info.resolution = 0.5;
    grid_msg.info.width = 10;
    grid_msg.info.height = 10;
    grid_msg.info.origin.position.x = -5.0;
    grid_msg.info.origin.position.y = 5.0;
    
    // Copy occupancy grid data
    grid_msg.data.resize(100);
    for (int y = 0; y < 10; ++y) {
        for (int x = 0; x < 10; ++x) {
            grid_msg.data[y * 10 + x] = occupancy_grid_[x][y];
        }
    }
    
    costmap_publisher_->publish(grid_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}