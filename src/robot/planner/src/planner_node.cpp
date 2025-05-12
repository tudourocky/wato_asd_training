#include "planner_node.hpp"
#include <cmath>
#include <unordered_map>
#include <vector>
#include <queue>

// Main path planning node implementation
class PathPlanner : public rclcpp::Node {
private:
    // State machine for tracking planning status
    enum class PlanningState {
        IDLE,
        PLANNING_ACTIVE
    };

    // Core data structures
    struct GridPosition {
        int row;
        int col;
        
        GridPosition(int r = 0, int c = 0) : row(r), col(c) {}
        
        bool operator==(const GridPosition& other) const {
            return row == other.row && col == other.col;
        }

        bool operator!=(const GridPosition& other) const {
            return !(*this == other);
        }
    };

    struct GridPositionHash {
        size_t operator()(const GridPosition& pos) const {
            return std::hash<int>()(pos.row) ^ (std::hash<int>()(pos.col) << 1);
        }
    };

    struct SearchNode {
        GridPosition pos;
        double cost_so_far;
        double estimated_cost;
        double total_cost;
        GridPosition came_from;
        
        SearchNode() : pos(GridPosition(-1, -1)), cost_so_far(-1), 
                      estimated_cost(-1), total_cost(-1), 
                      came_from(GridPosition(-1, -1)) {}
                      
        SearchNode(GridPosition p, double cost, double est, double total, GridPosition from)
            : pos(p), cost_so_far(cost), estimated_cost(est), 
              total_cost(total), came_from(from) {}
    };

    // Node state
    PlanningState current_state_;
    nav_msgs::msg::OccupancyGrid environment_map_;
    geometry_msgs::msg::PointStamped target_position_;
    geometry_msgs::msg::Pose current_pose_;
    bool has_target_;
    robot::PlannerCore path_engine_;

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_listener_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_listener_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Constants
    static constexpr double POSITION_TOLERANCE = 0.5;
    static constexpr double UPDATE_INTERVAL_MS = 500.0;
    static constexpr double SCALE_FACTOR = 10.0;

public:
    PathPlanner() : Node("path_planner"), 
                   current_state_(PlanningState::IDLE),
                   has_target_(false),
                   path_engine_(this->get_logger()) {
        setupSubscriptions();
        setupPublishers();
        setupTimer();
    }

private:
    void setupSubscriptions() {
        map_listener_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                handleMapUpdate(msg);
            });
            
        goal_listener_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_point", 10, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                handleGoalUpdate(msg);
            });
            
        pose_listener_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                handlePoseUpdate(msg);
            });
    }

    void setupPublishers() {
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    }

    void setupTimer() {
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(UPDATE_INTERVAL_MS)),
            [this]() { periodicUpdate(); });
    }

    void handleMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg) {
        environment_map_ = *msg;
        if (current_state_ == PlanningState::PLANNING_ACTIVE) {
            computePath();
        }
    }

    void handleGoalUpdate(const geometry_msgs::msg::PointStamped::SharedPtr& msg) {
        target_position_ = *msg;
        has_target_ = true;
        current_state_ = PlanningState::PLANNING_ACTIVE;
        computePath();
    }

    void handlePoseUpdate(const nav_msgs::msg::Odometry::SharedPtr& msg) {
        current_pose_ = msg->pose.pose;
    }

    bool isTargetReached() const {
        double dx = target_position_.point.x - current_pose_.position.x;
        double dy = target_position_.point.y - current_pose_.position.y;
        return std::sqrt(dx * dx + dy * dy) < POSITION_TOLERANCE;
    }

    void periodicUpdate() {
        if (current_state_ == PlanningState::PLANNING_ACTIVE) {
            if (isTargetReached()) {
                RCLCPP_INFO(this->get_logger(), "Target position reached!");
                current_state_ = PlanningState::IDLE;
            } else {
                RCLCPP_INFO(this->get_logger(), "Recomputing path...");
                computePath();
            }
        }
    }

    double calculateDistance(double x1, double y1, double x2, double y2) const {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    void computePath() {
        if (!has_target_ || environment_map_.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot compute path: Missing map or target!");
            return;
        }

        // A* search implementation
        std::unordered_map<GridPosition, SearchNode, GridPositionHash> open_nodes;
        std::unordered_map<GridPosition, SearchNode, GridPositionHash> closed_nodes;

        // Initialize start node
        GridPosition start_pos(
            static_cast<int>(std::round((current_pose_.position.x + 20) * SCALE_FACTOR)),
            static_cast<int>(std::round((current_pose_.position.y + 20) * SCALE_FACTOR))
        );

        double initial_heuristic = calculateDistance(
            (current_pose_.position.x + 20) * SCALE_FACTOR,
            (current_pose_.position.y + 20) * SCALE_FACTOR,
            (target_position_.point.x + 20) * SCALE_FACTOR,
            (target_position_.point.y + 20) * SCALE_FACTOR
        );

        SearchNode start_node(
            start_pos,
            0.0,
            initial_heuristic,
            initial_heuristic + environment_map_.data[start_pos.row * environment_map_.info.width + start_pos.col] * 100,
            GridPosition(-1, -1)
        );

        open_nodes.emplace(start_pos, start_node);

        // Main A* loop
        while (!open_nodes.empty()) {
            // Find node with lowest total cost
            SearchNode current;
            for (const auto& [pos, node] : open_nodes) {
                if (current.total_cost == -1 || current.total_cost > node.total_cost) {
                    current = node;
                }
            }
            open_nodes.erase(current.pos);
            closed_nodes.emplace(current.pos, current);

            // Check if goal reached
            if (current.pos.row == static_cast<int>(std::round((target_position_.point.x + 20) * SCALE_FACTOR)) &&
                current.pos.col == static_cast<int>(std::round((target_position_.point.y + 20) * SCALE_FACTOR))) {
                break;
            }

            // Explore neighbors
            const int neighbor_offsets[8][2] = {
                {-1, 0}, {1, 0}, {0, -1}, {0, 1},
                {1, 1}, {1, -1}, {-1, -1}, {-1, 1}
            };

            for (const auto& offset : neighbor_offsets) {
                GridPosition neighbor_pos(current.pos.row + offset[0], current.pos.col + offset[1]);
                
                if (neighbor_pos.row >= 0 && 
                    static_cast<unsigned int>(neighbor_pos.row) < environment_map_.info.width &&
                    neighbor_pos.col >= 0 && 
                    static_cast<unsigned int>(neighbor_pos.col) < environment_map_.info.height) {
                    
                    if (closed_nodes.find(neighbor_pos) != closed_nodes.end()) {
                        continue;
                    }

                    double obstacle_cost = environment_map_.data[neighbor_pos.row * environment_map_.info.width + neighbor_pos.col] * 100;
                    double movement_cost = std::sqrt(offset[0] * offset[0] + offset[1] * offset[1]);
                    double new_cost = current.cost_so_far + movement_cost;
                    double heuristic = calculateDistance(
                        neighbor_pos.row,
                        neighbor_pos.col,
                        (target_position_.point.x + 20) * SCALE_FACTOR,
                        (target_position_.point.y + 20) * SCALE_FACTOR
                    );

                    SearchNode neighbor_node(
                        neighbor_pos,
                        new_cost,
                        heuristic,
                        new_cost + heuristic + obstacle_cost,
                        current.pos
                    );

                    if (open_nodes.find(neighbor_pos) == open_nodes.end() ||
                        open_nodes[neighbor_pos].total_cost > new_cost + heuristic) {
                        open_nodes.emplace(neighbor_pos, neighbor_node);
                    }
                }
            }
        }

        // Construct and publish path
        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "sim_world";
        
        GridPosition current_pos(
            static_cast<int>(std::round((target_position_.point.x + 20) * SCALE_FACTOR)),
            static_cast<int>(std::round((target_position_.point.y + 20) * SCALE_FACTOR))
        );

        if (closed_nodes.find(current_pos) == closed_nodes.end()) {
            path_publisher_->publish(path);
            return;
        }

        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        while (current_pos != GridPosition(-1, -1)) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = static_cast<double>(current_pos.row) / SCALE_FACTOR - 20;
            pose.pose.position.y = static_cast<double>(current_pos.col) / SCALE_FACTOR - 20;
            pose.pose.orientation.w = 1.0;
            pose.header.frame_id = "sim_world";
            waypoints.insert(waypoints.begin(), pose);
            current_pos = closed_nodes[current_pos].came_from;
        }

        path.poses = waypoints;
        path_publisher_->publish(path);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
