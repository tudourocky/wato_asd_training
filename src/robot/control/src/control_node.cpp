#include "control_node.hpp"
#include <cmath>
#include <optional>

PurePursuitController::PurePursuitController() : Node("pure_pursuit_controller") {
    // Configuration parameters
    ld_ = 1.0;      // Look-ahead distance in meters
    goal_threshold_ = 0.5;  // Goal reaching threshold
    v_max_ = 0.5;   // Maximum linear velocity

    // Setup ROS communication
    setupSubscribers();
    setupPublishers();
    setupTimer();
}

void PurePursuitController::setupSubscribers() {
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, 
        [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; }
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { current_odom_ = msg; }
    );
}

void PurePursuitController::setupPublishers() {
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void PurePursuitController::setupTimer() {
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { executeControlLoop(); }
    );
}

std::optional<geometry_msgs::msg::PoseStamped> PurePursuitController::getLookaheadPoint() {
    if (!current_path_ || current_path_->poses.empty()) {
        return std::nullopt;
    }

    for (const auto& pose : current_path_->poses) {
        double dist = calculateDistance(pose.pose, current_odom_->pose.pose);
        if (std::abs(dist - ld_) <= 0.1) {
            return pose;
        }
    }

    // If no point found, use the last point
    auto last_point = current_path_->poses.back();
    if (calculateDistance(last_point.pose, current_odom_->pose.pose) <= goal_threshold_) {
        return std::nullopt;
    }
    return last_point;
}

geometry_msgs::msg::Point PurePursuitController::convertToLocalFrame(
    const geometry_msgs::msg::Point& global_pt,
    const geometry_msgs::msg::Pose& robot_pose
) {
    double theta = getYawFromQuaternion(robot_pose.orientation);
    double c = std::cos(theta);
    double s = std::sin(theta);

    geometry_msgs::msg::Point local_pt;
    local_pt.x = c * (global_pt.x - robot_pose.position.x) + 
                s * (global_pt.y - robot_pose.position.y);
    local_pt.y = -s * (global_pt.x - robot_pose.position.x) + 
                 c * (global_pt.y - robot_pose.position.y);
    local_pt.z = 0.0;
    return local_pt;
}

geometry_msgs::msg::Twist PurePursuitController::calculateControlInput(
    const geometry_msgs::msg::PoseStamped& target
) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v_max_;

    // Transform target to robot frame
    auto local_target = convertToLocalFrame(target.pose.position, current_odom_->pose.pose);
    
    // Calculate steering angle
    double alpha = std::atan2(local_target.y, local_target.x);
    double ld = std::sqrt(local_target.x * local_target.x + local_target.y * local_target.y);
    
    // Pure pursuit control law
    double k = 1.0;  // Control gain
    cmd.angular.z = k * (2.0 * v_max_ * std::sin(alpha)) / ld;

    return cmd;
}

double PurePursuitController::calculateDistance(
    const geometry_msgs::msg::Pose& p1,
    const geometry_msgs::msg::Pose& p2
) {
    double dx = p1.position.x - p2.position.x;
    double dy = p1.position.y - p2.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

double PurePursuitController::getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

void PurePursuitController::executeControlLoop() {
    if (!current_path_ || !current_odom_) {
        return;
    }

    auto target = getLookaheadPoint();
    if (!target) {
        // Stop the robot if no valid target
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        vel_pub_->publish(stop_cmd);
        return;
    }

    auto cmd = calculateControlInput(*target);
    vel_pub_->publish(cmd);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}
