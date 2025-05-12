#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <optional>

/**
 * @brief Pure Pursuit Controller for path following
 * 
 * This class implements a pure pursuit controller for following a path.
 * It uses a look-ahead point on the path to generate velocity commands
 * that steer the robot towards the path.
 */
class PurePursuitController : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     * 
     * Initializes the controller with default parameters and sets up
     * ROS subscribers, publishers, and timer.
     */
    PurePursuitController();

private:
    // ROS communication setup methods
    void setupSubscribers();
    void setupPublishers();
    void setupTimer();

    // Core control methods
    void executeControlLoop();
    std::optional<geometry_msgs::msg::PoseStamped> getLookaheadPoint();
    geometry_msgs::msg::Twist calculateControlInput(const geometry_msgs::msg::PoseStamped& target);
    
    // Utility methods
    geometry_msgs::msg::Point convertToLocalFrame(
        const geometry_msgs::msg::Point& global_pt,
        const geometry_msgs::msg::Pose& robot_pose
    );
    double calculateDistance(
        const geometry_msgs::msg::Pose& p1,
        const geometry_msgs::msg::Pose& p2
    );
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

    // ROS communication members
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // State variables
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    
    // Controller parameters
    double ld_;              // Look-ahead distance
    double goal_threshold_;  // Goal threshold
    double v_max_;          // Maximum velocity
};

#endif
