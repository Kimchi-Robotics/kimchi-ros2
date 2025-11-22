#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"


enum ObstaclesPosition {
    SECTOR_1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4
};

class ScapeManeuver
{
public:
    ScapeManeuver(std::shared_ptr<rclcpp::Node> node);
    void InitializeScapeManeuver(std::vector<ObstaclesPosition> obstacles);
    bool IsDone();
    void ScapeLoop();

private:
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    double target_angle_ = 0.0;
    geometry_msgs::msg::Point start_pos_;

    geometry_msgs::msg::Pose current_pose_;

    // Constants for control
    const double kEscapedDistance = 0.35; // meters
    const double kLinearVelocity = 0.2; // m/s
    const double kAngularVelocity = 0.1; // rad/s
    const double kYawTolerance = 0.05; // radians
    int scape_direction_{0};

};
