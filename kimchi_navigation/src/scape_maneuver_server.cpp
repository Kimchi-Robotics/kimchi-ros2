#include "kimchi_navigation/scape_maneuver_server.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


double calculateDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2) {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

ScapeManeuver::ScapeManeuver(std::shared_ptr<rclcpp::Node> node) : node_(node) {

    odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&ScapeManeuver::OdomCallback, this, std::placeholders::_1));

    auto qos_profile =
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_t{
          RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          10,  // depth
          RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
          RMW_QOS_POLICY_DURABILITY_VOLATILE, RMW_QOS_DEADLINE_DEFAULT,
          RMW_QOS_LIFESPAN_DEFAULT, RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
          RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
          false  // avoid_ros_namespace_conventions
      }));
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos_profile);

}

void ScapeManeuver::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Get current pose
    current_pose_ = msg->pose.pose;
}

void ScapeManeuver::InitializeScapeManeuver(std::vector<ObstaclesPosition> obstacles) {

    if(std::find(obstacles.begin(), obstacles.end(), ObstaclesPosition::SECTOR_1) != obstacles.end()) {
        scape_direction_ = 1;
    } else if (std::find(obstacles.begin(), obstacles.end(), ObstaclesPosition::SECTOR_2) != obstacles.end()){
        scape_direction_ = 4;
    } else if (std::find(obstacles.begin(), obstacles.end(), ObstaclesPosition::SECTOR_3) != obstacles.end()) {
        scape_direction_ = 4;
    } else {
        scape_direction_ = 1;
    }

    geometry_msgs::msg::Twist twist_msg;
    if (scape_direction_ == 1) {
        twist_msg.linear.x = 0.4;
        cmd_vel_pub_->publish(twist_msg);
        auto waiting_time = std::chrono::seconds(1);
        std::this_thread::sleep_for(waiting_time);
    } else if (scape_direction_ == 2) {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.angular.z = 0.3;
        cmd_vel_pub_->publish(twist_msg);
        auto waiting_time = std::chrono::milliseconds(2618);
        std::this_thread::sleep_for(waiting_time);
        twist_msg.angular.z = 0.0;
        twist_msg.linear.x = 0.4;
        cmd_vel_pub_->publish(twist_msg);
        waiting_time = std::chrono::seconds(1);
        std::this_thread::sleep_for(waiting_time);
    } else if (scape_direction_ == 3) {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.angular.z = -0.3;
        cmd_vel_pub_->publish(twist_msg);
        auto waiting_time = std::chrono::milliseconds(2618);
        std::this_thread::sleep_for(waiting_time);
        twist_msg.angular.z = 0.0;
        twist_msg.linear.x = 0.4;
        cmd_vel_pub_->publish(twist_msg);
        waiting_time = std::chrono::seconds(1);
        std::this_thread::sleep_for(waiting_time);
    } else if (scape_direction_ == 4) {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = -0.4;
        cmd_vel_pub_->publish(twist_msg);
        auto waiting_time = std::chrono::seconds(1);
        std::this_thread::sleep_for(waiting_time);
    }

    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(twist_msg);
}
