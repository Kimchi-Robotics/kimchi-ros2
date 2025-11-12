#include "kimchi_navigation/scape_manuver_server.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


double calculateDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2) {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

ScapeManuver::ScapeManuver(std::shared_ptr<rclcpp::Node> node) : node_(node) {

    odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&ScapeManuver::OdomCallback, this, std::placeholders::_1));

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

void ScapeManuver::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Get current pose
    current_pose_ = msg->pose.pose;
}

void ScapeManuver::InitializeScapeManuver(std::vector<ObstaclesPosition> obstacles) {
        // double angle;
    // if (obstacles.size() > 2) {
    //     RCLCPP_ERROR(node_->get_logger(), "[Lola] Robot trapped. Wait for operator.");
    //     return;
    // } else if (obstacles.size() == 2) {
    //     if (obstacles.count(ObstaclesPosition::SECTOR_1)) {
    //         if (obstacles.count(ObstaclesPosition::SECTOR_2)) {
    //             std::vector<double> angles = obstacles[ObstaclesPosition::SECTOR_2];
    //             auto it = std::find_if(angles.begin(), angles.end(),
    //                     [](double angle) {
    //                         return angle > (3 * M_PI / 4.0);
    //                     });
    //             if (it != angles.end()) {
    //                 RCLCPP_ERROR(node_->get_logger(), "[Lola] Robot trapped.");
    //             } else {
    //                 scape_direction_ = 1;
    //             }
    //         } else if (obstacles.count(ObstaclesPosition::SECTOR_3)) {
    //             scape_direction_ = 2;
    //         } else {
    //             scape_direction_ = 1;
    //         }
    //     } else if (obstacles.count(ObstaclesPosition::SECTOR_2)) {
    //         if(obstacles.count(ObstaclesPosition::SECTOR_4)) {
    //             std::vector<double> angles = obstacles[ObstaclesPosition::SECTOR_4];
    //             auto it = std::find_if(angles.begin(), angles.end(),
    //                     [](double angle) {
    //                         return angle > (7 * M_PI / 4.0);
    //                     });
    //             if (it != angles.end()) {
    //                 scape_direction_ = 4;
    //             }
    //         } else {
    //             scape_direction_ = 3;
    //         }
    //     } else if (obstacles.count(ObstaclesPosition::SECTOR_3)) {
    //         if(obstacles.count(ObstaclesPosition::SECTOR_4)) {
    //             std::vector<double> angles_s4 = obstacles[ObstaclesPosition::SECTOR_4];
    //             auto it_8 = std::find_if(angles_s4.begin(), angles_s4.end(),
    //                     [](double angle) {
    //                         return angle > (7 * M_PI / 4.0);
    //                     });

    //             std::vector<double> angles_s3 = obstacles[ObstaclesPosition::SECTOR_3];
    //             auto it_6 = std::find_if(angles_s3.begin(), angles_s3.end(),
    //                     [](double angle) {
    //                         return angle > (5 * M_PI / 4.0);
    //                     });
    //             auto it_5 = std::find_if(angles_s3.begin(), angles_s3.end(),
    //                     [](double angle) {
    //                         return angle > (M_PI);
    //                     });

    //             if(it_8 != angles_s4.end() && it_5 != angles_s3.end())
    //             {
    //                 RCLCPP_ERROR(node_->get_logger(), "[Lola] Robot trapped. Wait for operator.");

    //             } else if(it_8 != angles_s4.end() && it_6 != angles_s3.end())
    //             {
    //                 scape_direction_ = 1;
    //             } else {
    //                 scape_direction_ = 3;
    //             }
    //         }
    //     } else {
    //         scape_direction_ = 1;
    //     }
    // } else {
    //     if(auto it = obstacles.find(ObstaclesPosition::SECTOR_1);
    //         it != obstacles.end()) {
    //         scape_direction_ = 1;
    //     } else if (auto it = obstacles.find(ObstaclesPosition::SECTOR_2);
    //         it != obstacles.end()){
    //         scape_direction_ = 4;
    //     } else if (auto it = obstacles.find(ObstaclesPosition::SECTOR_3);
    //         it != obstacles.end()) {
    //         scape_direction_ = 4;
    //     } else {
    //         scape_direction_ = 1;
    //     }
    // }

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
