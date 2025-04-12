#pragma once
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cstdint>
#include <geometry_msgs/msg/pose2_d.hpp>


struct MapInfo {
    MapInfo(float resolution, const geometry_msgs::msg::Pose2D& origin_, const std::vector<long int>& image ): resolution_(resolution), origin_(origin_), image_(image) {}

    float resolution_;
    geometry_msgs::msg::Pose2D origin_;
    std::vector<long int> image_; // Assuming image is a byte array
};
