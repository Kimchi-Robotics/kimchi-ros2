#pragma once
#include <cstdint>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

struct MapInfo {
  MapInfo(float resolution, const geometry_msgs::msg::Pose2D& origin_,
          const std::vector<uint8_t>& image)
      : resolution_(resolution), origin_(origin_), image_(image) {}

  float resolution_;
  geometry_msgs::msg::Pose2D origin_;
  std::vector<uint8_t> image_;  // Assuming image is a byte array
};
