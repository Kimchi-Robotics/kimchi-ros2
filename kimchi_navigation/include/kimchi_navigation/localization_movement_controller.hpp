#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cmath>
#include <chrono>
#include <string>
#include <memory>

using namespace std::chrono_literals;

class LocalizationMovementController : public rclcpp::Node
{
public:
  LocalizationMovementController();
  ~LocalizationMovementController();

private:
  void executeMovementPattern();

  // Publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Timer for executing movement pattern
  rclcpp::TimerBase::SharedPtr timer_;

// TODO: lola: subscribe to a topic that indicates when to stop the movement
// the initial pose estimator is going to publish on this topic
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr particle_sub_;

  // Parameters
  double linear_speed_;
  double angular_speed_;
  std::string pattern_;

  // State variables
  rclcpp::Time start_time_;
};
