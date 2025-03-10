#pragma once

#include <andino_interfaces/srv/set_battery.hpp>
#include <atomic>
#include <geometry_msgs/msg/twist.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/int32.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

/**
 * ROS 2 node to Mock the battery behavior of Andino.
 */
class AndinoBMS : public rclcpp::Node {
 public:
  AndinoBMS(const rclcpp::NodeOptions& options);

 private:
  static constexpr int kStartBatteryLevel = 100;
  static constexpr int kBatteryDischargeRateMoving = 4;
  static constexpr int kBatteryDischargeRateStill = 1;

  std::atomic<int> battery_level_;
  std::atomic<int> battery_discharge_rate_;
  int previous_time_secs_;

  rclcpp::TimerBase::SharedPtr battery_publisher_timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr battery_publisher_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscriber_;

  rclcpp::Service<andino_interfaces::srv::SetBattery>::SharedPtr
      set_battery_service_;

  /**
   * Callback function for the cmd_vel topic.
   *
   * @param msg The message received from the cmd_vel topic.
   */
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * Callback function for the clock topic.
   *
   * @param msg The message received from the clock topic.
   */
  void batteryDischargeCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

  /**
   * Callback function for the battery publisher timer.
   */
  void batteryPublisherTimerCallback();

  /**
   * Service to set the battery level.
   *
   * @param request The request to set the battery level.
   * @param response Unused.
   */
  void setBattery(
      const andino_interfaces::srv::SetBattery::Request::SharedPtr request,
      andino_interfaces::srv::SetBattery::Response::SharedPtr response);
};

RCLCPP_COMPONENTS_REGISTER_NODE(AndinoBMS)
