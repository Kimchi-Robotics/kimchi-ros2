#include "andino_bms/andino_bms.h"

#include <andino_interfaces/srv/set_battery.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/int32.hpp>

AndinoBMS::AndinoBMS(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("andino_bms", options),
      battery_level_(kStartBatteryLevel),
      battery_discharge_rate_(kBatteryDischargeRateStill),
      previous_time_secs_(0) {
  using namespace std::chrono_literals;

  // Create a QoS profile
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10),
                         qos_profile);
  qos.best_effort();

  // Publish battery level.
  battery_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
      "andino_bms/battery_level", qos);
  battery_publisher_timer_ = this->create_wall_timer(
      1s, std::bind(&AndinoBMS::batteryPublisherTimerCallback, this));
  RCLCPP_INFO(this->get_logger(),
              "Publishing battery information to andino_bms/battery_level");

  // Subscribe to clock topic.
  clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", qos,
      std::bind(&AndinoBMS::batteryDischargeCallback, this,
                std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to /clock");

  // Subscribe to vel topic.
  cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&AndinoBMS::cmdVelCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to /cmd_vel");

  // Service to set Battery level.
  set_battery_service_ =
      this->create_service<andino_interfaces::srv::SetBattery>(
          "/andino_bms/set_battery",
          std::bind(&AndinoBMS::setBattery, this, std::placeholders::_1,
                    std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "/andino_bms/set_battery service ready.");
}

void AndinoBMS::batteryPublisherTimerCallback() {
  auto message = std_msgs::msg::Int32();
  message.data = battery_level_;
  this->battery_publisher_->publish(message);
}

void AndinoBMS::batteryDischargeCallback(
    const rosgraph_msgs::msg::Clock::SharedPtr msg) {
  int current_time_secs = msg->clock.sec;

  // Initialize previous_time_secs_ the first time this method is called.
  if (previous_time_secs_ == 0) {
    previous_time_secs_ = current_time_secs;
  }

  int diff_time_secs = current_time_secs - previous_time_secs_;
  previous_time_secs_ = current_time_secs;

  battery_level_ -= battery_discharge_rate_ * diff_time_secs;

  if (battery_level_ < 0) {
    battery_level_ = 0;
  }
}

void AndinoBMS::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  int battery_discharge_rate;
  if (msg->linear.x != 0 || msg->angular.z != 0) {
    battery_discharge_rate = kBatteryDischargeRateMoving;
  } else {
    battery_discharge_rate = kBatteryDischargeRateStill;
  }

  if (battery_discharge_rate_ != battery_discharge_rate) {
    battery_discharge_rate_ = battery_discharge_rate;
  }
}

void AndinoBMS::setBattery(
    const andino_interfaces::srv::SetBattery::Request::SharedPtr request,
    andino_interfaces::srv::SetBattery::Response::SharedPtr /*response*/) {
  battery_level_ = request->battery_level;
}
