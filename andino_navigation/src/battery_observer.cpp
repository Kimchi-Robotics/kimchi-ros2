#include "battery_observer/battery_observer.h"

#include "rclcpp/rclcpp.hpp"

BatteryObserver::BatteryObserver(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("battery_observer", options), docking_(false) {
  // Create a QoS profile
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10),
                         qos_profile);
  qos.best_effort();

  // Subscribe to battery topic.
  battery_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
      "andino_bms/battery_level", qos,
      std::bind(&BatteryObserver::batteryCallback, this,
                std::placeholders::_1));

  dock_client_ = rclcpp_action::create_client<Dock>(this, "/dock");
}

void BatteryObserver::batteryCallback(
    const std_msgs::msg::Int32::SharedPtr msg) {
  if (docking_) {
    return;  // already docking.
  }

  if (msg->data < kBatteryThreshold) {
    if (!dock_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(),
                "Battery level is below 90. Sending robot to charge.");
    sendGoal();
  }

  return;
}

void BatteryObserver::sendGoal() {
  using namespace std::placeholders;
  auto goal_msg = Dock::Goal();
  goal_msg.dock_pose.x = 1;
  goal_msg.dock_pose.y = 3;
  goal_msg.dock_pose.theta = 3.14;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<Dock>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&BatteryObserver::goalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&BatteryObserver::feedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&BatteryObserver::resultCallback, this, _1);
  dock_client_->async_send_goal(goal_msg, send_goal_options);

  docking_ = true;
  return;
}

void BatteryObserver::goalResponseCallback(
    const GoalHandleDock::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
  return;
}

void BatteryObserver::feedbackCallback(
    GoalHandleDock::SharedPtr,
    const std::shared_ptr<const Dock::Feedback> feedback) {
  std::stringstream ss;
  ss << "Current position is: ";
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  return;
}

void BatteryObserver::resultCallback(
    const GoalHandleDock::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Reached goal");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  return;
}
