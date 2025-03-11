#pragma once

#include <std_msgs/msg/int32.hpp>

#include "andino_interfaces/action/dock.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

/**
 * ROS 2 node that subscribes to a battery topic and sends the robot to charge
 * when the bettery level is below a certain point.
 */
class BatteryObserver : public rclcpp::Node {
 public:
  BatteryObserver(const rclcpp::NodeOptions& options);

 private:
  static constexpr int kBatteryThreshold = 30;

  using Dock = andino_interfaces::action::Dock;
  using GoalHandleDock = rclcpp_action::ClientGoalHandle<Dock>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;

  bool docking_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr battery_subscriber_;
  rclcpp_action::Client<Dock>::SharedPtr dock_client_;

  /**
   * Callback function for the battery topic.
   *
   * @param msg The message received from the battery topic.
   */
  void batteryCallback(const std_msgs::msg::Int32::SharedPtr msg);

  /**
    * Sends the robot to charge by calling the docking action.
   */
  void sendGoal();

  /**
   * Callback function for the goal response.
   *
   * @param result The response from the action server.
   */
  void goalResponseCallback(const GoalHandleDock::SharedPtr& result);

  /**
   * Callback function for the goal feedback.
   *
   * @param feedback The feedback of the goal.
   */
  void feedbackCallback(GoalHandleDock::SharedPtr,
                        const std::shared_ptr<const Dock::Feedback> feedback);


  /**
    * Callback function for the goal result.
    *
    * @param result The result after the action is finished.
    */
  void resultCallback(const GoalHandleDock::WrappedResult& goal_handle);
};

RCLCPP_COMPONENTS_REGISTER_NODE(BatteryObserver)
