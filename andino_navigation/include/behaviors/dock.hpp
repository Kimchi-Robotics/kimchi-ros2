#pragma once

#include "andino_interfaces/action/dock.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace andino_navigation {
using DockAction = andino_interfaces::action::Dock;

/**
 * Nav2 custom behavior that sends a robot to a defined docking position.
 */
class Dock : public nav2_behaviors::TimedBehavior<DockAction> {
 public:
  using DockActionGoal = DockAction::Goal;
  using DockActionResult = DockAction::Result;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief A constructor for Dock
   */
  Dock();
  ~Dock();

  /**
   * @brief Initialization to run behavior
   *
   * @param command Goal to execute
   * @return Status of behavior
   */
  nav2_behaviors::Status onRun(
      const std::shared_ptr<const DockActionGoal> command) override;

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  nav2_behaviors::Status onCycleUpdate() override;

 private:
  enum class State { WAITING, RUNNING, REJECTED, CANCELED, ABORTED, SUCCEEDED };

  State state_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      navigate_to_pose_client_;

  /**
   * @brief Callback function for the goal response.
   *
   * @param result The response from the action server.
   */
  void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr& result);

  /**
   * @brief Callback function for the goal feedback.
   *
   * @param feedback The feedback of the goal.
   */
  void goalFeedbackCallback(
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  /**
   * @brief Callback function for the goal result.
   *
   * @param result The result from the action server.
   */
  void goalResultCallback(
      const GoalHandleNavigateToPose::WrappedResult& result);
};

}  // namespace andino_navigation
