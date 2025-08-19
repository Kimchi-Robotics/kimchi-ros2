#pragma once

#include <lifecycle_msgs/srv/change_state.hpp>
#include <memory>
#include <nav2_lifecycle_manager/lifecycle_manager_client.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "point_2d.h"

#include "kimchi_navigation/global_localization.hpp"

/**
 * Class to manage navigation-related functionalities.
 * It handles:
 * - starting and stopping SLAM, and localization nav2 nodes
 * - Localization around a point.
 * - Path setting.
 */
class NavigationManager {
 public:
  /**
   * Interface for observing mission events.
   * This interface allows the observer to be notified about mission-related
   * events, such as starting navigation, reaching goals, and finishing
   * missions.
   */
  class MissionObserver {
   public:
    /**
     * Called when the robot starts a new mission.
     * @param path The initial path for the mission.
     */
    virtual void onNavigatingToGoal(const Point2D& point) = 0;

    /**
     * Called when the robot reaches a goal point.
     * @param point The point that was reached.
     */
    virtual void onGoalReached(const Point2D& point) = 0;

    /**
     * Called when the path is finished, e.g., when the robot reaches the goal.
     */
    virtual void onMissionFinished() = 0;
  };

  NavigationManager(std::shared_ptr<rclcpp::Node> node,
                    std::shared_ptr<MissionObserver> observer);

  ~NavigationManager() = default;

  void startSlam();
  void stopSlam();
  bool startLocating(const Point2D& point);
  void startNavigation();
  void stopNavigation();

  void addGoalToMission(const Point2D& point);
  void goToNextGoal();

  // Localizes the robot around a given pose.
  void localizeAround(const Point2D& point);

 private:
  using GlobalLocalization = kimchi_interfaces::action::Localizing;
  using GoalHandleGlobalLocalization =
      rclcpp_action::ClientGoalHandle<GlobalLocalization>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;
  void onNewGoal();

  void localizeGoalResponseCallback(
      GoalHandleGlobalLocalization::SharedPtr goal_handle);
  void localizeFeedbackCallback(
    GoalHandleGlobalLocalization::SharedPtr,
    const std::shared_ptr<const GlobalLocalization::Feedback> feedback);
  void localizeResultCallback(const GoalHandleGlobalLocalization::WrappedResult& result);

  void navigateToPoseGoalResponseCallback(
      GoalHandleNavigateToPose::SharedPtr goal_handle);
  void navigateToPoseResultCallback(
      const GoalHandleNavigateToPose::WrappedResult& result);

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MissionObserver> mission_observer_;
  std::unique_ptr<Point2D> current_goal_;
  std::queue<Point2D> goals_;

  rclcpp_action::Client<GlobalLocalization>::SharedPtr
      global_localization_action_client_ptr_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr
      navigate_to_pose_action_client_ptr_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr
      active_slam_toolbox_node_client_;

  std::unique_ptr<nav2_lifecycle_manager::LifecycleManagerClient>
      client_localization_;

  std::atomic<bool> robot_localized_{false};
};
