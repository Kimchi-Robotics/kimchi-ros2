#pragma once

#include <atomic>
#include <chrono>
#include <memory>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "kimchi_interfaces/action/localizing.hpp"
#include "kimchi_interfaces/srv/scape_maneuver.hpp"
#include "scape_manuver_server.hpp"


enum LocalizationState {
    LOOKING_FOR_OBSTACLES,
    SCAPING_MANEUVER,
    LOCALIZING
};


/**
 * Node to obtain the global localization of the robot.
 *
 * Subscribers
 * - /initial_pose_estimate : initial pose estimation marked by te user near the
 * real position of the robot
 *
 * Publisher
 * - /initialpose: publishes the initial pose of the robot
 */
class GlobalLocalizationServer {
 public:
  using GlobalLocalization = kimchi_interfaces::action::Localizing;
  using GoalHandleGlobalLocalization =
      rclcpp_action::ServerGoalHandle<GlobalLocalization>;
  GlobalLocalizationServer();

  std::shared_ptr<rclcpp::Node> getNode() const {
    return node_->shared_from_this();
  }

 private:
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const GlobalLocalization::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<GoalHandleGlobalLocalization> goal_handle);
  void handleAccepted(
      const std::shared_ptr<GoalHandleGlobalLocalization> goal_handle);

//   void InitializeScapeManuver();

  /**
   * Managed the workflow of the action.
   * 1 - Verifies if there was a canceleation request
   * 2 - Publish Feedback
   * 3 - Publish success state
   */
  void execute(const std::shared_ptr<GoalHandleGlobalLocalization> goal_handle);

  /**
   * Publish the initial pose to nav2.
   */
  void PublishInitialPoseWithHighVariance();

  /**
   * Set the localized state and stop the robot movement.
   */
  void RobotLocalized();

  /**
   * Rotate the robot with a constant velocity of 1 rad/s.
   */
  void RotateRobot();

  /**
   * Stop robot rotation.
   */
  void StopRobot();

  /**
   * Handle clean up setting the node to it's initial state
   */
  void cleanup();

  /**
   * AMCL pose callback
   * Callback method for pose published by AMCL.
   *
   * Verifies if the robot is localized by checking if the covariance of the
   * estimated amcl pose is within a set threshold
   */
  void AmclPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void CheckForObstacles();

  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      amcl_pose_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_robot_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;

  rclcpp_action::Server<GlobalLocalization>::SharedPtr action_server_;

  std::unique_ptr<ScapeManuver> scape_manuver_;

  float inital_pose_estimate_x_;
  float inital_pose_estimate_y_;
  geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;

  bool initial_pose_published_{false};
  std::atomic<bool> robot_localized_{false};
  double pos_uncertainty_threashold_;
  double orientation_uncertainty_threashold_;
  std::atomic<double> current_position_uncertainty_;
  std::atomic<double> current_orientation_uncertainty_;
  double obstacle_angle_{};
  double min_range_{};
  double kSafetyDistance{0.5};
  std::mutex mutex_;

  LocalizationState localization_state_{LocalizationState::LOOKING_FOR_OBSTACLES};
  std::vector<ObstaclesPosition> obstacles_;

  sensor_msgs::msg::LaserScan::SharedPtr lidar_reading_{nullptr};
};
