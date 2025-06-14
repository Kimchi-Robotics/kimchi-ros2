#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <nav2_lifecycle_manager/lifecycle_manager_client.hpp>

/**
 * Class to manage navigation-related functionalities.
 * It handles:
 * - starting and stopping SLAM, and localization nav2 nodes
 * - Localization around a point.
 * - Path setting.
 */
class NavigationManager {
 public:
  NavigationManager(std::shared_ptr<rclcpp::Node> node);
  ~NavigationManager() = default;

  void startSlam();
  void stopSlam();
  void startNavigation();
  void stopNavigation();

  // Localizes the robot around a given pose.
  void localizeAround(/*pose2D*/);

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<nav2_lifecycle_manager::LifecycleManagerClient>
      client_localization_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr
    active_slam_toolbox_node_client_;
};
