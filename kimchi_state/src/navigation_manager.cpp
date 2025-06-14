#include "kimchi_state/navigation_manager.h"

#include <chrono>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <memory>
#include <nav2_lifecycle_manager/lifecycle_manager_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

NavigationManager::NavigationManager(std::shared_ptr<rclcpp::Node> node)
    : node_(node) {
  RCLCPP_INFO(node_->get_logger(), "NavigationManager initialized.");
  // Additional initialization if needed

  active_slam_toolbox_node_client_ =
      node_->create_client<lifecycle_msgs::srv::ChangeState>(
          "/slam_toolbox/change_state");

  client_localization_ =
      std::make_unique<nav2_lifecycle_manager::LifecycleManagerClient>(
          "lifecycle_manager_localization", node_);
}

void NavigationManager::startSlam() {
  RCLCPP_INFO(node_->get_logger(), "Waiting for slam_toolbox service");
  active_slam_toolbox_node_client_->wait_for_service();
  RCLCPP_INFO(node_->get_logger(), "Finished waiting for slam_toolbox service");
  auto new_request =
      std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  new_request->transition.id = 3;  // Activate
  active_slam_toolbox_node_client_->async_send_request(new_request);
  RCLCPP_INFO(node_->get_logger(), "Starting SLAM");
}

void NavigationManager::stopSlam() {
  active_slam_toolbox_node_client_->wait_for_service();
  auto new_request =
      std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  new_request->transition.id = 4;  // Deactivate
  // TODO(arilow): Handle the response by passing a callback to
  // async_send_request
  active_slam_toolbox_node_client_->async_send_request(new_request);
}

void NavigationManager::startNavigation() {
  std::chrono::milliseconds wait_duration(100);
  std::thread startup_loc_thread(
      std::bind(&nav2_lifecycle_manager::LifecycleManagerClient::startup,
                client_localization_.get(), std::placeholders::_1),
      wait_duration  // Direct argument instead of placeholder
  );

  startup_loc_thread.detach();
}

void NavigationManager::stopNavigation() {}