/**
 * Kimchi State Server
 */
#pragma once

#include <atomic>
#include <kimchi_interfaces/msg/robot_state.hpp>
#include <kimchi_interfaces/srv/map_info.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <nav2_msgs/srv/save_map.hpp>

#include "map_info.h"

#include <nav2_lifecycle_manager/lifecycle_manager_client.hpp>

enum class RobotState {
  NO_MAP,
  MAPPING_WITH_EXPLORATION,
  MAPPING_WITH_TELEOP,
  NAVIGATING,
  TELEOP,
  IDLE
};

/**
 * ROS 2 node to handle and publish the state of the robot.
 */
class KimchiStateServer : public rclcpp::Node {
 public:
  KimchiStateServer(const rclcpp::NodeOptions& options);

 private:
  void statePublisherTimerCallback();
  void callGetMapInfoService();
  void startSlamCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void startNavigationCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void saveMap();
  void stopSlam();
  void startNavigation();

    // The client used to control the nav2 stack
  std::unique_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_nav_;
  std::unique_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_loc_;

  rclcpp::TimerBase::SharedPtr state_publisher_timer_;
  rclcpp::Publisher<kimchi_interfaces::msg::RobotState>::SharedPtr
      state_publisher_;

  // TODO Service to start mapping.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_slam_service_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr
      active_slam_toolbox_node_client_;
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_client_;

  // TODO Service to start navigation.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_navigation_service_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr
      active_navigation_node_client_;

  rclcpp::Client<kimchi_interfaces::srv::MapInfo>::SharedPtr
      get_map_info_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<RobotState> state_;

  std::unique_ptr<MapInfo> map_info_;
};
