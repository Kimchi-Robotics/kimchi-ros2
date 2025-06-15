/**
 * Kimchi State Server
 */
#pragma once

#include <atomic>
#include <kimchi_interfaces/msg/robot_state.hpp>
#include <kimchi_interfaces/srv/add_goal_to_mission.hpp>
#include <kimchi_interfaces/srv/map_info.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <memory>
#include <nav2_lifecycle_manager/lifecycle_manager_client.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "map_info.h"
#include "navigation_manager.h"

enum class RobotState {
  NO_MAP,
  MAPPING_WITH_EXPLORATION,
  MAPPING_WITH_TELEOP,
  NAVIGATING,
  LOCATING,
  TELEOP,
  IDLE,
  LOST,
  RECOVERING,
  GOAL_REACHED,
  CHARGING
};

/**
 * ROS 2 node to handle and publish the state of the robot.
 */
class KimchiStateServer
    : public NavigationManager::MissionObserver,
      public std::enable_shared_from_this<KimchiStateServer> {
 public:
  static std::shared_ptr<KimchiStateServer> Create(
      const rclcpp::NodeOptions& options);

  std::shared_ptr<rclcpp::Node> getNode() const {
    return node_->shared_from_this();
  }

  // MissionObserver implementted methods.
  void onNavigatingToGoal(const Point2D& point) override;
  void onGoalReached(const Point2D& point) override;
  void onMissionFinished() override;

 private:
  /**
   * The constructor must be private and instances of this class must be created
   * using Created() method. This is to call initialize() method after the
   * object is created.
   */
  explicit KimchiStateServer(const rclcpp::NodeOptions& options);

  /**
   * Initializes attributes that require the object to be fully constructed to
   * be passed to it.
   */
  void initialize();

  void statePublisherTimerCallback();
  void callGetMapInfoService();
  void saveMap();
  void changeState(RobotState new_state);

  // Callback methods for the services.
  void startSlamCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);
  void startNavigationCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);
  void addGoalToMissionCallback(
      const kimchi_interfaces::srv::AddGoalToMission::Request::SharedPtr
          request,
      kimchi_interfaces::srv::AddGoalToMission::Response::SharedPtr response);

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<NavigationManager> navigation_manager_;
  std::atomic<RobotState> state_;
  std::unique_ptr<MapInfo> map_info_;

  // Topics.
  rclcpp::TimerBase::SharedPtr state_publisher_timer_;
  rclcpp::Publisher<kimchi_interfaces::msg::RobotState>::SharedPtr
      state_publisher_;

  // Service servers.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_slam_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_navigation_service_;
  rclcpp::Service<kimchi_interfaces::srv::AddGoalToMission>::SharedPtr
      add_goal_to_mission_service_;

  // Service clients.
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_client_;
  rclcpp::Client<kimchi_interfaces::srv::MapInfo>::SharedPtr
      get_map_info_client_;
};
