/**
 * Kimchi State Server
 */
#pragma once

#include <atomic>
#include <future>  // For std::promise and std::future
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <kimchi_interfaces/msg/robot_state.hpp>
#include <kimchi_interfaces/srv/kimchi_state_server_command.hpp>
#include <kimchi_interfaces/srv/map_info.hpp>
#include <kimchi_interfaces/srv/proccess_selected_position.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <memory>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <nav2_lifecycle_manager/lifecycle_manager_client.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <kimchi_interfaces/msg/robot_state.hpp>
#include <kimchi_interfaces/srv/add_goal_to_mission.hpp>
#include <kimchi_interfaces/srv/map_info.hpp>
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

  // MissionObserver implemented methods.
  void onNavigatingToGoal(const Point2D& point) override;
  void onGoalReached(const Point2D& point) override;
  void onMissionFinished() override;
  void onGoalCancelled(const Point2D& point) override;

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
  void checkGlobalLocalizationCallback();
  void callGetMapInfoService();
  std::shared_future<nav2_msgs::srv::SaveMap::Response::SharedPtr> saveMap();
  void changeState(RobotState new_state);
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
  SetMapFileName();
  void startNavigation();

  // Callback methods for the services.
  void startSlamCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);
  void initialPoseCallback(
      const kimchi_interfaces::srv::ProccessSelectedPosition::Request::SharedPtr
          request,
      kimchi_interfaces::srv::ProccessSelectedPosition::Response::SharedPtr
          response);
  void startNavigationCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);
  void addGoalToMissionCallback(
      const kimchi_interfaces::srv::ProccessSelectedPosition::Request::SharedPtr
          request,
      kimchi_interfaces::srv::ProccessSelectedPosition::Response::SharedPtr response);
  void sendCommandCallback(
      const kimchi_interfaces::srv::KimchiStateServerCommand::Request::SharedPtr
          request,
      kimchi_interfaces::srv::KimchiStateServerCommand::Response::SharedPtr
          response);

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<NavigationManager> navigation_manager_;
  std::atomic<RobotState> state_;
  std::unique_ptr<MapInfo> map_info_;

  enum class LocalizationState { PENDING, LOCATING, SUCCESS, FAILED };
  std::atomic<LocalizationState> robot_localize_state_{LocalizationState::PENDING};
  std::optional<geometry_msgs::msg::Pose> inital_pose_estimate_;

  // Topics.
  rclcpp::TimerBase::SharedPtr state_publisher_timer_;
  rclcpp::TimerBase::SharedPtr global_localization_timer_;
  rclcpp::Publisher<kimchi_interfaces::msg::RobotState>::SharedPtr
      state_publisher_;

  // Service servers.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_slam_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_navigation_service_;
  rclcpp::Service<kimchi_interfaces::srv::ProccessSelectedPosition>::SharedPtr
      start_locating_service_;
  rclcpp::Service<kimchi_interfaces::srv::ProccessSelectedPosition>::SharedPtr
      add_goal_to_mission_service_;
  rclcpp::Service<kimchi_interfaces::srv::KimchiStateServerCommand>::SharedPtr
      send_command_service_;

  // Service clients.
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_client_;
  rclcpp::Client<kimchi_interfaces::srv::MapInfo>::SharedPtr
      get_map_info_client_;
};
