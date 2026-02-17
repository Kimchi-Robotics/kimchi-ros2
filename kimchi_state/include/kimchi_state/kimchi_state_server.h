/**
 * Kimchi State Server
 */
#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <future>  // For std::promise and std::future
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kimchi_interfaces/msg/robot_state.hpp>
#include <kimchi_interfaces/srv/kimchi_state_server_command.hpp>
#include <kimchi_interfaces/srv/map_info.hpp>
#include <kimchi_interfaces/srv/proccess_selected_position.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <memory>
#include <nav2_lifecycle_manager/lifecycle_manager_client.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "bumper_obstacle_publisher.h"
#include "map_info.h"
#include "navigation_manager.h"
#include "point_2d.h"

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
  CHARGING,
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
  void onNav2LocalizationStarted() override;
  void onSlamStarted() override;
  void onLocalizationStarted() override;
  void onLocalizationCancelled() override;
  void onLocalizationSucceded() override;
  void onNavigatingToGoal(const Point2D& point) override;
  void onGoalReached(const Point2D& point) override;
  void onMissionFinished() override;
  void onGoalCancelled(const Point2D& point) override;
  void onMissionPaused() override;

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

  void startLocating(const Point2D& point);

  void statePublisherTimerCallback();
  /**
   * Timer callback to check the state of the localization action.
   * It checks if the robot has been localized and sets the state in accordance
   */
  void callGetMapInfoService();
  std::shared_future<nav2_msgs::srv::SaveMap::Response::SharedPtr> saveMap();
  std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> finishMap();
  void changeState(RobotState new_state);
  void startLocalization();
  void startNavigation();
  // Makes the robot go backwards for a bit, then asks the navigation manager to
  // resume the goal.
  void goBackAndResume();
  void onBumperPressed(bool left_bumper_pressed, bool right_bumper_pressed);

  // Callback methods for the services.
  void startSlamCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);
  void initialPoseCallback(
      const kimchi_interfaces::srv::ProccessSelectedPosition::Request::SharedPtr
          request,
      kimchi_interfaces::srv::ProccessSelectedPosition::Response::SharedPtr
          response);
  void startLocalizationCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);
  void addGoalToMissionCallback(
      const kimchi_interfaces::srv::ProccessSelectedPosition::Request::SharedPtr
          request,
      kimchi_interfaces::srv::ProccessSelectedPosition::Response::SharedPtr
          response);
  void sendCommandCallback(
      const kimchi_interfaces::srv::KimchiStateServerCommand::Request::SharedPtr
          request,
      kimchi_interfaces::srv::KimchiStateServerCommand::Response::SharedPtr
          response);
  void jointStatesCallback(const control_msgs::msg::DynamicJointState::SharedPtr msg);
  void getRobotPositionFromTF();
  void mockBumperCallback(
      const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
      std_srvs::srv::Trigger::Response::SharedPtr response);

  /**
   * AMCL pose callback
   * Callback method for pose published by AMCL.
   *
   * Verifies if the robot is localized by checking if the covariance of the
   * estimated amcl pose is within a set threshold
   */
  void AmclPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<BumperObstaclePublisher> bumper_obstacle_publisher_;
  std::unique_ptr<NavigationManager> navigation_manager_;
  std::atomic<RobotState> state_;
  std::unique_ptr<MapInfo> map_info_;

  std::optional<geometry_msgs::msg::Pose> inital_pose_estimate_;

  // Topics.
  rclcpp::TimerBase::SharedPtr state_publisher_timer_;
  rclcpp::Publisher<kimchi_interfaces::msg::RobotState>::SharedPtr
      state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr
      joint_states_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      amcl_pose_subscription_;

  // Service servers.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_slam_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_navigation_service_;
  rclcpp::Service<kimchi_interfaces::srv::ProccessSelectedPosition>::SharedPtr
      start_locating_service_;
  rclcpp::Service<kimchi_interfaces::srv::ProccessSelectedPosition>::SharedPtr
      add_goal_to_mission_service_;
  rclcpp::Service<kimchi_interfaces::srv::KimchiStateServerCommand>::SharedPtr
      send_command_service_;
  // Service to mock a bumper press. Meant to test the bumper behavior in
  // simulation.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mock_bumper_service_;

  // Service clients.
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_client_;
  rclcpp::Client<kimchi_interfaces::srv::MapInfo>::SharedPtr
      get_map_info_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr finish_map_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  // Variables for bumpers and button.
  bool left_bumper_pressed_ = false;
  bool right_bumper_pressed_ = false;
  bool button_pressed_ = false;
  Point2D current_robot_position_;
};
