/**
 * Kimchi State Server 
 */
#pragma once

#include <atomic>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <kimchi_interfaces/srv/map_info.hpp>
#include <kimchi_interfaces/msg/robot_state.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "map_info.h"

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
  

  rclcpp::TimerBase::SharedPtr state_publisher_timer_;
  rclcpp::Publisher<kimchi_interfaces::msg::RobotState>::SharedPtr state_publisher_;


  // TODO Service to start mapping.
  // TODO Service to start navigation.
  rclcpp::Client<kimchi_interfaces::srv::MapInfo>::SharedPtr get_map_info_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<RobotState> state_;

  std::unique_ptr<MapInfo> map_info_;
};
 