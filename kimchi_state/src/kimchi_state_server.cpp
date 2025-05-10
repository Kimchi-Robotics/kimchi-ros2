#include "kimchi_state/kimchi_state_server.h"

#include <kimchi_state/map_info.h>

#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

KimchiStateServer::KimchiStateServer(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("kimchi_state_server", options), state_(RobotState::NO_MAP) {
  using namespace std::chrono_literals;
  // Create a QoS profile with best effort for sharing the state of the robot.
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10),
                         qos_profile);
  qos.best_effort();

  // Publish state.
  state_publisher_ = this->create_publisher<kimchi_interfaces::msg::RobotState>(
      "/kimchi_state_server/state", qos);
  state_publisher_timer_ = this->create_wall_timer(
      1s, std::bind(&KimchiStateServer::statePublisherTimerCallback, this));

  // Subscribe to the map info service
  get_map_info_client_ = create_client<kimchi_interfaces::srv::MapInfo>(
      "/kimchi_map/get_map_info");

      // Subscribe to the save map service
  save_map_client_ = create_client<nav2_msgs::srv::SaveMap>(
    "/map_saver/save_map");

  start_slam_service_ = create_service<std_srvs::srv::Trigger>("/kimchi_state_server/start_slam",
                            std::bind(&KimchiStateServer::startSlamCallback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));
  active_slam_toolbox_node_client_ = create_client<lifecycle_msgs::srv::ChangeState>("/slam_toolbox/change_state");

  start_navigation_service_ = create_service<std_srvs::srv::Trigger>("/kimchi_state_server/start_navigation",
    std::bind(&KimchiStateServer::startNavigationCallback, this,
              std::placeholders::_1,
              std::placeholders::_2));


  // Call the map info service
  callGetMapInfoService();
}

void KimchiStateServer::statePublisherTimerCallback() {
  auto message = kimchi_interfaces::msg::RobotState();
  message.state = static_cast<uint32_t>(state_.load());
  this->state_publisher_->publish(message);
}

void KimchiStateServer::callGetMapInfoService() {
  using namespace std::chrono_literals;

  while (!get_map_info_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<kimchi_interfaces::srv::MapInfo::Request>();
  request->str_place_holder = "May you share the mapo info please?";

  auto future = get_map_info_client_->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    if (result->success) {
      map_info_ = std::make_unique<MapInfo>(result->resolution, result->origin,
                                            result->map_image);
      state_ = RobotState::IDLE;
      startNavigation();
      RCLCPP_INFO(this->get_logger(), "MapInfo received");
    } else {
      state_ = RobotState::NO_MAP;
      RCLCPP_INFO(this->get_logger(), "MapInfo Service returned empty map");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service /kimchi_map/get_map_info");
  }
}

void KimchiStateServer::startSlamCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  state_ = RobotState::MAPPING_WITH_TELEOP;
  RCLCPP_INFO(this->get_logger(), "Waiting for slam_toolbox service");
  active_slam_toolbox_node_client_->wait_for_service();
  RCLCPP_INFO(this->get_logger(), "Finished waiting for slam_toolbox service");
  auto new_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  new_request->transition.id = 3;  // Activate
  active_slam_toolbox_node_client_->async_send_request(new_request);
  RCLCPP_INFO(this->get_logger(), "Starting SLAM");
  response->success = true;
}

void KimchiStateServer::startNavigationCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response) {

  if (state_ == RobotState::NO_MAP) {
    response->success = false;
    response->message = "There's no map available. Can't start navigation.";
    return;
  }

  if (state_ == RobotState::MAPPING_WITH_TELEOP) {
    saveMap();
    stopSlam();
  }

  state_ = RobotState::IDLE;
  RCLCPP_INFO(this->get_logger(), "Starting Navigation");

  startNavigation();
  response->success = true;
}

void KimchiStateServer::saveMap() {
  save_map_client_->wait_for_service();
  auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();

  request->map_topic = "/map";
  request->map_url = "kimchi_map";
  request->image_format = "png";
  request->map_mode = "trinary";
  request->free_thresh = 0.25;
  request->occupied_thresh = 0.65;

  // TODO(arilow): Handle the response by passing a callback to async_send_request
  auto future = save_map_client_->async_send_request(request);
}

void KimchiStateServer::stopSlam() {
  active_slam_toolbox_node_client_->wait_for_service();
  auto new_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  new_request->transition.id = 4;  // Deactivate
  // TODO(arilow): Handle the response by passing a callback to async_send_request
  active_slam_toolbox_node_client_->async_send_request(new_request);
}

void KimchiStateServer::startNavigation() {
  // TODO: call active_navigation_node_client_
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KimchiStateServer>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
