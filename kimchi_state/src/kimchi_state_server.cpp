#include "kimchi_state/kimchi_state_server.h"

#include <kimchi_state/map_info.h>

#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <filesystem>

namespace {
std::string toString(RobotState robot_state) {
  switch (robot_state) {
    case RobotState::NO_MAP:
      return "NO_MAP";
    case RobotState::MAPPING_WITH_EXPLORATION:
      return "MAPPING_WITH_EXPLORATION";
    case RobotState::MAPPING_WITH_TELEOP:
      return "MAPPING_WITH_TELEOP";
    case RobotState::NAVIGATING:
      return "NAVIGATING";
    case RobotState::LOCATING:
      return "LOCATING";
    case RobotState::TELEOP:
      return "TELEOP";
    case RobotState::IDLE:
      return "IDLE";
    case RobotState::LOST:
      return "LOST";
    case RobotState::RECOVERING:
      return "RECOVERING";
    case RobotState::GOAL_REACHED:
      return "GOAL_REACHED";
    case RobotState::CHARGING:
      return "CHARGING";
  }
  return "UNKNOWN_STATE";
}
}  // namespace

std::shared_ptr<KimchiStateServer> KimchiStateServer::Create(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) {
  auto output =
      std::shared_ptr<KimchiStateServer>(new KimchiStateServer(options));
  output->initialize();
  return output;
}

void KimchiStateServer::onNavigatingToGoal(const Point2D &point) {
  changeState(RobotState::NAVIGATING);
  RCLCPP_INFO(node_->get_logger(), "Navigating to goal at point: (%f, %f)",
              point.x, point.y);
}

void KimchiStateServer::onGoalReached(const Point2D &point) {
  changeState(RobotState::GOAL_REACHED);
  RCLCPP_INFO(node_->get_logger(), "Goal reached at point: (%f, %f)", point.x,
              point.y);
}

void KimchiStateServer::onMissionFinished() {
  changeState(RobotState::IDLE);
  RCLCPP_INFO(node_->get_logger(), "Mission finished");
}

void KimchiStateServer::initialize() {
  RCLCPP_INFO(node_->get_logger(), "KimchiStateServer::initialize.");

  navigation_manager_ =
      std::make_unique<NavigationManager>(node_, shared_from_this());
  // Create a QoS profile with best effort for sharing the state of the robot.
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10),
                         qos_profile);
  qos.best_effort();

  // Publish state.
  state_publisher_ =
      node_->create_publisher<kimchi_interfaces::msg::RobotState>(
          "/kimchi_state_server/state", qos);
  state_publisher_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&KimchiStateServer::statePublisherTimerCallback, this));

  // Subscribe to the map info service
  get_map_info_client_ = node_->create_client<kimchi_interfaces::srv::MapInfo>(
      "/kimchi_map/get_map_info");

  // Subscribe to the save map service
  save_map_client_ =
      node_->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");

  start_slam_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/kimchi_state_server/start_slam",
      std::bind(&KimchiStateServer::startSlamCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  start_navigation_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/kimchi_state_server/start_navigation",
      std::bind(&KimchiStateServer::startNavigationCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  add_goal_to_mission_service_ =
      node_->create_service<kimchi_interfaces::srv::AddGoalToMission>(
          "/kimchi_state_server/add_goal_to_mission",
          std::bind(&KimchiStateServer::addGoalToMissionCallback, this,
                    std::placeholders::_1, std::placeholders::_2));

  // Call the map info service
  callGetMapInfoService();
}

KimchiStateServer::KimchiStateServer(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : node_(new rclcpp::Node("kimchi_state_server", options)),
      navigation_manager_(nullptr),
      state_(RobotState::NO_MAP) {}

void KimchiStateServer::statePublisherTimerCallback() {
  auto message = kimchi_interfaces::msg::RobotState();
  message.state = static_cast<uint32_t>(state_.load());
  state_publisher_->publish(message);
}

void KimchiStateServer::callGetMapInfoService() {
  while (!get_map_info_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<kimchi_interfaces::srv::MapInfo::Request>();
  request->str_place_holder = "May you share the map info please?";

  auto future = get_map_info_client_->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                         future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    if (result->success) {
      map_info_ = std::make_unique<MapInfo>(result->resolution, result->origin,
                                            result->map_image);
      changeState(RobotState::IDLE);
      navigation_manager_->startNavigation();
      RCLCPP_INFO(node_->get_logger(), "MapInfo received");
    } else {
      changeState(RobotState::NO_MAP);
      RCLCPP_INFO(node_->get_logger(), "MapInfo Service returned empty map");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service /kimchi_map/get_map_info");
  }
}

void KimchiStateServer::startSlamCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  changeState(RobotState::MAPPING_WITH_TELEOP);

  navigation_manager_->startSlam();
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

  response->success = true;
  if (state_ == RobotState::MAPPING_WITH_TELEOP) {
    saveMap();
    navigation_manager_->stopSlam();
    return;
  }

  navigation_manager_.startNavigation();
  changeState(RobotState::IDLE);
}

void KimchiStateServer::addGoalToMissionCallback(
    const kimchi_interfaces::srv::AddGoalToMission::Request::SharedPtr request,
    kimchi_interfaces::srv::AddGoalToMission::Response::SharedPtr response) {
  navigation_manager_->addGoalToMission(
      Point2D(request->goal.x, request->goal.y));
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

  // TODO(arilow): Handle the response by passing a callback to
  // async_send_request  
  auto future = save_map_client_->async_send_request(
      request, [this](std::shared_future<nav2_msgs::srv::SaveMap::Response::
                                             SharedPtr> /*response_future*/) {
        auto map_server_param_client_ =
            std::make_shared<rclcpp::AsyncParametersClient>(node_,
                                                            "/map_server");
        if (map_server_param_client_->wait_for_service(
                std::chrono::seconds(1))) {
          auto future =
              map_server_param_client_->set_parameters({rclcpp::Parameter(
                  "yaml_filename", std::filesystem::absolute("kimchi_map.yaml").c_str())});
          // Handle future result...
        }

        navigation_manager_->startNavigation();
        changeState(RobotState::IDLE);
      });
}

void KimchiStateServer::changeState(RobotState new_state) {
  state_ = new_state;
  RCLCPP_INFO(node_->get_logger(), "State changed to: %s",
              toString(state_).c_str());
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // auto kimchi_state_server =
  //     std::make_shared<KimchiStateServer>(rclcpp::NodeOptions());
  std::shared_ptr<KimchiStateServer> kimchi_state_server =
      KimchiStateServer::Create(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4);

  executor.add_node(kimchi_state_server->getNode());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
