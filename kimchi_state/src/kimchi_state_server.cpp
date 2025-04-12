#include <functional>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "kimchi_state/kimchi_state_server.h"
#include <kimchi_state/map_info.h>


KimchiStateServer::KimchiStateServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()): Node("kimchi_state_server", options), state_(RobotState::NO_MAP) {
  using namespace std::chrono_literals;
  // Create a QoS profile with best effort for sharing the state of the robot.
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10),
                         qos_profile);
  qos.best_effort();

  // Publish state.
  state_publisher_ = this->create_publisher<kimchi_interfaces::msg::RobotState>("/kimchi_state_server/state", qos);
  state_publisher_timer_ = this->create_wall_timer(1s, std::bind(&KimchiStateServer::statePublisherTimerCallback, this));

  // Subscribe to the map info service
  get_map_info_client_ = create_client<kimchi_interfaces::srv::MapInfo>("/kimchi_map/get_map_info");

  callGetMapInfoService();
}

void KimchiStateServer::statePublisherTimerCallback() {
  auto message = kimchi_interfaces::msg::RobotState();
  message.state = static_cast<uint32_t>(state_.load());
  this->state_publisher_->publish(message);
}

void KimchiStateServer::callGetMapInfoService()
{
  using namespace std::chrono_literals;

  while (!get_map_info_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<kimchi_interfaces::srv::MapInfo::Request>();
  request->str_place_holder = "May you share the mapo info please?";

  auto future = get_map_info_client_->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = future.get();
    if(result->success) {
      map_info_ = std::make_unique<MapInfo>(result->resolution, result->origin, result->map_image);
      state_ = RobotState::IDLE;
      RCLCPP_INFO(this->get_logger(), "MapInfo received");
    } else {
      RCLCPP_INFO(this->get_logger(), "MapInfo Service returned empty map");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /kimchi_map/get_map_info");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KimchiStateServer>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}