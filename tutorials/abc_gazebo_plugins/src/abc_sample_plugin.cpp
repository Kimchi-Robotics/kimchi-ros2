#include "abc_gazebo_plugins/abc_sample_plugin.h"

namespace gazebo {

void ABCWorldSamplePlugin::Load(physics::WorldPtr _world,
                                sdf::ElementPtr _sdf) {
  world_ = _world;

  // Initialize gazebo node
  nh_ = gazebo_ros::Node::Get(_sdf);
  // Initialize publisher
  sec_publisher_ = nh_->create_publisher<std_msgs::msg::Int32>(
                    "secs", rclcpp::SystemDefaultsQoS());

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ABCWorldSamplePlugin::OnUpdate, this));
}

void ABCWorldSamplePlugin::OnUpdate() {
  common::Time time = world_->SimTime();
  std_msgs::msg::Int32 sec;
  sec.data = time.sec;

  sec_publisher_->publish(sec);
}

} // namespace gazebo
