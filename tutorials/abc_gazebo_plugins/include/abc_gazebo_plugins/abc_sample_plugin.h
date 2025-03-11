// Gazebo stuff
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
// ROS stuff
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace gazebo {
class ABCWorldSamplePlugin : public WorldPlugin {
public:
  ABCWorldSamplePlugin() = default;

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

private:
  /// \brief ROS node
  gazebo_ros::Node::SharedPtr nh_{ nullptr };
  /// \brief ROS publisher
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr sec_publisher_;
  /// \brief World pointer
  physics::WorldPtr world_{ nullptr };
  /// \brief Conection pointer to update world function
  event::ConnectionPtr update_connection_;
  /// \brief Update world function
  void OnUpdate();
};

// Register the plugin
GZ_REGISTER_WORLD_PLUGIN(ABCWorldSamplePlugin)
} // namespace gazebo
