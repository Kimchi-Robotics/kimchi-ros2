// BSD 3-Clause License
//
// Copyright (c) 2023, Ekumen Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "kimchi_base/diffdrive_kimchi.h"

#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace kimchi_base {

hardware_interface::CallbackReturn DiffDriveKimchi::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "On init...");

  // Wheel names.
  config_.left_wheel_name = info_.hardware_parameters[kLeftWheelNameParam];
  RCLCPP_DEBUG(logger_, (kLeftWheelNameParam + static_cast<std::string>(": ") + config_.left_wheel_name).c_str());
  config_.right_wheel_name = info_.hardware_parameters[kRightWheelNameParam];
  RCLCPP_DEBUG(logger_, (kRightWheelNameParam + static_cast<std::string>(": ") + config_.right_wheel_name).c_str());

  // Bumper and button names.
  config_.left_bumper_name = info_.hardware_parameters[kLeftBumperNameParam];
  RCLCPP_DEBUG(logger_, (kLeftBumperNameParam + static_cast<std::string>(": ") + config_.left_bumper_name).c_str());
  config_.right_bumper_name = info_.hardware_parameters[kRightBumperNameParam];
  RCLCPP_DEBUG(logger_, (kRightBumperNameParam + static_cast<std::string>(": ") + config_.right_bumper_name).c_str());
  config_.button_name = info_.hardware_parameters[kButtonNameParam];
  RCLCPP_DEBUG(logger_, (kButtonNameParam + static_cast<std::string>(": ") + config_.button_name).c_str());

  // Serial communication parameters.
  config_.serial_device = info_.hardware_parameters[kSerialDeviceParam];
  RCLCPP_DEBUG(logger_, (kSerialDeviceParam + static_cast<std::string>(": ") + config_.serial_device).c_str());
  config_.baud_rate = std::stoi(info_.hardware_parameters[kBaudRateParam]);
  RCLCPP_DEBUG(logger_,
               (kBaudRateParam + static_cast<std::string>(": ") + info_.hardware_parameters[kBaudRateParam]).c_str());
  config_.timeout = std::stoi(info_.hardware_parameters[kTimeoutParam]);
  RCLCPP_DEBUG(logger_,
               (kTimeoutParam + static_cast<std::string>(": ") + info_.hardware_parameters[kTimeoutParam]).c_str());
  config_.enc_ticks_per_rev = std::stoi(info_.hardware_parameters[kEncTicksPerRevParam]);
  RCLCPP_DEBUG(logger_,
               (kEncTicksPerRevParam + static_cast<std::string>(": ") + info_.hardware_parameters[kEncTicksPerRevParam])
                   .c_str());

  for (const hardware_interface::ComponentInfo& joint : info.joints) {
    bool is_wheel = (joint.name.find("wheel") != std::string::npos);

    // Set expected interface counts based on joint type.
    // Wheels have 1 command interface (velocity) and 2 state interfaces (position and velocity).
    // Bumpers and button have no command interfaces and only 1 state interfaces (position).
    size_t expected_command_interfaces = is_wheel ? 1 : 0;
    size_t expected_state_interfaces = is_wheel ? 2 : 1;

    // Validate command interfaces
    if (joint.command_interfaces.size() != expected_command_interfaces) {
      RCLCPP_FATAL(logger_, "Joint '%s' has %zu command interfaces found. %zu expected.", joint.name.c_str(),
                   joint.command_interfaces.size(), expected_command_interfaces);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate state interfaces
    if (joint.state_interfaces.size() != expected_state_interfaces) {
      RCLCPP_FATAL(logger_, "Joint '%s' has %zu state interfaces found. %zu expected.", joint.name.c_str(),
                   joint.state_interfaces.size(), expected_state_interfaces);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Set up the wheels
  left_wheel_.Setup(config_.left_wheel_name, config_.enc_ticks_per_rev);
  right_wheel_.Setup(config_.right_wheel_name, config_.enc_ticks_per_rev);

  RCLCPP_INFO(logger_, "Finished On init.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveKimchi::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "On configure...");

  // Set up communication with motor driver controller.
  motor_driver_.Setup(config_.serial_device, config_.baud_rate, config_.timeout);

  RCLCPP_INFO(logger_, "Finished Configuration");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveKimchi::export_state_interfaces() {
  // We need to set up a position and a velocity interface for each wheel
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Add left and right wheel interfaces
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(left_wheel_.name_, hardware_interface::HW_IF_POSITION, &left_wheel_.pos_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(right_wheel_.name_, hardware_interface::HW_IF_POSITION, &right_wheel_.pos_));

  // Add bumper and button interfaces
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(config_.left_bumper_name, hardware_interface::HW_IF_POSITION, &left_bumper_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(config_.right_bumper_name,
                                                                   hardware_interface::HW_IF_POSITION, &right_bumper_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(config_.button_name, hardware_interface::HW_IF_POSITION, &button_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveKimchi::export_command_interfaces() {
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd_));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveKimchi::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(logger_, "On activate...");
  RCLCPP_INFO(logger_, "Finished Activation");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveKimchi::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(logger_, "On deactivate...");
  RCLCPP_INFO(logger_, "Finished Deactivation");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveKimchi::read(const rclcpp::Time& /* time */, const rclcpp::Duration& period) {
  const double delta_secs = period.seconds();

  if (!motor_driver_.is_connected()) {
    RCLCPP_ERROR(logger_, "Motor driver is not connected.");
    return hardware_interface::return_type::ERROR;
  }

  try {
    const std::optional<MotorDriver::HardwareData> hardware_data = motor_driver_.ReadHardwareData();
    left_wheel_.enc_ = (*hardware_data)[0];
    right_wheel_.enc_ = (*hardware_data)[1];

    // Convert int (0/1) to double since the state interface expects a double for the position.
    left_bumper_ = static_cast<double>((*hardware_data)[2]);
    right_bumper_ = static_cast<double>((*hardware_data)[3]);
    button_ = static_cast<double>((*hardware_data)[4]);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception while reading hardware data: %s", e.what());
    return hardware_interface::return_type::OK;
  }

  const double left_pos_prev = left_wheel_.pos_;
  left_wheel_.pos_ = left_wheel_.Angle();
  left_wheel_.vel_ = (left_wheel_.pos_ - left_pos_prev) / delta_secs;

  const double right_pos_prev = right_wheel_.pos_;
  right_wheel_.pos_ = right_wheel_.Angle();
  right_wheel_.vel_ = (right_wheel_.pos_ - right_pos_prev) / delta_secs;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveKimchi::write(const rclcpp::Time& /* time */,
                                                       const rclcpp::Duration& /* period */) {
  if (!motor_driver_.is_connected()) {
    RCLCPP_ERROR(logger_, "Motor driver is not connected.");
    return hardware_interface::return_type::ERROR;
  }

  // The command is in rad/sec (rps), we need to convert it to ticks/sec (tps)
  // Using the rads per tick(rpt) of the motor information
  // Formula: ticks/sec = rads/sec / rads/tick
  const int left_value_target = static_cast<int>(left_wheel_.cmd_ / left_wheel_.rads_per_tick_);
  const int right_value_target = static_cast<int>(right_wheel_.cmd_ / right_wheel_.rads_per_tick_);

  motor_driver_.SetMotorValues(left_value_target, right_value_target);

  return hardware_interface::return_type::OK;
}

}  // namespace kimchi_base

PLUGINLIB_EXPORT_CLASS(kimchi_base::DiffDriveKimchi, hardware_interface::SystemInterface)
