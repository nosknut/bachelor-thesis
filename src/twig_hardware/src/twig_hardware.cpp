// https://github.com/ros-controls/ros2_control_demos/blob/humble/example_12/hardware/rrbot.cpp

// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "twig_hardware/twig_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace twig_hardware
{
hardware_interface::CallbackReturn TwigHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.resize(info_.joints.size() * 1, std::numeric_limits<double>::quiet_NaN());
  hw_states_.resize(info_.joints.size() * 3, std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // TwigHardware has exactly 1 command and 3 state interfaces on each joint

    // Validate command interface configs

    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("TwigHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate command interface 1 config

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("TwigHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate state interface configs

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("TwigHardware"),
        "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate state interface 1 config

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("TwigHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate state interface 2 config

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("TwigHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate state interface 3 config

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        rclcpp::get_logger("TwigHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  
  // Configure the pull and push rates
  // https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/different_update_rates_userdoc.html#by-measuring-elapsed-time
  pull_rate_ = stod(info_.hardware_parameters["pull_rate"]);
  push_rate_ = stod(info_.hardware_parameters["push_rate"]);


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TwigHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = 0;
  }

  for (uint i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = 0;
  }

  // Docs for hardware params:
  // https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/different_update_rates_userdoc.html
  
  // Load all twig driver hardware configurations from hardware params
  TwigHardwareConfig c;
  c.connectionTimeout = stoi(info_.hardware_parameters["hardware_config.connection_timeout"]);
  
  c.maxShoulderCurrent = twig.current_to_raw_psm(stod(info_.hardware_parameters["hardware_config.max_current"]));
  c.maxWristCurrent = twig.current_to_raw_psm(stod(info_.hardware_parameters["hardware_config.max_current"]));
  c.maxGripperCurrent = twig.current_to_raw_acs(stod(info_.hardware_parameters["hardware_config.max_current"]));
  c.maxCurrentDuration = stoi(info_.hardware_parameters["hardware_config.max_current_duration"]);
  c.maxCurrentCooldownDuration = stoi(info_.hardware_parameters["hardware_config.max_current_cooldown_duration"]);
  c.encoderMinMagnitude = stoi(info_.hardware_parameters["hardware_config.encoder_min_magnitude"]);
  twig.set_hardware_config(c);

  // Load all twig driver joint configurations from hardware params
  twig.jointConfig.shoulderLimits.min = stod(info_.hardware_parameters["joint_config.shoulder.limits.min"]);
  twig.jointConfig.shoulderLimits.max = stod(info_.hardware_parameters["joint_config.shoulder.limits.max"]);

  twig.jointConfig.gripperLimits.min = stod(info_.hardware_parameters["joint_config.gripper.limits.min"]);
  twig.jointConfig.gripperLimits.max = stod(info_.hardware_parameters["joint_config.gripper.limits.max"]);

  twig.jointConfig.shoulderOffset = stod(info_.hardware_parameters["joint_config.shoulder.offset"]);
  twig.jointConfig.wristOffset = stod(info_.hardware_parameters["joint_config.wrist.offset"]);
  twig.jointConfig.gripperOffset = stod(info_.hardware_parameters["joint_config.gripper.offset"]);

  // Push the configurations to hardware
  if (twig.write_command()) {
    RCLCPP_INFO(rclcpp::get_logger("TwigHardware"), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  RCLCPP_ERROR(rclcpp::get_logger("TwigHardware"), "Failed to configure hardware!");
  return hardware_interface::CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface>
TwigHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TwigHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn TwigHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{ \
  twig.activate_all_servos();
  
  // Configure the first read and write pass to respect the push and pull rates
  // https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/different_update_rates_userdoc.html#by-measuring-elapsed-time
  first_read_pass_ = true;
  first_write_pass_ = true;

  if (twig.write_command()) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  RCLCPP_ERROR(rclcpp::get_logger("TwigHardware"), "Failed to activate servos!");
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn TwigHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  twig.deactivate_all_servos();

  if (twig.write_command()) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  RCLCPP_ERROR(rclcpp::get_logger("TwigHardware"), "Failed to deactivate servos!");
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type TwigHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Skip reads to respect the pull rate
  // https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/different_update_rates_userdoc.html#by-measuring-elapsed-time
  if (!first_read_pass_ && ((time - last_read_time_ ) < rclcpp::Duration::from_seconds(1.0 / pull_rate_)))
  {
    return hardware_interface::return_type::OK;
  }

  first_read_pass_ = false;
  last_read_time_ = time;
  
  if (twig.read_state(3)) {
    if (twig.driver_rebooted()) {
      twig.acknowledge_hardware_reboot();
      RCLCPP_DEBUG(rclcpp::get_logger("TwigHardware"), "Driver rebooted");
    }
    if (twig.hardware_rebooted()) {
      RCLCPP_WARN(rclcpp::get_logger("TwigHardware"), "Hardware rebooted");
      twig.acknowledge_hardware_reboot();
      RCLCPP_INFO(
        rclcpp::get_logger("TwigHardware"),
        "Hardware reboot was automatically acknowledged and the system will continue normal operation");
    }

    hw_states_[0] = twig.get_shoulder_servo_position();
    hw_states_[1] = twig.get_shoulder_servo_velocity();
    hw_states_[2] = twig.get_shoulder_servo_effort();

    hw_states_[3] = twig.get_wrist_servo_position();
    hw_states_[4] = twig.get_wrist_servo_velocity();
    hw_states_[5] = twig.get_wrist_servo_effort();

    hw_states_[6] = twig.get_gripper_servo_position();
    hw_states_[7] = twig.get_gripper_servo_velocity();
    hw_states_[8] = twig.get_gripper_servo_effort();

    return hardware_interface::return_type::OK;
  }
  RCLCPP_ERROR(rclcpp::get_logger("TwigHardware"), "Failed to read state from hardware!");
  return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type TwigHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Skip writes to respect the pull rate
  // https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/different_update_rates_userdoc.html#by-measuring-elapsed-time
  if (!first_write_pass_ && ((time - last_write_time_ ) < rclcpp::Duration::from_seconds(1.0 / push_rate_)))
  {
    return hardware_interface::return_type::OK;
  }

  first_write_pass_ = false;
  last_write_time_ = time;

  twig.set_shoulder_servo_velocity(hw_commands_[0]);
  twig.set_wrist_servo_velocity(hw_commands_[1]);
  twig.set_gripper_servo_velocity(hw_commands_[2]);

  if (twig.write_command(3)) {
    return hardware_interface::return_type::OK;
  }
  RCLCPP_ERROR(rclcpp::get_logger("TwigHardware"), "Failed to write commands to hardware!");
  return hardware_interface::return_type::ERROR;
}

}  // namespace twig_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  twig_hardware::TwigHardware,
  hardware_interface::SystemInterface)
